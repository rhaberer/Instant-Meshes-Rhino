"""Trace the pinned algorithms from scratch builds or opt in to a separate score oracle."""
from collections import Counter
from fractions import Fraction
import struct


def insert(path, marker, code):
    source = path.read_text()
    assert source.count(marker) == 1
    path.write_text(source.replace(marker, code + marker))


def instrument_point_cloud(scratch):
    bvh = scratch / "vendor/instant-meshes/src/bvh.cpp"
    bvh.write_text("#include <fstream>\n" + bvh.read_text())
    insert(bvh, '            SHOW_PROGRESS_RANGE(range, mV->cols(), "Assigning disk radius");', """
            if (const char *path = std::getenv("IM_POINT_RADIUS_BIN")) {
                std::ofstream stream(path, std::ios::binary | std::ios::app);
                uint32_t span[] = {range.begin(), range.end()};
                stream.write(reinterpret_cast<const char*>(span), sizeof(span));
                stream.write(reinterpret_cast<const char*>(&radius_sum), sizeof(radius_sum));
            }
""")
    driver = scratch / "native/instant_meshes_api.cpp"
    insert(driver, "        adj = generate_adjacency_matrix_pointcloud", """
        if (const char *path = std::getenv("IM_POINT_CANDIDATES_BIN")) {
            std::ofstream stream(path, std::ios::binary);
            std::vector<std::pair<Float, uint32_t>> hits;
            for (uint32_t i = 0; i < V.cols(); ++i) {
                Float radius = bvh->diskRadius() * 3;
                bvh->findKNearest(V.col(i), N.col(i), options.knn_points, radius, hits);
                uint32_t count = hits.size();
                stream.write(reinterpret_cast<const char*>(&count), sizeof(count));
                for (auto hit : hits)
                    stream.write(reinterpret_cast<const char*>(&hit.second), sizeof(hit.second));
            }
        }
""")
    insert(driver, '    mRes.setAdj(std::move(adj));', """
    if (pointcloud) {
        if (const char *path = std::getenv("IM_POINT_NEIGHBORS_BIN")) {
            std::ofstream stream(path, std::ios::binary);
            double measures[] = {stats.mSurfaceArea, stats.mAverageEdgeLength};
            stream.write(reinterpret_cast<const char*>(measures), sizeof(measures));
            for (int i = 0; i <= V.cols(); ++i) {
                uint32_t offset = adj[i] - adj[0];
                stream.write(reinterpret_cast<const char*>(&offset), sizeof(offset));
            }
            for (auto link = adj[0]; link != adj[V.cols()]; ++link)
                stream.write(reinterpret_cast<const char*>(&link->id), sizeof(link->id));
            stream.write(reinterpret_cast<const char*>(V.data()), V.size() * sizeof(Float));
            stream.write(reinterpret_cast<const char*>(N.data()), N.size() * sizeof(Float));
        }
    }
""")


def instrument_raw_scores(hierarchy):
    insert(hierarchy, "    if (deterministic)\n        pss::parallel_stable_sort", """
    if (const char *path = std::getenv("IM_RAW_SCORES_BIN")) {
        std::ofstream stream(path, std::ios::binary | std::ios::app);
        uint32_t vertices = V.cols();
        stream.write(reinterpret_cast<const char*>(&vertices), sizeof(vertices));
        stream.write(reinterpret_cast<const char*>(&nLinks), sizeof(nLinks));
        for (uint32_t index = 0; index < nLinks; ++index) {
            const auto &e = entries[index];
            stream.write(reinterpret_cast<const char*>(&e.i), sizeof(e.i));
            stream.write(reinterpret_cast<const char*>(&e.j), sizeof(e.j));
            stream.write(reinterpret_cast<const char*>(&e.order_numerator), sizeof(e.order_numerator));
            stream.write(reinterpret_cast<const char*>(&e.order_denominator), sizeof(e.order_denominator));
        }
    }
""")


def raw_score_evidence(raw_scores, sorted_candidates):
    data = raw_scores.read_bytes()
    vertices, count = struct.unpack_from("<II", data)
    keys = []
    summary = Counter()

    def score_class(entry):
        _, numerator, denominator = entry
        if denominator:
            return "finite"
        return "positiveOverZero" if numerator > 0 else "negativeOverZero" if numerator < 0 else "zeroOverZero"

    for offset in range(8, 8 + 32 * count, 32):
        a, b = struct.unpack_from("<II", data, offset)
        numerator = int.from_bytes(data[offset + 8:offset + 24], "little", signed=True)
        denominator, = struct.unpack_from("<q", data, offset + 24)
        entry = ((a, b), numerator, denominator)
        keys.append(entry)
        summary[score_class(entry)] += 1
    by_edge = {entry[0]: entry for entry in keys}

    def less(a, b):
        lhs, rhs = a[1] * b[2], b[1] * a[2]
        return lhs > rhs if lhs != rhs else a[0] < b[0]

    def describe(entry):
        edge, numerator, denominator = entry
        return {"edge": edge, "numerator": numerator, "denominator": denominator}

    sorted_data = sorted_candidates.read_bytes()
    assert struct.unpack_from("<I", sorted_data)[0] == count
    ordered = [by_edge[(a, b)] for a, b, _ in struct.iter_unpack("<IIf", sorted_data[4:4 + 12 * count])]
    # Check earlier candidates of every score class. Endpoint maxima cover
    # the cross-products that tie on a zero denominator. A finite score minimum
    # covers numeric inversions within the finite class.
    last_by_endpoint = {}
    minimum_finite = None
    inversion = None
    for index, entry in enumerate(ordered):
        witnesses = list(last_by_endpoint.values())
        if minimum_finite:
            witnesses.append(minimum_finite[1])
        earlier = next(((i, key) for i, key in witnesses if less(entry, key)), None)
        if earlier:
            i, key = earlier
            inversion = {"leftIndex": i, "left": describe(key), "rightIndex": index, "right": describe(entry)}
            break
        endpoint, numerator, denominator = entry
        category = score_class(entry)
        if category not in last_by_endpoint or endpoint > last_by_endpoint[category][1][0]:
            last_by_endpoint[category] = (index, entry)
        if denominator:
            key = (Fraction(numerator, denominator), -endpoint[0], -endpoint[1])
            if minimum_finite is None or key < minimum_finite[0]:
                minimum_finite = (key, (index, entry))

    # For 0/0, the original comparator falls back to endpoint order against
    # every finite score. Find the earliest zero key between an increasing pair
    # of finite scores, which gives a strict three-edge cycle.
    by_endpoint = sorted(keys)
    suffix = []
    largest = None
    for entry in reversed(by_endpoint):
        suffix.append(largest)
        if entry[2]:
            score = Fraction(entry[1], entry[2])
            if largest is None or score > largest[0]:
                largest = (score, entry)

    smallest = None
    cycle = None
    for entry, next_finite in zip(by_endpoint, reversed(suffix)):
        if entry[2]:
            score = Fraction(entry[1], entry[2])
            if smallest is None or score < smallest[0]:
                smallest = (score, entry)
        elif entry[1] == 0 and smallest and next_finite and smallest[0] < next_finite[0]:
            a, b, c = smallest[1], entry, next_finite[1]
            assert less(a, b) and less(b, c) and less(c, a)
            cycle = [describe(key) for key in (a, b, c)]
            break
    return {"vertices": vertices, "candidateCount": count, "scoreClasses": dict(summary),
            "firstOutputInversion": inversion, "strictOrderCycle": cycle}
