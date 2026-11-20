// Diagnostic executable linked to the pinned reference. Excluded from the plugin.
#define main unused_parity_main
#include "parity_main.cpp"
#undef main

#include "adjacency.h"
#include "dedge.h"
#include "fixed_point.h"
#include "meshstats.h"
#include "hierarchy.h"
#include "normal.h"
#include "subdivide.h"

extern bool im_browser_stable;

int main(int argc, char **argv) {
    if (argc != 4 && argc != 5 && argc != 6) {
        std::fprintf(stderr, "usage: preprocessing input.obj output.bin stable [max-edge-length | hierarchy target-vertices]\n");
        return 2;
    }

    Mesh mesh;
    if (!load_obj(argv[1], mesh) || mesh.indices.empty())
        return 2;

    im_browser_stable = std::atoi(argv[3]) != 0;
    MatrixXf positions = Eigen::Map<const MatrixXf>(mesh.positions.data(), 3, mesh.positions.size() / 3);
    MatrixXu faces = Eigen::Map<const MatrixXu>(mesh.indices.data(), 3, mesh.indices.size() / 3);
    if (im_browser_stable)
        for (Eigen::Index i = 0; i < positions.size(); ++i)
            positions.data()[i] = instant_meshes_port::canonicalFloat(positions.data()[i]);

    bool hierarchy = argc == 6;
    MeshStats stats;
    Float scale = -1;
    if (hierarchy) {
        stats = compute_mesh_stats(faces, positions, true);
        scale = std::sqrt((Float)(stats.mSurfaceArea / std::atoi(argv[5])));
    }
    Float maxEdgeLength = argc == 5 ? std::strtof(argv[4], nullptr) : -1;
    if (hierarchy && (stats.mMaximumEdgeLength * 2 > scale ||
                      stats.mMaximumEdgeLength > stats.mAverageEdgeLength * 2))
        maxEdgeLength = std::min(scale / 2, (Float)stats.mAverageEdgeLength * 2);

    VectorXu vertexEdges, oppositeEdges;
    VectorXb boundary, nonManifold;
    MatrixXf normals;
    VectorXf areas;
    build_dedge(faces, positions, vertexEdges, oppositeEdges, boundary, nonManifold);
    if (maxEdgeLength > 0) {
        subdivide(faces, positions, vertexEdges, oppositeEdges, boundary, nonManifold,
                  maxEdgeLength, true);
        if (im_browser_stable)
            for (Eigen::Index i = 0; i < positions.size(); ++i)
                positions.data()[i] = instant_meshes_port::canonicalFloat(positions.data()[i]);
        build_dedge(faces, positions, vertexEdges, oppositeEdges, boundary, nonManifold);
    }
    auto neighbors = generate_adjacency_matrix_uniform(faces, vertexEdges, oppositeEdges, nonManifold);
    generate_smooth_normals(faces, positions, vertexEdges, oppositeEdges, nonManifold, normals);
    compute_dual_vertex_areas(faces, positions, vertexEdges, oppositeEdges, nonManifold, areas);

    if (hierarchy && im_browser_stable) {
        for (Eigen::Index i = 0; i < normals.size(); ++i)
            normals.data()[i] = instant_meshes_port::canonicalFloat(normals.data()[i]);
        for (Eigen::Index i = 0; i < areas.size(); ++i)
            areas[i] = instant_meshes_port::canonicalFloat(areas[i]);
    }

    std::ofstream stream(argv[2], std::ios::binary);
    auto write = [&](const void *data, size_t bytes) {
        stream.write(static_cast<const char *>(data), bytes);
    };
    auto number = [&](uint32_t value) { write(&value, sizeof(value)); };
    number(static_cast<uint32_t>(positions.cols()));
    number(static_cast<uint32_t>(faces.size()));
    write(positions.data(), positions.size() * sizeof(float));
    write(faces.data(), faces.size() * sizeof(uint32_t));
    write(vertexEdges.data(), vertexEdges.size() * sizeof(uint32_t));
    write(oppositeEdges.data(), oppositeEdges.size() * sizeof(uint32_t));
    for (Eigen::Index vertex = 0; vertex < positions.cols(); ++vertex) {
        number(boundary[vertex]);
        number(nonManifold[vertex]);
    }
    write(normals.data(), normals.size() * sizeof(float));
    write(areas.data(), areas.size() * sizeof(float));
    for (Eigen::Index vertex = 0; vertex <= positions.cols(); ++vertex)
        number(static_cast<uint32_t>(neighbors[vertex] - neighbors[0]));
    for (auto link = neighbors[0]; link != neighbors[positions.cols()]; ++link)
        number(link->id);
    if (hierarchy) {
        const double measures[] = {stats.mSurfaceArea, stats.mAverageEdgeLength, stats.mMaximumEdgeLength};
        write(measures, sizeof(measures));
        write(stats.mWeightedCenter.data(), 3 * sizeof(float));
        write(stats.mAABB.min.data(), 3 * sizeof(float));
        write(stats.mAABB.max.data(), 3 * sizeof(float));
        write(&scale, sizeof(scale));
        MultiResolutionHierarchy levels;
        levels.setV(std::move(positions));
        levels.setN(std::move(normals));
        levels.setA(std::move(areas));
        levels.setAdj(std::move(neighbors));
        levels.setScale(scale);
        levels.build(true);
        levels.resetSolution();
        number(levels.levels());
        for (int depth = 0; depth < levels.levels(); ++depth) {
            auto size = levels.size(depth);
            number(size);
            write(levels.V(depth).data(), 3 * size * sizeof(float));
            write(levels.N(depth).data(), 3 * size * sizeof(float));
            write(levels.A(depth).data(), size * sizeof(float));
            auto adj = levels.adj(depth);
            for (uint32_t i = 0; i <= size; ++i)
                number(adj[i] - adj[0]);
            for (auto link = adj[0]; link != adj[size]; ++link) {
                number(link->id);
                write(&link->weight, sizeof(float));
            }
            number(levels.phases(depth).size());
            for (const auto &phase : levels.phases(depth)) {
                number(phase.size());
                write(phase.data(), phase.size() * sizeof(uint32_t));
            }
            write(levels.Q(depth).data(), 3 * size * sizeof(float));
            write(levels.O(depth).data(), 3 * size * sizeof(float));
            if (depth + 1 < levels.levels()) {
                auto children = levels.toUpper(depth);
                number(children.size());
                write(children.data(), children.size() * sizeof(uint32_t));
                write(levels.toLower(depth).data(), size * sizeof(uint32_t));
            }
        }
        levels.free();
    } else {
        delete[] neighbors[0];
        delete[] neighbors;
    }
    return stream ? 0 : 1;
}
