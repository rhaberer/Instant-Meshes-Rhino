// Point-cloud adjacency adapted from Instant Meshes adjacency.cpp and bvh.cpp. See LICENSE-instant-meshes.
using System.Numerics;
using static InstantMeshes.Core.Preprocessing;

namespace InstantMeshes.Core;

internal static class PointCloudPreprocessing
{
    public static (HierarchyLevel Level, double Area, int Filtered) Prepare(
        Vector3[] positions, ReadOnlySpan<Vector3> inputNormals, RemeshOptions options, CancellationToken token,
        Action<int, int[]>? observeCandidates = null)
    {
        if (positions.Length < 3 || inputNormals.Length != positions.Length)
        {
            throw new ArgumentException("Provide at least three points with one oriented normal per point.");
        }

        var normals = inputNormals.ToArray();
        for (int i = 0; i < normals.Length; i++)
        {
            CheckCancellation(i, token);
            float length = Norm(normals[i]);
            if (!float.IsFinite(length) || length <= ReciprocalOverflow)
            {
                throw new ArgumentException($"Point {i} has an unusable normal.", nameof(inputNormals));
            }
            // Retain the supplied float components of already-unit normals.
            if (MathF.Abs(length - 1f) > 4 * 1.1920929e-7f)
            {
                normals[i] /= length;
            }
        }

        var index = new PointIndex(positions, token);
        var (diskRadius, preciseRadius, useDiskBounds) = index.MeasureSpacing();
        if (!(diskRadius > 0f) || !float.IsFinite(diskRadius))
        {
            throw new ArgumentException("The point cloud has no usable spacing.");
        }

        var neighbors = new int[positions.Length][];
        var sets = new DisjointSets(positions.Length);
        int neighborLimit = Math.Min(options.PointNeighbors, positions.Length - 1);
        var candidates = new PointCandidate[neighborLimit + 1];
        if (useDiskBounds)
        {
            index.Refit(normals, diskRadius);
        }

        if (!QueryNeighbors(diskRadius) && useDiskBounds)
        {
            // The native bounds and truncated radius can hide every point.
            // Retry with point-center bounds and untruncated spacing.
            diskRadius = preciseRadius;
            index.Refit(null, diskRadius);
            QueryNeighbors(diskRadius);
        }
        for (int i = 0; i < positions.Length; i++)
        {
            CheckCancellation(i, token);
            observeCandidates?.Invoke(i, neighbors[i]);
            foreach (int j in neighbors[i])
            {
                sets.Unite(i, j);
            }
        }
        // Skip outgoing links from components smaller than one percent of the input.
        var counts = new int[positions.Length];
        for (int i = 0; i < counts.Length; i++)
        {
            counts[sets.Find(i)]++;
        }

        var rows = new List<Link>[positions.Length];
        int filtered = 0;
        for (int i = 0; i < rows.Length; i++)
        {
            rows[i] = [];
        }

        for (int i = 0; i < rows.Length; i++)
        {
            CheckCancellation(i, token);
            if (counts[sets.Find(i)] < rows.Length * 0.01f)
            {
                filtered++;
                continue;
            }
            foreach (int j in neighbors[i])
            {
                rows[i].Add(new(j));
                // Preserve directed-query order and add only missing reverse links.
                if (Array.IndexOf(neighbors[j], i) < 0)
                {
                    rows[j].Add(new(i));
                }
            }
        }
        var offsets = new int[positions.Length + 1];
        var links = new List<Link>();
        for (int i = 0; i < rows.Length; i++)
        {
            links.AddRange(rows[i]);
            offsets[i + 1] = links.Count;
            normals[i] = Canonical(normals[i], options.StableTopology);
        }
        if (links.Count == 0)
        {
            throw new ArgumentException("No compatible point neighbors were found. Check the spacing and orientation of the cloud.");
        }

        var areas = new float[positions.Length];
        Array.Fill(areas, 1f);
        return (new(positions, normals, areas, offsets, links.ToArray()),
            Math.PI * diskRadius * diskRadius * 0.5 * positions.Length, filtered);

        bool QueryNeighbors(float radius)
        {
            bool found = false;
            for (int i = 0; i < positions.Length; i++)
            {
                CheckCancellation(i, token);
                neighbors[i] = index.OrientedNeighbors(positions[i], normals, normals[i], radius * 3, candidates);
                found |= neighbors[i].Length != 0;
            }
            return found;
        }
    }

    private readonly record struct PointCandidate(float Distance, int Index);

    private readonly record struct Bounds(Vector3 Minimum, Vector3 Maximum)
    {
        public static Bounds Empty => new(new(float.PositiveInfinity), new(float.NegativeInfinity));

        public Bounds Include(Vector3 point) =>
            new(Vector3.Min(Minimum, point), Vector3.Max(Maximum, point));

        public Bounds Merge(Bounds other) =>
            new(Vector3.Min(Minimum, other.Minimum), Vector3.Max(Maximum, other.Maximum));

        public float SurfaceArea()
        {
            Vector3 d = Maximum - Minimum;
            return 2f * (d.X * d.Y + d.X * d.Z + d.Y * d.Z);
        }

        public float SquaredDistance(Vector3 point)
        {
            Vector3 delta = Vector3.Max(Vector3.Max(Minimum - point, point - Maximum), Vector3.Zero);
            return (delta.X * delta.X + delta.Y * delta.Y) + delta.Z * delta.Z;
        }
    }

    private sealed class PointIndex
    {
        private const int SerialThreshold = 32;
        private const int BinCount = 8;
        private const int ReductionGrain = 1024;
        private const int MaxDepth = 64;
        // The left child follows its parent in nodes; Right stores the other child.
        private readonly record struct Node(Bounds Bounds, int First, int End, int Right);
        private readonly Vector3[] positions;
        private readonly int[] points;
        private readonly int[] partition;
        private readonly List<Node> nodes = [];
        private readonly CancellationToken token;

        public PointIndex(Vector3[] positions, CancellationToken token)
        {
            this.positions = positions;
            this.token = token;
            points = Enumerable.Range(0, positions.Length).ToArray();
            partition = new int[points.Length];
            Bounds bounds = Bounds.Empty;
            for (int i = 0; i < positions.Length; i++)
            {
                CheckCancellation(i, token);
                bounds = bounds.Include(positions[i]);
            }
            Build(0, points.Length, bounds, false, 0);
        }

        private int Build(int first, int end, Bounds bounds, bool serial, int depth)
        {
            token.ThrowIfCancellationRequested();
            int node = nodes.Count;
            nodes.Add(new(bounds, first, end, -1));
            int size = end - first;
            float area = bounds.SurfaceArea();
            if (size <= 1 || depth == MaxDepth)
            {
                return node;
            }

            serial |= size < SerialThreshold || !(area > 0) || !float.IsFinite(area);
            Bounds left = Bounds.Empty, right = Bounds.Empty;
            int split = serial ? first : BinSplit(first, end, bounds, out left, out right);
            if (split == first)
            {
                serial = true;
                split = SerialSplit(first, end, area, out left, out right);
                if (split == first)
                {
                    return node;
                }
            }

            Build(first, split, left, serial, depth + 1);
            int rightIndex = Build(split, end, right, serial, depth + 1);
            nodes[node] = nodes[node] with { Right = rightIndex };
            return node;
        }

        private int BinSplit(int first, int end, Bounds bounds, out Bounds left, out Bounds right)
        {
            Span<Bounds> bins = stackalloc Bounds[BinCount];
            Span<Bounds> prefix = stackalloc Bounds[BinCount];
            Span<int> counts = stackalloc int[BinCount];
            bins.Fill(Bounds.Empty);
            counts.Clear();
            int axis = SpatialAxis.Longest(bounds.Maximum - bounds.Minimum);
            float min = bounds.Minimum[axis];
            float multiplier = BinCount / (bounds.Maximum[axis] - min);
            for (int i = first; i < end; i++)
            {
                CheckCancellation(i, token);
                int bin = Classify(points[i]);
                bins[bin] = bins[bin].Include(positions[points[i]]);
                counts[bin]++;
            }
            prefix[0] = bins[0];
            for (int i = 1; i < BinCount; i++)
            {
                counts[i] += counts[i - 1];
                prefix[i] = prefix[i - 1].Merge(bins[i]);
            }

            Bounds suffix = bins[^1];
            left = right = Bounds.Empty;
            float bestCost = end - first, inverseArea = 1f / bounds.SurfaceArea();
            int bestBin = -1;
            for (int i = BinCount - 2; i >= 0; i--)
            {
                int leftCount = counts[i], rightCount = end - first - leftCount;
                float cost = 2f + inverseArea * (leftCount * prefix[i].SurfaceArea() +
                                                rightCount * suffix.SurfaceArea());
                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestBin = i;
                    left = prefix[i];
                    right = suffix;
                }
                suffix = suffix.Merge(bins[i]);
            }
            if (bestBin < 0)
            {
                return first;
            }

            int split = first + counts[bestBin], nextLeft = first, nextRight = split;
            for (int i = first; i < end; i++)
            {
                CheckCancellation(i, token);
                int point = points[i];
                partition[Classify(point) <= bestBin ? nextLeft++ : nextRight++] = point;
            }
            Array.Copy(partition, first, points, first, end - first);
            return split;

            int Classify(int point) => Math.Clamp((int)((positions[point][axis] - min) * multiplier), 0, BinCount - 1);
        }

        private int SerialSplit(int first, int end, float area, out Bounds left, out Bounds right)
        {
            int size = end - first;
            Span<float> leftAreas = size < SerialThreshold ? stackalloc float[size] : new float[size];
            float bestCost = size, inverseArea = 1f / area;
            int bestAxis = -1, split = first;
            for (int axis = 0; axis < 3; axis++)
            {
                SortAxis(first, end, axis);
                Bounds bounds = Bounds.Empty;
                for (int i = first; i < end; i++)
                {
                    CheckCancellation(i, token);
                    bounds = bounds.Include(positions[points[i]]);
                    leftAreas[i - first] = bounds.SurfaceArea();
                }
                bounds = Bounds.Empty;
                for (int i = end - 1; i > first; i--)
                {
                    CheckCancellation(i, token);
                    bounds = bounds.Include(positions[points[i]]);
                    float cost = 2f + inverseArea * ((i - first) * leftAreas[i - first - 1] +
                                                     (end - i) * bounds.SurfaceArea());
                    if (cost < bestCost)
                    {
                        bestCost = cost;
                        split = i;
                        bestAxis = axis;
                    }
                }
            }
            left = right = Bounds.Empty;
            if (split != first)
            {
                SortAxis(first, end, bestAxis);
                left = GetBounds(first, split);
                right = GetBounds(split, end);
            }
            return split;
        }

        private void SortAxis(int first, int end, int axis)
        {
            if (end - first >= SerialThreshold)
            {
                Array.Sort(points, first, end - first, Comparer<int>.Create((a, b) =>
                {
                    int order = positions[a][axis].CompareTo(positions[b][axis]);
                    return order == 0 ? a.CompareTo(b) : order;
                }));
                return;
            }

            for (int i = first + 1; i < end; i++)
            {
                int point = points[i], j = i;
                float coordinate = positions[point][axis];
                while (j > first && coordinate < positions[points[j - 1]][axis])
                {
                    points[j] = points[j - 1];
                    j--;
                }
                points[j] = point;
            }
        }

        private Bounds GetBounds(int first, int end)
        {
            Bounds bounds = Bounds.Empty;
            for (int i = first; i < end; i++)
            {
                CheckCancellation(i, token);
                bounds = bounds.Include(positions[points[i]]);
            }
            return bounds;
        }

        public (float Radius, float PreciseRadius, bool UseDiskBounds) MeasureSpacing()
        {
            (double precise, long truncated) = SumDistances(0, positions.Length);
            float preciseRadius = 3f * (float)(precise / positions.Length);
            // An integer identity truncates every native reduction chunk.
            // Use the untruncated radius if the native sum is zero or out of range.
            bool useDiskBounds = truncated is > 0 and <= int.MaxValue;
            float radius = useDiskBounds ? 3f * (float)((double)truncated / positions.Length) : preciseRadius;
            return (radius, preciseRadius, useDiskBounds);
        }

        private (double Precise, long Truncated) SumDistances(int first, int end)
        {
            if (end - first > ReductionGrain)
            {
                int middle = first + (end - first) / 2;
                var left = SumDistances(first, middle);
                var right = SumDistances(middle, end);
                return (left.Precise + right.Precise, left.Truncated + right.Truncated);
            }
            double sum = 0;
            for (int i = first; i < end; i++)
            {
                CheckCancellation(i, token);
                float limit = float.PositiveInfinity;
                int count = 0;
                bool isHeap = false;
                Search(0, positions[i], ref limit, null, default, default, ref count, ref isHeap);
                if (float.IsFinite(limit))
                {
                    sum += MathF.Sqrt(limit);
                }
            }
            return (sum, (long)Math.Min(sum, (double)int.MaxValue + 1));
        }

        public void Refit(Vector3[]? normals, float radius) => Refit(0, normals, radius);

        private Bounds Refit(int index, Vector3[]? normals, float radius)
        {
            token.ThrowIfCancellationRequested();
            Node node = nodes[index];
            Bounds bounds;
            if (node.Right >= 0)
            {
                bounds = Refit(index + 1, normals, radius).Merge(Refit(node.Right, normals, radius));
            }
            else if (normals is null)
            {
                bounds = GetBounds(node.First, node.End);
            }
            else
            {
                // The native refit overwrites the leaf bounds at every point,
                // so traversal sees the disk of the leaf's last point.
                int last = points[node.End - 1];
                Vector3 n = normals[last];
                Vector3 t;
                if (MathF.Abs(n.X) > MathF.Abs(n.Y))
                {
                    float invLength = 1f / MathF.Sqrt(n.X * n.X + n.Z * n.Z);
                    t = new(n.Z * invLength, 0, -n.X * invLength);
                }
                else
                {
                    float invLength = 1f / MathF.Sqrt(n.Y * n.Y + n.Z * n.Z);
                    t = new(0, n.Z * invLength, -n.Y * invLength);
                }
                Vector3 s = Vector3.Cross(t, n);
                bounds = Bounds.Empty;
                for (int corner = 0; corner < 4; corner++)
                {
                    bounds = bounds.Include(positions[last] + radius * (((corner & 1) * 2 - 1) * s +
                                                                         ((corner & 2) - 1) * t));
                }
            }
            nodes[index] = node with { Bounds = bounds };
            return bounds;
        }

        public int[] OrientedNeighbors(Vector3 point, Vector3[] normals, Vector3 normal, float radius,
            Span<PointCandidate> candidates)
        {
            int count = 0;
            bool isHeap = false;
            float limit = radius * radius;
            Search(0, point, ref limit, normals, normal, candidates, ref count, ref isHeap);
            var result = new int[count];
            for (int i = 0; i < result.Length; i++)
            {
                result[i] = candidates[i].Index;
            }

            return result;
        }

        private void Search(int index, Vector3 point, ref float limit, Vector3[]? normals, Vector3 normal,
            Span<PointCandidate> candidates, ref int count, ref bool isHeap)
        {
            const float cos30 = 0.866025403784439f;
            Node node = nodes[index];
            if (node.Bounds.SquaredDistance(point) > limit)
            {
                return;
            }

            if (node.Right >= 0)
            {
                int near = index + 1, far = node.Right;
                float nearDistance = nodes[near].Bounds.SquaredDistance(point);
                float farDistance = nodes[far].Bounds.SquaredDistance(point);
                if (!(nearDistance < farDistance))
                {
                    (near, far) = (far, near);
                    farDistance = nearDistance;
                }
                bool visitFar = farDistance < limit;
                Search(near, point, ref limit, normals, normal, candidates, ref count, ref isHeap);
                if (visitFar)
                {
                    Search(far, point, ref limit, normals, normal, candidates, ref count, ref isHeap);
                }

                return;
            }

            for (int i = node.First; i < node.End; i++)
            {
                CheckCancellation(i, token);
                int candidate = points[i];
                Vector3 offset = positions[candidate] - point;
                float distance = Dot(offset, offset);
                if (distance == 0 || !(distance < limit))
                {
                    continue;
                }

                if (normals is null)
                {
                    limit = distance;
                }
                else if (Dot(normals[candidate], normal) > cos30)
                {
                    AddCandidate(new(distance, candidate), candidates, ref count, ref isHeap, ref limit);
                }
            }
        }
    }

    // The native query returns insertion order up to k hits, then the buffer
    // order of a binary max heap. Equal distances compare by distance alone.
    private static void AddCandidate(PointCandidate item, Span<PointCandidate> heap,
        ref int count, ref bool isHeap, ref float limit)
    {
        int k = heap.Length - 1;
        if (count < k)
        {
            heap[count++] = item;
            return;
        }

        if (!isHeap)
        {
            for (int parent = k / 2 - 1; parent >= 0; parent--)
            {
                PointCandidate top = heap[parent];
                int hole = parent;
                while (hole < k / 2)
                {
                    int child = LargerChild(heap, hole, k);
                    if (heap[child].Distance < top.Distance)
                    {
                        break;
                    }

                    heap[hole] = heap[child];
                    hole = child;
                }
                heap[hole] = top;
            }
            isHeap = true;
        }

        SiftUp(heap, k, item);
        // Move the greater children up, then insert the old tail from the
        // resulting hole. On distance ties, the left child goes first.
        int leaf = 0;
        while (leaf < (k + 1) / 2)
        {
            int child = LargerChild(heap, leaf, k + 1);
            heap[leaf] = heap[child];
            leaf = child;
        }
        if (leaf < k)
        {
            SiftUp(heap, leaf, heap[k]);
        }

        limit = heap[0].Distance;
    }

    private static int LargerChild(Span<PointCandidate> heap, int parent, int length)
    {
        int left = 2 * parent + 1, right = left + 1;
        return right < length && heap[left].Distance < heap[right].Distance ? right : left;
    }

    private static void SiftUp(Span<PointCandidate> heap, int hole, PointCandidate item)
    {
        while (hole > 0)
        {
            int parent = (hole - 1) / 2;
            if (!(heap[parent].Distance < item.Distance))
            {
                break;
            }

            heap[hole] = heap[parent];
            hole = parent;
        }
        heap[hole] = item;
    }
}
