// C# adaptation of Instant Meshes extract.cpp and reorder.cpp. See LICENSE-instant-meshes.
using System.Diagnostics;
using System.Numerics;
using static InstantMeshes.Core.FieldGeometry;
using static InstantMeshes.Core.Preprocessing;

namespace InstantMeshes.Core;

internal sealed record ExtractedMesh(Vector3[] Positions, Vector3[] Normals, int[] Indices,
    Vector3[] FaceNormals, int Arity, TimeSpan Reprojection = default);

internal static class FaceExtraction
{
    private sealed class HalfEdge(int vertex)
    {
        public int Vertex { get; } = vertex;
        public bool Used { get; set; }
        public bool Border { get; set; }
    }

    public static ExtractedMesh Extract(ExtractedGraph graph, float scale, RemeshOptions options,
        PreparedMesh? input, CancellationToken token)
    {
        int arity = options.PositionSymmetry;
        var positions = graph.Positions.ToList();
        var normals = graph.Normals.ToList();
        var creases = graph.Creases;
        var adjacency = Array.ConvertAll(graph.Adjacency, list => list.Select(i => new HalfEdge(i)).ToList());
        var faces = new List<int[]>();
        var irregular = new List<int[]>();
        var cycle = new List<(int Vertex, HalfEdge Edge)>();
        int linkCount = adjacency.Sum(row => row.Count);

        // Claim small cycles first, then trace the remaining boundary loops.
        for (int size = 3; size <= 8; size++)
        {
            Trace(size);
        }

        for (int i = 0; i < adjacency.Length; i++)
        {
            CheckCancellation(i, token);
            foreach (HalfEdge edge in adjacency[i])
            {
                if (edge.Used)
                {
                    continue;
                }

                HalfEdge? reverse = adjacency[edge.Vertex].Find(other => other.Vertex == i);
                if (reverse?.Used == true)
                {
                    edge.Border = reverse.Border = true;
                }
            }
        }
        foreach (List<HalfEdge> row in adjacency)
        {
            row.RemoveAll(edge => !edge.Border);
        }

        Trace(0);
        if (faces.Count == 0)
        {
            throw new InvalidOperationException("No faces were extracted. Try a finer density or verify the input orientation.");
        }

        if (options.Faces == FaceMode.PureQuads)
        {
            Subdivide();
        }

        Vector3[] vertices = positions.ToArray(), vertexNormals = normals.ToArray();
        var timer = Stopwatch.StartNew();
        (vertices, vertexNormals) = SurfaceProjection.Smooth(input, vertices, vertexNormals, faces,
            creases, scale, options, token);
        TimeSpan reprojection = timer.Elapsed;
        var faceNormals = new Vector3[faces.Count];
        for (int i = 0; i < faces.Count; i++)
        {
            CheckCancellation(i, token);
            faceNormals[i] = SurfaceProjection.FitNormal(vertices, vertexNormals, faces[i], options.StableTopology);
        }
        if (options.Faces == FaceMode.QuadDominant)
        {
            foreach (int[] group in irregular)
            {
                int[] ring = Array.ConvertAll(group, i => faces[i][0]);
                Vector3 normal = SurfaceProjection.FitNormal(vertices, vertexNormals, ring, options.StableTopology);
                foreach (int i in group)
                {
                    faceNormals[i] = normal;
                }
            }
        }
        return Reorder(vertices, vertexNormals, faces, faceNormals, arity, token) with { Reprojection = reprojection };

        void Trace(int target)
        {
            for (int start = 0; start < adjacency.Length; start++)
            {
                CheckCancellation(start, token);
                for (int index = 0; index < adjacency[start].Count; index++)
                {
                    if (adjacency[start][index].Used)
                    {
                        continue;
                    }

                    cycle.Clear();
                    int vertex = start, edgeIndex = index;
                    while (cycle.Count < (target == 0 ? linkCount : target))
                    {
                        HalfEdge edge = adjacency[vertex][edgeIndex];
                        if (edge.Used)
                        {
                            break;
                        }

                        cycle.Add((vertex, edge));
                        int next = edge.Vertex;
                        List<HalfEdge> row = adjacency[next];
                        int reverse = row.FindIndex(other => other.Vertex == vertex);
                        if (reverse < 0 || row.Count == 1)
                        {
                            break;
                        }

                        vertex = next;
                        // The next angular neighbor continues the face on the same side.
                        edgeIndex = (reverse + 1) % row.Count;
                        if (vertex != start)
                        {
                            continue;
                        }

                        if (target == 0 || cycle.Count == target)
                        {
                            foreach ((_, HalfEdge used) in cycle)
                            {
                                used.Used = true;
                            }

                            if (target != 0 || cycle.Count < 7)
                            {
                                Fill(cycle.Select(entry => entry.Vertex).ToList());
                            }
                        }
                        break;
                    }
                }
            }
        }

        void Fill(List<int> vertices)
        {
            while (vertices.Count > 2)
            {
                int count = vertices.Count;
                if (count == arity)
                {
                    faces.Add(vertices.ToArray());
                    return;
                }
                if (arity == 4 && count <= 5)
                {
                    Vector3 center = default, normal = default;
                    foreach (int v in vertices)
                    {
                        center += positions[v];
                        normal += normals[v];
                    }
                    int midpoint = positions.Count;
                    positions.Add(Canonical(center / count, options.StableTopology));
                    normals.Add(Canonical(Unit(normal), options.StableTopology));
                    var group = new int[count];
                    for (int i = 0; i < count; i++)
                    {
                        group[i] = faces.Count;
                        faces.Add([vertices[i], vertices[(i + 1) % count], midpoint, midpoint]);
                    }
                    irregular.Add(group);
                    return;
                }

                float best = float.PositiveInfinity;
                int first = 0;
                for (int i = 0; i < count; i++)
                {
                    float score = 0;
                    for (int k = 0; k < arity; k++)
                    {
                        Vector3 a = positions[vertices[(i + k) % count]],
                            b = positions[vertices[(i + k + 1) % count]],
                            c = positions[vertices[(i + k + 2) % count]];
                        float angle = (float)(MathF.Acos(Math.Clamp(Dot(Unit(a - b), Unit(c - b)), -1f, 1f))
                            * 180f / Math.PI);
                        score += MathF.Abs(angle - (arity == 4 ? 90 : 60));
                    }
                    if (score < best)
                    {
                        (first, best) = (i, score);
                    }
                }

                var face = new int[arity];
                for (int i = 0; i < arity; i++)
                {
                    int index = (first + i) % count;
                    face[i] = vertices[index];
                    if (i > 0 && i < arity - 1)
                    {
                        vertices[index] = -1;
                    }
                }
                faces.Add(face);
                vertices.RemoveAll(v => v < 0);
            }
        }

        void Subdivide()
        {
            int count = faces.Count;
            // Share edge points between adjacent faces while turning each face into quads.
            int[] opposite = OppositeEdges(faces, positions.Count, 4, token);
            var edgePoints = new int[opposite.Length];
            Array.Fill(edgePoints, -1);
            var corners = new int[4];
            for (int i = 0; i < count; i++)
            {
                CheckCancellation(i, token);
                int[] face = faces[i];
                if (face[2] == face[3])
                {
                    continue;
                }

                int center = positions.Count;
                positions.Add((positions[face[0]] + positions[face[1]] +
                    positions[face[2]] + positions[face[3]]) / 4);
                normals.Add(Unit(normals[face[0]] + normals[face[1]] + normals[face[2]] + normals[face[3]]));
                if (Array.TrueForAll(face, creases.Contains))
                {
                    creases.Add(center);
                }

                for (int edge = 0; edge < 4; edge++)
                {
                    corners[edge] = EdgePoint(i * 4 + edge, face[edge], face[(edge + 1) % 4]);
                }

                for (int edge = 0; edge < 4; edge++)
                {
                    int next = (edge + 1) % 4;
                    int[] quad = [corners[edge], face[next], corners[next], center];
                    if (edge == 0)
                    {
                        faces[i] = quad;
                    }
                    else
                    {
                        faces.Add(quad);
                    }
                }
            }

            foreach (int[] group in irregular)
            {
                token.ThrowIfCancellationRequested();
                int center = faces[group[0]][3];
                int[] vertices = new int[group.Length], midpoints = new int[group.Length];
                for (int i = 0; i < group.Length; i++)
                {
                    int face = group[i], a = faces[face][0], b = faces[face][1];
                    vertices[i] = a;
                    midpoints[i] = EdgePoint(face * 4, a, b);
                }
                for (int i = 0; i < group.Length; i++)
                {
                    int next = (i + 1) % group.Length;
                    faces[group[i]] = [midpoints[i], vertices[next], midpoints[next], center];
                }
            }

            int EdgePoint(int edge, int a, int b)
            {
                int reverse = opposite[edge];
                if (reverse >= 0 && edgePoints[reverse] >= 0)
                {
                    return edgePoints[reverse];
                }

                int id = positions.Count;
                edgePoints[edge] = id;
                positions.Add((positions[a] + positions[b]) * 0.5f);
                normals.Add(Unit(normals[a] + normals[b]));
                if (creases.Contains(a) && creases.Contains(b))
                {
                    creases.Add(id);
                }

                return id;
            }
        }
    }

    private static int[] OppositeEdges(List<int[]> faces, int count, int arity, CancellationToken token)
    {
        int size = checked(faces.Count * arity);
        var first = new int[count];
        var next = new int[size];
        var opposite = new int[size];
        Array.Fill(first, -1);
        Array.Fill(next, -1);
        Array.Fill(opposite, -1);
        for (int edge = size - 1; edge >= 0; edge--)
        {
            CheckCancellation(edge, token);
            int from = Start(edge);
            if (from == End(edge))
            {
                continue;
            }

            next[edge] = first[from];
            first[from] = edge;
        }
        for (int edge = 0; edge < size; edge++)
        {
            CheckCancellation(edge, token);
            int from = Start(edge), to = End(edge), reverse = -1;
            if (from == to)
            {
                continue;
            }

            for (int other = first[to]; other >= 0; other = next[other])
            {
                if (End(other) != from)
                {
                    continue;
                }

                if (reverse >= 0)
                {
                    reverse = -1;
                    break;
                }
                reverse = other;
            }
            if (reverse > edge)
            {
                (opposite[edge], opposite[reverse]) = (reverse, edge);
            }
        }
        return opposite;

        int Start(int edge) => faces[edge / arity][edge % arity];
        int End(int edge) => faces[edge / arity][(edge % arity + 1) % arity];
    }

    private static ExtractedMesh Reorder(Vector3[] vertices, Vector3[] normals, List<int[]> faces,
        Vector3[] faceNormals, int arity, CancellationToken token)
    {
        int[] opposite = OppositeEdges(faces, vertices.Length, arity, token);
        var newVertices = new List<Vector3>();
        var newNormals = new List<Vector3>();
        var newFaces = new int[opposite.Length];
        var newFaceNormals = new Vector3[faces.Count];
        var vertexMap = new int[vertices.Length];
        Array.Fill(vertexMap, -1);
        var visited = new bool[faces.Count];
        var queue = new Queue<int>();
        int target = 0;
        // Pinned macOS native order: reverse insertion order for component seeds.
        for (int seed = faces.Count - 1; seed >= 0; seed--)
        {
            if (visited[seed])
            {
                continue;
            }

            Enqueue(seed);
            while (queue.TryDequeue(out int source))
            {
                CheckCancellation(target, token);
                newFaceNormals[target] = faceNormals[source];
                for (int i = 0; i < arity; i++)
                {
                    int vertex = faces[source][i];
                    if (vertexMap[vertex] < 0)
                    {
                        vertexMap[vertex] = newVertices.Count;
                        newVertices.Add(vertices[vertex]);
                        newNormals.Add(normals[vertex]);
                    }
                    newFaces[target * arity + i] = vertexMap[vertex];
                    int reverse = opposite[source * arity + i];
                    if (reverse >= 0 && !visited[reverse / arity])
                    {
                        Enqueue(reverse / arity);
                    }
                }
                target++;
            }
        }
        return new(newVertices.ToArray(), newNormals.ToArray(), newFaces, newFaceNormals, arity);

        void Enqueue(int face)
        {
            visited[face] = true;
            queue.Enqueue(face);
        }
    }
}
