using System.Numerics;
using InstantMeshes.Core;
using Rhino.Geometry;

namespace InstantMeshes.RhinoGeometry;

public sealed class GeometrySnapshot
{
    // Owned arrays detach the background solve from mutable Rhino geometry.
    private readonly Vector3[] positions;
    private readonly int[]? triangles;
    private readonly Vector3[]? normals;

    internal GeometrySnapshot(Vector3[] positions, int[] triangles, Point3d origin) =>
        (this.positions, this.triangles, Origin) = (positions, triangles, origin);

    internal GeometrySnapshot(Vector3[] positions, Vector3[] normals, Point3d origin) =>
        (this.positions, this.normals, Origin) = (positions, normals, origin);

    public Point3d Origin { get; }

    public RemeshResult Solve(RemeshOptions options, CancellationToken token) =>
        triangles is not null ? Remesher.RemeshTriangles(positions, triangles, options, token) :
            Remesher.RemeshPoints(positions, normals, options, token);

    public bool HasSameGeometry(GeometrySnapshot other) =>
        Origin == other.Origin &&
        positions.AsSpan().SequenceEqual(other.positions) &&
        triangles.AsSpan().SequenceEqual(other.triangles) &&
        normals.AsSpan().SequenceEqual(other.normals);
}

public static class MeshAdapter
{
    public static GeometrySnapshot Snapshot(Mesh source)
    {
        ArgumentNullException.ThrowIfNull(source);
        if (source.Vertices.Count == 0 || source.Faces.Count == 0)
        {
            throw new ArgumentException("Select a mesh with vertices and faces.");
        }

        // Shift before converting to float so large world coordinates retain local detail.
        Point3d origin = source.Vertices.Point3dAt(0);
        var positions = new Vector3[source.Vertices.Count];
        for (int i = 0; i < positions.Length; i++)
        {
            positions[i] = Local(source.Vertices.Point3dAt(i), origin);
        }

        var triangles = new List<int>(checked(source.Faces.Count * 3));
        foreach (MeshFace face in source.Faces)
        {
            triangles.Add(face.A);
            triangles.Add(face.B);
            triangles.Add(face.C);
            // Keep the A–C diagonal and the original face order.
            if (face.IsQuad)
            {
                triangles.Add(face.A);
                triangles.Add(face.C);
                triangles.Add(face.D);
            }
        }
        return new(positions, triangles.ToArray(), origin);
    }

    public static GeometrySnapshot Snapshot(PointCloud source)
    {
        ArgumentNullException.ThrowIfNull(source);
        if (source.Count < 3 || !source.ContainsNormals)
        {
            throw new ArgumentException("Provide a point cloud with at least three points and oriented normals.");
        }

        Point3d[] points = source.GetPoints();
        Vector3d[] sourceNormals = source.GetNormals();
        if (sourceNormals.Length != points.Length)
        {
            throw new ArgumentException("The point cloud must supply one oriented normal per point.");
        }

        Point3d origin = points[0];
        var positions = new Vector3[points.Length];
        var normals = new Vector3[points.Length];
        for (int i = 0; i < points.Length; i++)
        {
            positions[i] = Local(points[i], origin);
            Vector3d normal = sourceNormals[i];
            if (!normal.IsValid || !normal.Unitize())
            {
                throw new ArgumentException($"Point {i} has an unusable normal.");
            }

            normals[i] = new((float)normal.X, (float)normal.Y, (float)normal.Z);
        }
        return new(positions, normals, origin);
    }

    private static Vector3 Local(Point3d point, Point3d origin)
    {
        if (!point.IsValid)
        {
            throw new ArgumentException("Input coordinates must be valid finite Rhino points.");
        }

        Vector3d local = point - origin;
        return new((float)local.X, (float)local.Y, (float)local.Z);
    }

    public static Mesh ToMesh(RemeshResult result, Point3d origin)
    {
        var output = new Mesh();
        try
        {
            output.Vertices.UseDoublePrecisionVertices = true;
            foreach (Vector3 position in result.Positions)
            {
                output.Vertices.Add(origin.X + position.X, origin.Y + position.Y, origin.Z + position.Z);
            }

            int[] faces = result.Indices;
            int arity = result.FaceArity;
            for (int i = 0; i < faces.Length; i += arity)
            {
                if (arity == 3 || faces[i + 2] == faces[i + 3])
                {
                    output.Faces.AddFace(faces[i], faces[i + 1], faces[i + 2]);
                }
                else
                {
                    output.Faces.AddFace(faces[i], faces[i + 1], faces[i + 2], faces[i + 3]);
                }
            }
            output.FaceNormals.ComputeFaceNormals();
            output.Normals.ComputeNormals();
            if (!output.IsValid)
            {
                throw new InvalidOperationException("Rhino reported invalid output mesh geometry.");
            }
            // Ownership passes to the caller only after Rhino validates the mesh.
            return output;
        }
        catch
        {
            output.Dispose();
            throw;
        }
    }
}
