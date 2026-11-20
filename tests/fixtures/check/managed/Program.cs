using System.Diagnostics;
using System.Globalization;
using System.Numerics;
using InstantMeshes.Core;

CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;
var points = new List<Vector3>();
var normals = new List<Vector3>();
var triangles = new List<int>();
bool cloud = Path.GetExtension(args[0]) == ".ply";
bool inHeader = cloud;
foreach (string line in File.ReadLines(args[0]))
{
    if (inHeader)
    {
        inHeader = line != "end_header";
        continue;
    }
    var f = line.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
    if (cloud)
    {
        points.Add(Vector(f, 0));
        normals.Add(Vector(f, 3));
    }
    else if (f.Length > 0 && f[0] == "v")
    {
        points.Add(Vector(f, 1));
    }
    else if (f.Length > 0 && f[0] == "f")
    {
        for (int i = 1; i <= 3; i++)
        {
            triangles.Add(int.Parse(f[i]) - 1);
        }
    }
}
var options = new RemeshOptions
{
    Density = DensityMode.Vertices,
    Target = int.Parse(args[1]),
    StableTopology = args[2] == "1",
    OrientationSymmetry = int.Parse(args[3]),
    Faces = Enum.Parse<FaceMode>(args[4]),
    SmoothingPasses = int.Parse(args[5]),
    AlignToBoundaries = args[6] == "1",
    CreaseAngle = args[7] == "-" ? null : float.Parse(args[7]),
    PointNeighbors = int.Parse(args[8])
};
var positions = points.ToArray();
var triangleIndices = triangles.ToArray();
var inputNormals = normals.ToArray();
var timer = Stopwatch.StartNew();
var result = cloud
    ? Remesher.RemeshPoints(positions, inputNormals, options)
    : Remesher.RemeshTriangles(positions, triangleIndices, options);
timer.Stop();
using var writer = new BinaryWriter(File.Create(args[9]));
writer.Write(result.VertexCount);
writer.Write(result.FaceCount);
writer.Write(result.FaceArity);
foreach (Vector3 p in result.Positions)
{
    writer.Write(p.X);
    writer.Write(p.Y);
    writer.Write(p.Z);
}
foreach (int i in result.Indices)
{
    writer.Write(i);
}

Console.WriteLine($"{result.VertexCount} vertices, {result.FaceCount} faces, arity {result.FaceArity}, cold solve {timer.Elapsed.TotalMilliseconds:F2} ms");

static Vector3 Vector(string[] f, int i) => new(float.Parse(f[i]), float.Parse(f[i + 1]), float.Parse(f[i + 2]));
