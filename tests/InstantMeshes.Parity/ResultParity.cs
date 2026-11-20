using System.Diagnostics;
using System.Globalization;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Text.Json;
using InstantMeshes.Core;

internal static class ResultParity
{
    public static int Run(string[] args)
    {
        string input = args[1], reference = args[2];
        RemeshOptions options = new()
        {
            Density = DensityMode.Vertices,
            Target = int.Parse(args[3], CultureInfo.InvariantCulture),
            OrientationSymmetry = int.Parse(args[4], CultureInfo.InvariantCulture),
            Faces = args[5] == "3" ? FaceMode.Triangles : args[6] == "1" ? FaceMode.PureQuads : FaceMode.QuadDominant,
            SmoothingPasses = int.Parse(args[7], CultureInfo.InvariantCulture),
            StableTopology = args[8] == "1",
            AlignToBoundaries = args[9] == "1",
            Extrinsic = args[10] == "1",
            CreaseAngle = args[11] == "-" ? null : float.Parse(args[11], CultureInfo.InvariantCulture),
        };
        int runs = args.Length > 12 ? int.Parse(args[12], CultureInfo.InvariantCulture) : 0;
        var import = Stopwatch.StartNew();
        (Vector3[] vertices, int[] faces, Vector3[] normals) = ReadObj(input);
        double importSeconds = import.Elapsed.TotalSeconds;
        RemeshResult result = Solve();
        var samples = new List<object>();
        if (runs > 0)
        {
            for (int i = 0; i < 2; i++)
            {
                Solve();
            }
        }

        for (int i = 0; i < runs; i++)
        {
            long allocated = GC.GetTotalAllocatedBytes(true);
            var watch = Stopwatch.StartNew();
            RemeshResult next = Solve();
            watch.Stop();
            if (!result.Indices.AsSpan().SequenceEqual(next.Indices) ||
                !result.Positions.AsSpan().SequenceEqual(next.Positions))
            {
                throw new InvalidDataException("The managed solve changed output when repeated.");
            }

            samples.Add(new { seconds = watch.Elapsed.TotalSeconds, bytes = GC.GetTotalAllocatedBytes(true) - allocated, stages = next.Diagnostics.Timings });
        }

        if (args.Length > 13)
        {
            using var binary = new BinaryWriter(File.Create(args[13]));
            binary.Write(result.VertexCount);
            binary.Write(result.FaceCount);
            binary.Write(result.FaceArity);
            foreach (Vector3 point in result.Positions)
            {
                binary.Write(point.X);
                binary.Write(point.Y);
                binary.Write(point.Z);
            }
            foreach (int vertex in result.Indices)
            {
                binary.Write(vertex);
            }
        }

        object current = new { vertices = result.VertexCount, faces = result.FaceCount, arity = result.FaceArity };
        (int invalid, int triangles, int border) = Measure(result);
        object? comparison = null;
        if (reference != "-")
        {
            using var reader = new BinaryReader(File.OpenRead(reference));
            int nv = reader.ReadInt32(), nf = reader.ReadInt32(), arity = reader.ReadInt32();
            bool shape = nv == result.VertexCount && nf == result.FaceCount && arity == result.FaceArity;
            int? mismatches = shape ? 0 : null;
            int? firstIndexDifference = null;
            double? max = null, rms = null, maxComponent = null;
            if (shape)
            {
                double squared = 0;
                max = maxComponent = 0;
                foreach (Vector3 point in result.Positions)
                {
                    double x = point.X - (double)reader.ReadSingle();
                    double y = point.Y - (double)reader.ReadSingle();
                    double z = point.Z - (double)reader.ReadSingle();
                    max = Math.Max(max.Value, Math.Sqrt(x * x + y * y + z * z));
                    squared += x * x + y * y + z * z;
                    maxComponent = Math.Max(maxComponent.Value, Math.Max(Math.Abs(x), Math.Max(Math.Abs(y), Math.Abs(z))));
                }
                rms = Math.Sqrt(squared / nv);
                for (int i = 0; i < result.Indices.Length; i++)
                {
                    if (reader.ReadInt32() == result.Indices[i])
                    {
                        continue;
                    }

                    mismatches++;
                    firstIndexDifference ??= i;
                }
                if (reader.BaseStream.Position != reader.BaseStream.Length)
                {
                    throw new InvalidDataException("Unexpected bytes after the native mesh dump.");
                }

                if (mismatches > 0)
                {
                    max = rms = maxComponent = null;
                }
            }
            comparison = new
            {
                native = new { vertices = nv, faces = nf, arity },
                ordered = mismatches == 0,
                indexMismatches = mismatches,
                firstIndexDifference,
                maxPositionDelta = max,
                rmsPositionDelta = rms,
                maxAbsoluteCoordinateDelta = maxComponent
            };
        }

        using Process process = Process.GetCurrentProcess();
        Console.WriteLine(JsonSerializer.Serialize(new
        {
            managed = current,
            managedImportSeconds = importSeconds,
            managedColdStages = result.Diagnostics.Timings,
            managedColdSeconds = result.Diagnostics.Elapsed.TotalSeconds,
            managedSamples = samples,
            managedPeakBytes = PeakWorkingSet(process),
            result.Diagnostics.RegularizedAreaCount,
            mesh = new { invalidFaces = invalid, triangles, borderEdges = border },
            comparison
        }));
        if (invalid > 0 || result.FaceCount == 0)
        {
            throw new InvalidDataException($"Managed output has {invalid} invalid faces.");
        }

        return 0;

        RemeshResult Solve() => faces.Length == 0
            ? Remesher.RemeshPoints(vertices, normals, options)
            : Remesher.RemeshTriangles(vertices, faces, options);
    }

    private static long PeakWorkingSet(Process process)
    {
        if (OperatingSystem.IsMacOS() && IntPtr.Size == 8 && GetResourceUsage(0, out MacResourceUsage usage) == 0)
        {
            return usage.MaximumResidentBytes;
        }

        return process.PeakWorkingSet64;
    }

    // Darwin's 64-bit rusage has a pair of timevals before ru_maxrss.
    [StructLayout(LayoutKind.Explicit, Size = 144)]
    private struct MacResourceUsage
    {
        [FieldOffset(32)]
        public long MaximumResidentBytes;
    }

    [DllImport("libSystem.B.dylib", EntryPoint = "getrusage")]
    private static extern int GetResourceUsage(int who, out MacResourceUsage usage);

    internal static (Vector3[] Vertices, int[] Faces, Vector3[] Normals) ReadObj(string path)
    {
        var vertices = new List<Vector3>();
        var triangles = new List<int>();
        var normals = new List<Vector3>();
        foreach (string line in File.ReadLines(path))
        {
            string[] fields = line.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
            if (fields.Length == 0)
            {
                continue;
            }

            if (fields[0] is "v" or "vn")
            {
                Vector3 value = new(ParseFloat(fields[1]), ParseFloat(fields[2]), ParseFloat(fields[3]));
                (fields[0] == "v" ? vertices : normals).Add(value);
            }
            else if (fields[0] == "f")
            {
                for (int i = 2; i + 1 < fields.Length; i++)
                {
                    triangles.Add(ObjIndex(fields[1]));
                    triangles.Add(ObjIndex(fields[i]));
                    triangles.Add(ObjIndex(fields[i + 1]));
                }
            }
        }
        return (vertices.ToArray(), triangles.ToArray(), normals.ToArray());

        int ObjIndex(string field)
        {
            int slash = field.IndexOf('/');
            int index = int.Parse(slash < 0 ? field.AsSpan() : field.AsSpan(0, slash), CultureInfo.InvariantCulture);
            return index > 0 ? index - 1 : vertices.Count + index;
        }

        static float ParseFloat(string token) => float.Parse(token, CultureInfo.InvariantCulture);
    }

    private static (int Invalid, int Triangles, int Border) Measure(RemeshResult result)
    {
        int invalid = 0, triangles = 0;
        var incidence = new Dictionary<(int, int), int>();
        int arity = result.FaceArity;
        for (int i = 0; i < result.Indices.Length; i += arity)
        {
            ReadOnlySpan<int> face = result.Indices.AsSpan(i, arity);
            if (arity == 4 && face[2] == face[3])
            {
                face = face[..3];
            }

            if (face.Length == 3)
            {
                triangles++;
            }

            bool valid = true;
            for (int k = 0; k < face.Length; k++)
            {
                int a = face[k], b = face[(k + 1) % face.Length];
                if ((uint)a >= result.VertexCount || face[(k + 1)..].Contains(a))
                {
                    valid = false;
                    break;
                }
                var key = (Math.Min(a, b), Math.Max(a, b));
                incidence.TryGetValue(key, out int count);
                incidence[key] = count + 1;
            }
            if (!valid)
            {
                invalid++;
            }
        }

        return (invalid, triangles, incidence.Values.Count(count => count == 1));
    }
}
