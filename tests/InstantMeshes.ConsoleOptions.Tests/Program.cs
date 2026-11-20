using InstantMeshes.Core;
using InstantMeshes.GrasshopperHost;

var defaults = new RemeshOptions { Density = DensityMode.Vertices, Target = 500 };
Require(ConsoleOptions.Apply("", defaults).Options == defaults, "empty flags preserve defaults");
var (advanced, advancedWarnings) = ConsoleOptions.Apply(
    "--faces '800' -D -b -r 2 -S 0 --intrinsic -c 32.5 --knn 12 -d -t 1 --ordinary", defaults);
Require(advancedWarnings.Count == 0, "supported flags have no warnings");
Require(advanced is
{
    Density: DensityMode.Faces, Target: 800, Faces: FaceMode.QuadDominant,
    OrientationSymmetry: 2, SmoothingPasses: 0, AlignToBoundaries: true, Extrinsic: false,
    CreaseAngle: 32.5f, PointNeighbors: 12, StableTopology: false
}, "complete option mapping");
Require(ConsoleOptions.Apply("-r 6 --posy 6 -D -v 120", defaults).Options is
{ OrientationSymmetry: 6, Faces: FaceMode.Triangles, Target: 120 }, "triangular extraction");
Require(ConsoleOptions.Apply("-p 3 -s \"1.25\"", defaults).Options is
{ Faces: FaceMode.Triangles, Density: DensityMode.EdgeLength, Target: 1.25 }, "legacy PoSy alias and quotes");
Require(ConsoleOptions.Apply("-c -1 -S 2 -v 100 --vertices 200", defaults).Options is
{ CreaseAngle: null, SmoothingPasses: 2, Target: 200 }, "crease off and repeated alias");
var (mixed, warnings) = ConsoleOptions.Apply(
    "-F -C --help -psn_0_42 -t 2 -o \"some file.obj\" model.ply -v 750 -b", defaults);
Require(mixed is { Target: 750, AlignToBoundaries: true }, "valid flags apply alongside ignored arguments");
Require(warnings.Count == 7 && warnings.All(warning => warning.Contains("ignored", StringComparison.OrdinalIgnoreCase)),
    "each known unsupported argument emits a warning");

foreach (string invalid in new[] { "-s 1 -f 20", "-t 0", "-v", "-c 181", "-r 3", "-p 5",
                                   "-v '20", "-v 0", "--scale NaN", "-S -1", "-o", "-o -v 100",
                                   "--unknown", "a.obj b.obj" })
{
    try
    {
        ConsoleOptions.Apply(invalid, defaults);
        throw new Exception($"Accepted invalid input {invalid}.");
    }
    catch (ArgumentException)
    {
    }
}
Console.WriteLine("Verified console option mapping, ignored-argument warnings, and validation errors.");

static void Require(bool value, string test)
{
    if (!value)
    {
        throw new Exception($"Failed {test}.");
    }
}
