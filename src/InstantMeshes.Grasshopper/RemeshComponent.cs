using Grasshopper.Kernel;
using InstantMeshes.Core;

namespace InstantMeshes.GrasshopperHost;

public sealed class RemeshComponent : RemeshComponentBase
{
    public RemeshComponent()
        : base("Instant Meshes", "IM Remesh",
            "Create a field-aligned quad, quad-dominant, or triangle mesh from a mesh or oriented point cloud. Use Options for Instant Meshes console flags.")
    {
    }

    public override Guid ComponentGuid => new("a6ba13b9-361a-4e94-8d87-82c40d927d42");

    protected override string HelpDescription => base.HelpDescription + """
        <h3>Density and Options</h3>
        <p>Leave Target Vertices and Options empty for automatic density. Counts are approximate,
        especially for pure quads, which use a final subdivision step. Put space-separated flags
        into a Panel and connect it to Options. Flags are case-sensitive. Use a dot for decimals.
        Choose one density flag. It takes precedence over Target Vertices. If repeated, the same
        density flag uses its last value.</p>
        <h3>Examples</h3>
        <ul>
          <li><code>-v 1000 -D -b</code> requests about 1,000 vertices in a quad-dominant mesh and aligns the field to open mesh boundaries.</li>
          <li><code>-s 0.5 -p 6 -S 0</code> requests triangles with a target edge length of 0.5 model units and skips smoothing.</li>
          <li><code>-f 2000 -k 16</code> targets about 2,000 faces and uses up to 16 neighbors per sample on an oriented point cloud.</li>
        </ul>
        <h3>Supported flags</h3>
        <dl>
          <dt><code>-s</code>, <code>--scale</code></dt><dd>Positive target edge length in model units.</dd>
          <dt><code>-f</code>, <code>--faces</code></dt><dd>Positive target face count.</dd>
          <dt><code>-v</code>, <code>--vertices</code></dt><dd>Positive target vertex count.</dd>
          <dt><code>-r</code>, <code>--rosy</code></dt><dd>Orientation field symmetry. Choose 2, 4 (default), or 6.</dd>
          <dt><code>-p</code>, <code>--posy</code></dt><dd>Position field symmetry. Use 4 for quads (default) or 6 for triangles. The alias 3 also selects triangles.</dd>
          <dt><code>-D</code>, <code>--dominant</code></dt><dd>Allow triangles alongside quads with position symmetry 4. The default at symmetry 4 is pure quads.</dd>
          <dt><code>-i</code>, <code>--intrinsic</code></dt><dd>Align the fields intrinsically along the surface. The default uses extrinsic alignment.</dd>
          <dt><code>-b</code>, <code>--boundaries</code></dt><dd>Align the field to open mesh boundaries. Off by default.</dd>
          <dt><code>-c</code>, <code>--crease</code></dt><dd>Detect mesh creases by dihedral angle. Enter 0–180 degrees. Negative values down to -1 disable detection. Default -1.</dd>
          <dt><code>-S</code>, <code>--smooth</code></dt><dd>Number of smoothing passes. Mesh inputs also reproject to the input surface. Default 2. Use 0 to skip.</dd>
          <dt><code>-k</code>, <code>--knn</code></dt><dd>Positive maximum neighbor count per point-cloud sample. Default 10.</dd>
          <dt><code>-t</code>, <code>--threads</code></dt><dd>Accepts a positive worker count. The managed solver uses one worker and warns if you request more.</dd>
          <dt><code>-d</code>, <code>--deterministic</code></dt><dd>Accepted for compatibility. The managed solver always runs deterministically.</dd>
          <dt><code>--stable-topology</code>, <code>--ordinary</code></dt><dd>Select stable arithmetic (default) or ordinary arithmetic. If both are present, the last flag wins.</dd>
        </dl>
        <p>Standalone file paths, output, window, and help switches are ignored with warnings.
        Export or bake the result from Mesh. Unknown switches, missing or invalid values, and
        conflicting density flags produce errors.</p>
        """ + HelpSources;

    protected override void RegisterSettings(GH_InputParamManager parameters)
    {
        int vertices = parameters.AddIntegerParameter("Target Vertices", "V",
            "Approximate vertex count. Leave empty for automatic density. -s, -f, or -v in Options overrides this input.",
            GH_ParamAccess.item);
        int options = parameters.AddTextParameter("Options", "O",
            "Optional space-separated flags, such as -v 1000 -D -b. Right-click the component and select Help for examples and the flag reference.",
            GH_ParamAccess.item);
        parameters[vertices].Optional = parameters[options].Optional = true;
    }

    protected override RemeshOptions ReadOptions(IGH_DataAccess data)
    {
        int vertices = 0;
        string? arguments = null;
        var defaults = data.GetData(1, ref vertices)
            ? new RemeshOptions { Density = DensityMode.Vertices, Target = vertices }
            : new RemeshOptions();
        data.GetData(2, ref arguments);
        (RemeshOptions options, IReadOnlyList<string> warnings) = ConsoleOptions.Apply(arguments, defaults);
        foreach (string warning in warnings)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, warning);
        }

        return options;
    }
}
