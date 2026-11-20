using Grasshopper.Kernel;
using InstantMeshes.Core;

namespace InstantMeshes.GrasshopperHost;

public sealed class TypedRemeshComponent : RemeshComponentBase
{
    public TypedRemeshComponent()
        : base("Instant Meshes (Typed)", "IM Remesh Typed", "Remesh with separate Grasshopper inputs for each solver setting.")
    {
    }

    public override Guid ComponentGuid => new("b96819cc-ca44-4cf2-b043-30d8b232f247");

    protected override string HelpDescription => base.HelpDescription + """
        <h3>Density and face type</h3>
        <p>Connect one density target (Edge Length, Target Faces, or Target Vertices) or leave all
        three empty for automatic density. Counts are approximate. Edge Length uses model units.
        Use Position Symmetry 4 for quads and turn on Quad Dominant to allow triangles alongside
        quads. Use Position Symmetry 6 for triangles. For example, connect 1,000 to Target Vertices
        and set Quad Dominant and Align Boundaries to True.</p>
        <p>Align Boundaries and Crease Angle apply to meshes. Smoothing Passes applies to both
        geometry types and also reprojects mesh results onto the input surface. Point Neighbors
        controls the neighborhood size used for point clouds. The managed solver uses one worker
        and deterministic execution.</p>
        """ + HelpSources;

    protected override void RegisterSettings(GH_InputParamManager parameters)
    {
        parameters[parameters.AddNumberParameter("Edge Length", "L", "Target edge length in document units. Leave empty for Auto.",
            GH_ParamAccess.item)].Optional = true;
        parameters[parameters.AddIntegerParameter("Target Faces", "F", "Approximate face count. Leave empty for Auto.",
            GH_ParamAccess.item)].Optional = true;
        parameters[parameters.AddIntegerParameter("Target Vertices", "V", "Approximate vertex count. Leave empty for Auto.",
            GH_ParamAccess.item)].Optional = true;
        parameters.AddIntegerParameter("Orientation Symmetry", "R", "2, 4 or 6. Default 4.",
            GH_ParamAccess.item, 4);
        parameters.AddIntegerParameter("Position Symmetry", "P", "4 for quads or 6 for triangles. Default 4.",
            GH_ParamAccess.item, 4);
        parameters.AddBooleanParameter("Quad Dominant", "Q", "Mix triangles and quads with position symmetry 4. Default false.",
            GH_ParamAccess.item, false);
        parameters.AddBooleanParameter("Intrinsic", "I", "Use intrinsic field alignment. Default false.",
            GH_ParamAccess.item, false);
        parameters.AddBooleanParameter("Align Boundaries", "B", "Align the field to mesh boundaries. Default false.",
            GH_ParamAccess.item, false);
        parameters.AddNumberParameter("Crease Angle", "C", "Dihedral threshold in degrees. Default -1 disables creases.",
            GH_ParamAccess.item, -1);
        parameters.AddIntegerParameter("Smoothing Passes", "S", "Smoothing and reprojection passes. Default 2.",
            GH_ParamAccess.item, 2);
        parameters.AddIntegerParameter("Point Neighbors", "K", "Maximum oriented neighbors per point. Default 10.",
            GH_ParamAccess.item, 10);
        parameters.AddBooleanParameter("Stable Topology", "ST", "Use canonical arithmetic. Default true.",
            GH_ParamAccess.item, true);
        parameters.AddIntegerParameter("Worker Limit", "W", "The managed solver uses one worker.",
            GH_ParamAccess.item, 1);
        parameters.AddBooleanParameter("Deterministic", "DT", "The managed solver uses deterministic execution.",
            GH_ParamAccess.item, true);
    }

    protected override RemeshOptions ReadOptions(IGH_DataAccess data)
    {
        double scale = 0, crease = -1;
        int faceCount = 0, vertices = 0;
        int orientation = 4, position = 4, passes = 2, neighbors = 10, workers = 1;
        bool dominant = false, intrinsic = false, boundary = false;
        bool stable = true, deterministic = true;
        bool hasScale = data.GetData(1, ref scale);
        bool hasFaces = data.GetData(2, ref faceCount);
        bool hasVertices = data.GetData(3, ref vertices);
        if ((hasScale && (hasFaces || hasVertices)) || (hasFaces && hasVertices))
        {
            throw new ArgumentException("Connect at most one density target.");
        }

        data.GetData(4, ref orientation);
        data.GetData(5, ref position);
        data.GetData(6, ref dominant);
        data.GetData(7, ref intrinsic);
        data.GetData(8, ref boundary);
        data.GetData(9, ref crease);
        data.GetData(10, ref passes);
        data.GetData(11, ref neighbors);
        data.GetData(12, ref stable);
        data.GetData(13, ref workers);
        data.GetData(14, ref deterministic);
        if (workers < 1)
        {
            throw new ArgumentOutOfRangeException(nameof(workers), "Worker limit must be positive.");
        }

        if (workers > 1)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Worker limit ignored. The managed solver uses one worker.");
        }

        if (!deterministic)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "The managed solver always uses deterministic execution.");
        }

        FaceMode faceMode = position switch
        {
            4 => dominant ? FaceMode.QuadDominant : FaceMode.PureQuads,
            3 or 6 => FaceMode.Triangles,
            _ => throw new ArgumentException("Position symmetry must be 4 or 6.")
        };
        return new RemeshOptions
        {
            Density = hasScale ? DensityMode.EdgeLength
                : hasFaces ? DensityMode.Faces
                : hasVertices ? DensityMode.Vertices
                : DensityMode.Auto,
            Target = hasScale ? scale : hasFaces ? faceCount : vertices,
            Faces = faceMode,
            OrientationSymmetry = orientation,
            Extrinsic = !intrinsic,
            AlignToBoundaries = boundary,
            CreaseAngle = crease == -1 ? null : (float)crease,
            SmoothingPasses = passes,
            PointNeighbors = neighbors,
            StableTopology = stable
        };
    }
}
