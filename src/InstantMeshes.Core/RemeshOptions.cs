namespace InstantMeshes.Core;

public enum DensityMode
{
    Auto,
    EdgeLength,
    Faces,
    Vertices
}

public enum FaceMode
{
    PureQuads,
    QuadDominant,
    Triangles
}

public sealed record RemeshOptions
{
    public DensityMode Density { get; init; }
    public double Target { get; init; }
    public FaceMode Faces { get; init; }
    public int OrientationSymmetry { get; init; } = 4;
    public bool Extrinsic { get; init; } = true;
    public bool AlignToBoundaries { get; init; }
    public float? CreaseAngle { get; init; }
    public int SmoothingPasses { get; init; } = 2;
    public int PointNeighbors { get; init; } = 10;
    public bool StableTopology { get; init; } = true;

    // Triangular position fields have three axes; the UI calls this sixfold symmetry.
    internal int PositionSymmetry => Faces == FaceMode.Triangles ? 3 : 4;
}
