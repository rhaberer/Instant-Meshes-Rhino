using System.Globalization;
using System.Text;
using InstantMeshes.Core;

namespace InstantMeshes.GrasshopperHost;

/// <summary>Applies Instant Meshes batch flags to the Grasshopper inputs.</summary>
internal static class ConsoleOptions
{
    public static (RemeshOptions Options, IReadOnlyList<string> Warnings) Apply(string? arguments, RemeshOptions defaults)
    {
        if (string.IsNullOrWhiteSpace(arguments))
        {
            return (defaults, Array.Empty<string>());
        }

        string[] tokens = Tokenize(arguments).ToArray();
        RemeshOptions options = defaults;
        var warnings = new List<string>();
        bool hasInputPath = false;
        // A supplied flag overrides the component default, but density flags cannot conflict.
        DensityMode? consoleDensity = null;
        int positionSymmetry = options.Faces == FaceMode.Triangles ? 6 : 4;
        bool dominant = options.Faces == FaceMode.QuadDominant;
        for (int i = 0; i < tokens.Length; i++)
        {
            string flag = tokens[i];
            switch (flag)
            {
                case "-s":
                case "--scale":
                    SetDensity(DensityMode.EdgeLength, ReadNumber(ReadValue(), flag, double.Epsilon, float.MaxValue));
                    break;
                case "-f":
                case "--faces":
                    SetDensity(DensityMode.Faces, ReadInteger(ReadValue(), flag, 1));
                    break;
                case "-v":
                case "--vertices":
                    SetDensity(DensityMode.Vertices, ReadInteger(ReadValue(), flag, 1));
                    break;
                case "-r":
                case "--rosy":
                    int rosy = ReadInteger(ReadValue(), flag, 2);
                    if (rosy is not (2 or 4 or 6))
                    {
                        throw new ArgumentException($"{flag} expects 2, 4 or 6.");
                    }

                    options = options with { OrientationSymmetry = rosy };
                    break;
                case "-p":
                case "--posy":
                    positionSymmetry = ReadInteger(ReadValue(), flag, 3);
                    if (positionSymmetry is not (3 or 4 or 6))
                    {
                        throw new ArgumentException($"{flag} expects 4 or 6. The original alias 3 also selects 6.");
                    }

                    break;
                case "-D":
                case "--dominant":
                    dominant = true;
                    break;
                case "-i":
                case "--intrinsic":
                    options = options with { Extrinsic = false };
                    break;
                case "-b":
                case "--boundaries":
                    options = options with { AlignToBoundaries = true };
                    break;
                case "-c":
                case "--crease":
                    float angle = (float)ReadNumber(ReadValue(), flag, -1, 180);
                    options = options with { CreaseAngle = angle < 0 ? null : angle };
                    break;
                case "-S":
                case "--smooth":
                    options = options with { SmoothingPasses = ReadInteger(ReadValue(), flag, 0) };
                    break;
                case "-k":
                case "--knn":
                    options = options with { PointNeighbors = ReadInteger(ReadValue(), flag, 1) };
                    break;
                case "-t":
                case "--threads":
                    if (ReadInteger(ReadValue(), flag, 1) != 1)
                    {
                        warnings.Add($"{flag} ignored. The managed solver uses one worker.");
                    }

                    break;
                case "-d":
                case "--deterministic":
                    // The managed solver already runs deterministically. Stable topology is a separate numeric option.
                    break;
                case "--stable-topology":
                    options = options with { StableTopology = true };
                    break;
                case "--ordinary":
                    options = options with { StableTopology = false };
                    break;
                case "-o":
                case "--output":
                    ReadValue();
                    warnings.Add($"{flag} ignored. Use the component's Mesh output to export geometry.");
                    break;
                case "-F":
                case "--fullscreen":
                case "-C":
                case "--compat":
                    warnings.Add($"{flag} ignored. It applies to the standalone Instant Meshes window.");
                    break;
                case "-h":
                case "--help":
                    warnings.Add($"{flag} ignored. See the component description for supported flags.");
                    break;
                case var finderMarker when finderMarker.StartsWith("-psn", StringComparison.Ordinal):
                    warnings.Add("The macOS Finder launch marker was ignored.");
                    break;
                default:
                    if (flag.Length == 0 || flag[0] == '-')
                    {
                        throw new ArgumentException($"Unknown argument '{flag}'.");
                    }

                    if (hasInputPath)
                    {
                        throw new ArgumentException("The original CLI accepts only one input file. Connect geometry directly in Grasshopper.");
                    }

                    hasInputPath = true;
                    warnings.Add($"Input path '{flag}' ignored. The component uses the Geometry input.");
                    break;
            }

            string ReadValue()
            {
                if (++i >= tokens.Length || tokens[i].Length == 0)
                {
                    throw new ArgumentException($"{flag} requires a value.");
                }

                string value = tokens[i];
                if (value.Length > 1 && value[0] == '-' && (value[1] == '-' || char.IsLetter(value[1])))
                {
                    throw new ArgumentException($"{flag} requires a value before '{value}'.");
                }

                return value;
            }
        }

        // Resolve these together so -D and -p work in either argument order.
        FaceMode faces = positionSymmetry == 4
            ? dominant ? FaceMode.QuadDominant : FaceMode.PureQuads
            : FaceMode.Triangles;
        return (options with { Faces = faces }, warnings.AsReadOnly());

        void SetDensity(DensityMode mode, double target)
        {
            if (consoleDensity is not null && consoleDensity != mode)
            {
                throw new ArgumentException("Choose only one of --scale, --faces and --vertices.");
            }

            consoleDensity = mode;
            options = options with { Density = mode, Target = target };
        }
    }

    private static int ReadInteger(string text, string flag, int minimum)
    {
        if (!int.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture, out int value) || value < minimum)
        {
            throw new ArgumentException($"{flag} expects an integer of at least {minimum}.");
        }

        return value;
    }

    private static double ReadNumber(string text, string flag, double minimum, double maximum)
    {
        if (!double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out double value) ||
            !double.IsFinite(value) || value < minimum || value > maximum)
        {
            throw new ArgumentException($"{flag} has an invalid numeric value '{text}'. Use a decimal point.");
        }

        return value;
    }

    private static IEnumerable<string> Tokenize(string text)
    {
        var token = new StringBuilder();
        char quote = '\0';
        bool started = false;
        foreach (char ch in text)
        {
            if (quote != '\0')
            {
                if (ch == quote)
                {
                    quote = '\0';
                }
                else
                {
                    token.Append(ch);
                }
            }
            else if (ch is '"' or '\'')
            {
                quote = ch;
                started = true;
            }
            else if (char.IsWhiteSpace(ch))
            {
                if (started)
                {
                    yield return token.ToString();
                    token.Clear();
                    started = false;
                }
            }
            else
            {
                token.Append(ch);
                started = true;
            }
        }
        if (quote != '\0')
        {
            throw new ArgumentException("The argument string contains an unclosed quote.");
        }

        if (started)
        {
            yield return token.ToString();
        }
    }
}
