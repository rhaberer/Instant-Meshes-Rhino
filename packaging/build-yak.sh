#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "$0")/.." && pwd)"
release_dir="${1:-$project/artifacts/release}"
platforms=("${@:2}")
if [[ "${#platforms[@]}" == 0 ]]; then
    platforms=(mac)
fi
for platform in "${platforms[@]}"; do
    case "$platform" in
        mac|win) ;;
        *) printf 'Unsupported Yak platform: %s\n' "$platform" >&2; exit 1 ;;
    esac
done
yak_tool="${YAK_TOOL:-/Applications/Rhino 8.app/Contents/Resources/bin/yak}"
mkdir -p "$release_dir"
release_dir="$(cd "$release_dir" && pwd)"

dotnet build "$project/src/InstantMeshes.Rhino" -c Release --disable-build-servers -m:1 -p:NuGetAudit=false
dotnet build "$project/src/InstantMeshes.Grasshopper" -c Release --disable-build-servers -m:1 -p:NuGetAudit=false

package_stage="$(mktemp -d "${TMPDIR:-/tmp}/instant-meshes-yak.XXXXXXXX")"
trap 'rm -rf -- "$package_stage"' EXIT
rhino="$project/src/InstantMeshes.Rhino/bin/Release/net8.0"
grasshopper="$project/src/InstantMeshes.Grasshopper/bin/Release/net8.0"
cp "$rhino/InstantMeshes.Rhino.rhp" "$rhino/InstantMeshes.Core.dll" \
   "$rhino/InstantMeshes.RhinoGeometry.dll" "$grasshopper/InstantMeshes.Grasshopper.gha" "$package_stage/"
cp "$project/packaging/manifest.yml" "$project/packaging/icon.png" "$package_stage/"
cp "$project/packaging/README.md" "$project/NOTICE.md" "$project"/LICENSE* "$package_stage/"
mkdir "$package_stage/examples" "$package_stage/source"
cp "$project/examples/Instant Meshes - Options.gh" "$project/examples/Instant Meshes - Typed.gh" "$package_stage/examples/"
cp "$project/src/InstantMeshes.Core/ProjectionSvd.cs" "$package_stage/source/"
(
    cd "$package_stage"
    for platform in "${platforms[@]}"; do
        "$yak_tool" build --platform "$platform"
        mv ./*.yak "$release_dir/"
    done
)
