#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "$0")/.." && pwd)"
tag="${1:?Usage: bash packaging/build-release.sh vMAJOR.MINOR.PATCH [output directory]}"
if [[ ! "$tag" =~ ^v[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    printf 'Expected a release tag like v1.0.0, got %s\n' "$tag" >&2
    exit 1
fi

version="${tag#v}"
manifest_version="$(sed -n 's/^version: *//p' "$project/packaging/manifest.yml")"
assembly_version="$(sed -n 's/^[[:space:]]*<Version>\(.*\)<\/Version>$/\1/p' "$project/src/Directory.Build.props")"
if [[ "$version" != "$manifest_version" || "$version" != "$assembly_version" ]]; then
    printf 'Tag %s must match the Yak manifest version (%s) and project version (%s).\n' \
        "$tag" "$manifest_version" "$assembly_version" >&2
    exit 1
fi

release_dir="${2:-$project/artifacts/release/$tag}"
stage="$(mktemp -d "${TMPDIR:-/tmp}/instant-meshes-release.XXXXXXXX")"
trap 'rm -rf -- "$stage"' EXIT

bash "$project/packaging/build-yak.sh" "$stage" mac win
shopt -s nullglob
packages=()
for platform in mac win; do
    platform_packages=("$stage"/instant-meshes-rhino-"$version"-rh8*-"$platform".yak)
    if [[ "${#platform_packages[@]}" != 1 ]]; then
        printf 'Expected exactly one Rhino 8 Yak package for %s on %s.\n' "$tag" "$platform" >&2
        exit 1
    fi
    unzip -tq "${platform_packages[0]}"
    packages+=("${platform_packages[0]}")
done

cp "$project/src/InstantMeshes.Grasshopper/bin/Release/net8.0/InstantMeshes.Grasshopper.gha" "$stage/"
repository="${GITHUB_SERVER_URL:-https://github.com}/${GITHUB_REPOSITORY:-rhaberer/Instant-Meshes-Rhino}"
{
    printf 'Instant Meshes for Rhino and Grasshopper %s — licenses and notices\n\n' "$version"
    printf 'Corresponding source, including the MPL-2.0-covered ProjectionSvd.cs:\n\n'
    printf '%s/tree/%s\n%s/blob/%s/src/InstantMeshes.Core/ProjectionSvd.cs\n\n' \
        "$repository" "$tag" "$repository" "$tag"
    cat "$project/NOTICE.md"
    for license in "$project"/LICENSE*; do
        printf '\n\n===== %s =====\n\n' "${license##*/}"
        cat "$license"
    done
} > "$stage/InstantMeshes-Licenses-and-Notices.txt"

mkdir -p "$release_dir"
cp "${packages[@]}" "$stage/InstantMeshes.Grasshopper.gha" \
    "$stage/InstantMeshes-Licenses-and-Notices.txt" "$release_dir/"
