#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

# Format each physical source directory once. Grasshopper and its tests compile
# linked files, which can conflict if multiple projects format them together.
while IFS= read -r -d '' project; do
    directory="$(dirname "$project")"
    sources=()
    while IFS= read -r -d '' source; do
        sources+=("$source")
    done < <(find "$directory" -type d \( -name bin -o -name obj \) -prune \
        -o -type f -name '*.cs' -print0)
    dotnet format style "$project" --include "${sources[@]}" \
        --diagnostics IDE0011 IDE0065 IDE0161 --severity info "$@"
    dotnet format whitespace "$project" --include "${sources[@]}" "$@"
done < <(find src tests -type f -name '*.csproj' -print0)
