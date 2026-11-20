# Contributing

Use the .NET SDK from the repository root. See the [README](README.md#build-and-test-locally) for build and test commands.

## Format C# code

The EditorConfig follows common .NET conventions, including four-space indentation, Allman brace placement, braced control blocks, and file-scoped namespaces. Use a 300-character line-length guide for C#. The formatter applies the supported style rules across all C# projects. Wrap longer lines manually. [Microsoft C# coding conventions](https://learn.microsoft.com/en-us/dotnet/csharp/fundamentals/coding-style/coding-conventions)

```sh
bash scripts/format-csharp.sh
```

Append `--verify-no-changes` to check formatting without modifying files.
