# Instant Meshes Rhino notices

Original contributions to this project are licensed under the [MIT License](LICENSE). Third-party code and adapted materials retain the terms listed below.

The managed solver adapts **Instant Field-Aligned Meshes** by Wenzel Jakob, Daniele Panozzo, Marco Tarini and Olga Sorkine-Hornung, ACM Transactions on Graphics, SIGGRAPH Asia 2015. The upstream terms are in [LICENSE-instant-meshes](LICENSE-instant-meshes). This project uses the native port at `cc463c0c509e8c50238a648c6cd92df8f3ad3c97`, based on upstream `7b3160864a2e1025af498c84cfed91cbfb613698`, for reference. Adapted algorithm files identify their origin. [Paper and source](https://igl.ethz.ch/projects/instant-meshes/)

The Yak package includes all license texts and these notices.

- `LICENSE-pcg32` contains the Apache License 2.0 for the adapted PCG32 random-number generator in `Pcg32.cs`, credited to Melissa O’Neill and Wenzel Jakob.
- `LICENSE-dset` contains Wenzel Jakob’s terms for the adapted disjoint-set code in `DisjointSets.cs`, which tracks groups as the solver merges them.
- `LICENSE-eigen` contains MPL-2.0 for the Eigen scalar Jacobi SVD adapted in `ProjectionSvd.cs`. The corresponding source accompanies the binaries at `source/ProjectionSvd.cs` in the Yak package.

The bunny mesh embedded in the Grasshopper examples and the Yak package icon were generated for this project. The Grasshopper component icon and its larger README artwork are adaptations of the original Instant Meshes icon and retain the upstream terms. [Original icon](https://github.com/wjakob/instant-meshes/blob/7b3160864a2e1025af498c84cfed91cbfb613698/resources/icon.png)

The optional Stanford Bunny and Armadillo test meshes are attributed to the Stanford University Computer Graphics Laboratory. The test fixture guide records the provider and preparation steps. [Stanford 3D Scanning Repository](https://graphics.stanford.edu/data/3Dscanrep/)
