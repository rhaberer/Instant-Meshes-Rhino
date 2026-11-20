#define main unused_parity_main
#include "parity_main.cpp"
#undef main

int main(int argc, char **argv) {
    if (argc != 11) return 2;
    Mesh mesh;
    std::vector<float> normals;
    std::string input(argv[1]);
    if (input.size() > 4 && input.substr(input.size() - 4) == ".ply") {
        std::ifstream file(input);
        std::string line;
        while (std::getline(file, line) && line != "end_header") {}
        float x, y, z, nx, ny, nz;
        while (file >> x >> y >> z >> nx >> ny >> nz) {
            mesh.positions.insert(mesh.positions.end(), {x, y, z});
            normals.insert(normals.end(), {nx, ny, nz});
        }
        if (mesh.positions.empty()) return 2;
    } else if (!load_obj(argv[1], mesh)) return 2;
    im_options options;
    im_options_default(&options);
    options.target_vertex_count = std::atoi(argv[2]);
    options.browser_stable = std::atoi(argv[3]);
    options.rosy = std::atoi(argv[4]);
    options.posy = std::strcmp(argv[5], "Triangles") == 0 ? 3 : 4;
    options.pure_quad = std::strcmp(argv[5], "QuadDominant") != 0;
    options.smooth_iterations = std::atoi(argv[6]);
    options.align_to_boundaries = std::atoi(argv[7]);
    options.crease_angle = std::strcmp(argv[8], "-") == 0 ? -1 : std::strtof(argv[8], nullptr);
    options.knn_points = std::atoi(argv[9]);
    im_result *result = im_remesh(mesh.positions.data(), mesh.positions.size() / 3,
        mesh.indices.data(), mesh.indices.size(), normals.empty() ? nullptr : normals.data(), &options);
    if (!im_result_ok(result)) {
        std::fprintf(stderr, "%s\n", im_result_error(result));
        im_result_free(result);
        return 1;
    }
    uint32_t nv = im_result_vertex_count(result), nf = im_result_face_count(result),
        arity = im_result_vertices_per_face(result);
    std::ofstream out(argv[10], std::ios::binary);
    uint32_t header[] = {nv, nf, arity};
    out.write(reinterpret_cast<const char *>(header), sizeof(header));
    out.write(reinterpret_cast<const char *>(im_result_positions(result)), 3 * nv * sizeof(float));
    out.write(reinterpret_cast<const char *>(im_result_indices(result)), nf * arity * sizeof(uint32_t));
    im_result_free(result);
    return out ? 0 : 1;
}
