#include <iostream>
#include <algorithm>
#include <stack>
#include <array>
#include <vector>
#include <cmath>
#include <fstream>
#include <cfloat>
#include <numeric>

#include "vec3.hpp"
#include "ray.hpp"
#include "bounding_box.hpp"
#include "triangle.hpp"
#include "aabb_intersector.hpp"
#include "bvh.hpp"
#include "happly.h"

int main() {
    happly::PLYData ply_in("../bun_zipper.ply");
    std::vector<std::array<double, 3>> v_pos = ply_in.getVertexPositions();
    std::vector<std::vector<size_t>> f_index = ply_in.getFaceIndices<size_t>();

    std::vector<Triangle> triangles;
    for (const auto &face : f_index) {
        triangles.emplace_back(Vec3(v_pos[face[0]][0], v_pos[face[0]][1], v_pos[face[0]][2]),
                               Vec3(v_pos[face[1]][0], v_pos[face[1]][1], v_pos[face[1]][2]),
                               Vec3(v_pos[face[2]][0], v_pos[face[2]][1], v_pos[face[2]][2]));
    }

    std::cout << "Building BVH... " << std::flush;
    Bvh bvh(&triangles);
    std::cout << "done" << std::endl;

    Vec3 origin(0, 0, 1);
    Vec3 upper_left_corner(-0.12, 0.2, 0);
    Vec3 horizontal(0.2, 0, 0);
    Vec3 vertical(0, -0.2, 0);

    float min_t = FLT_MAX;
    float max_t = 0;
    float t[800][800];
    bool miss[800][800];
    for (int j = 0; j < 800; j++) {
        for (int i = 0; i < 800; i++) {
            Vec3 lookat = upper_left_corner + horizontal * (i / 800.0f) + vertical * (j / 800.0f);
            Ray ray(origin, lookat - origin);

            Triangle::Intersection intersection;
            bool hit = bvh.traverse(ray, intersection);
            if (hit) {
                min_t = std::min(min_t, intersection.t);
                max_t = std::max(max_t, intersection.t);
                t[i][j] = intersection.t;
                miss[i][j] = false;
            }
            else miss[i][j] = true;
        }
    }

    std::ofstream file("../image.ppm");
    file << "P3\n800 800\n255\n";
    for (int j = 0; j < 800; j++) {
        for (int i = 0; i < 800; i++) {
            if (miss[i][j]) file << "0 0 0 ";
            else {
                float normalized_t = (t[i][j] - min_t) / (max_t - min_t);
                float color = std::sqrt(1 - normalized_t);
                int r = std::min(255, int(color * 255));
                file << r << " " << r << " " << r << " ";
            }
        }
    }
}
