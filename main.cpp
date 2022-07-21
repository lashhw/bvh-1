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

    std::ofstream file("../image.ppm");
    file << "P3\n800 800\n255\n";
    for (int j = 0; j < 800; j++) {
        for (int i = 0; i < 800; i++) {
            Vec3 lookat = upper_left_corner + horizontal * (i / 800.0f) + vertical * (j / 800.0f);
            Ray ray(origin, lookat - origin);

            Bvh::Intersection intersection;
            bool hit = bvh.traverse(ray, intersection);
            if (hit) {
                Vec3 color = triangles[intersection.index].n;
                float length = std::sqrt(color.x * color.x + color.y * color.y + color.z * color.z);
                color = (color / length) + Vec3(1, 1, 1);
                color = color / 2;
                file << (int)(color.x * 255) << " " << (int)(color.y * 255) << " " << (int)(color.z * 255) << " ";
            }
            else file << "0 0 0 ";
        }
    }
}
