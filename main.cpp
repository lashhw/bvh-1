#include <iostream>
#include <algorithm>
#include <stack>
#include <array>
#include <vector>
#include <cmath>
#include <cfloat>
#include <numeric>

#include "vec3.hpp"
#include "ray.hpp"
#include "bounding_box.hpp"
#include "triangle.hpp"
#include "aabb_intersector.hpp"
#include "bvh.hpp"

int main() {
    std::vector<Triangle> triangles;
    triangles.emplace_back(Vec3( 0,  0, 0), Vec3( 1,  2,  3), Vec3( 0,  1,  2));
    triangles.emplace_back(Vec3( 2,  1, 0), Vec3( 0,  2,  1), Vec3( 2,  4,  1));
    triangles.emplace_back(Vec3( 2, -4, 1), Vec3( 1, -3, -1), Vec3( 4, -4,  0));

    Bvh bvh(&triangles);

    Vec3 origin(3, -5, 0);
    Vec3 direction(-2, 9, 1);
    Ray ray(origin, direction);
    Triangle::Intersection intersection;
    bool hit = bvh.traverse(ray, intersection);

    if (hit) {
        Vec3 hit_point = origin + intersection.t * direction;
        std::cout << "Hit: " << hit_point.x << ' ' << hit_point.y << ' ' << hit_point.z << std::endl;
    }
}
