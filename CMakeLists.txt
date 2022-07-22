cmake_minimum_required(VERSION 3.22)
project(bvh)

set(CMAKE_CXX_STANDARD 11)

add_executable(bvh main.cpp vec3.hpp triangle.hpp bvh.hpp bounding_box.hpp ray.hpp aabb_intersector.hpp matrix4x4.hpp transform.hpp)
