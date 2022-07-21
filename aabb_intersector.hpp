#ifndef BVH_AABB_INTERSECTOR_HPP
#define BVH_AABB_INTERSECTOR_HPP

struct AABBIntersector {
    AABBIntersector(const Ray &ray)
        : octant { std::signbit(ray.direction.x),
                   std::signbit(ray.direction.y),
                   std::signbit(ray.direction.z) } {
        inverse_direction = Vec3(1.0f / ray.direction.x,
                                 1.0f / ray.direction.y,
                                 1.0f / ray.direction.z);
        scaled_origin = -ray.origin * inverse_direction;
    }

    bool intersect(const BoundingBox &bbox, const Ray &ray, float &entry) const {
        entry = std::max({
            inverse_direction.x * bbox.bounds[0 + octant[0]] + scaled_origin.x,
            inverse_direction.y * bbox.bounds[2 + octant[1]] + scaled_origin.y,
            inverse_direction.z * bbox.bounds[4 + octant[2]] + scaled_origin.z
        });

        float exit = std::min({
            inverse_direction.x * bbox.bounds[1 - octant[0]] + scaled_origin.x,
            inverse_direction.y * bbox.bounds[3 - octant[1]] + scaled_origin.y,
            inverse_direction.z * bbox.bounds[5 - octant[2]] + scaled_origin.z
        });

        return entry <= exit;
    }

    int octant[3];
    Vec3 inverse_direction;
    Vec3 scaled_origin;
};

#endif //BVH_AABB_INTERSECTOR_HPP
