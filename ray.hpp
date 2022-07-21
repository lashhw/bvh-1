#ifndef BVH_RAY_HPP
#define BVH_RAY_HPP

struct Ray {
    Ray() { }
    Ray(const Vec3 &origin,
        const Vec3 &direction,
        float tmin = 0.0f,
        float tmax = FLT_MAX)
        : origin(origin), direction(direction), tmin(tmin), tmax(tmax) { }
    Vec3 at(float t) const { return origin + t * direction; }

    Vec3 origin;
    Vec3 direction;
    float tmin;
    float tmax;
};

#endif //BVH_RAY_HPP
