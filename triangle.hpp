#ifndef BVH_TRIANGLE_HPP
#define BVH_TRIANGLE_HPP

struct Triangle {
    Triangle() { }
    Triangle(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2)
            : p0(p0), e1(p0-p1), e2(p2-p0), n(cross(e1, e2)) { }

    Vec3 p1() const { return p0 - e1; }
    Vec3 p2() const { return p0 + e2; }

    BoundingBox bounding_box() const {
        BoundingBox bbox;

        Vec3 p1_ = p1();
        Vec3 p2_ = p2();

        bbox.bounds[0] = std::min({p0.x, p1_.x, p2_.x});
        bbox.bounds[2] = std::min({p0.y, p1_.y, p2_.y});
        bbox.bounds[4] = std::min({p0.z, p1_.z, p2_.z});

        bbox.bounds[1] = std::max({p0.x, p1_.x, p2_.x});
        bbox.bounds[3] = std::max({p0.y, p1_.y, p2_.y});
        bbox.bounds[5] = std::max({p0.z, p1_.z, p2_.z});

        return bbox;
    }

    Vec3 center() const {
        return (p0 + p1() + p2()) * (1.0f / 3.0f);
    }

    struct Intersection {
        float t, u, v;
    };

    bool intersect(const Ray &ray, Intersection &intersection) const {
        Vec3 c = p0 - ray.origin;
        Vec3 r = cross(ray.direction, c);
        float inv_det = 1.0f / dot(ray.direction, n);

        float u = inv_det * dot(e2, r);
        float v = inv_det * dot(e1, r);

        if (u >= 0.0f && v >= 0.0f && (u + v) <= 1.0f) {
            float t = inv_det * dot(c, n);
            if (ray.tmin <= t && t <= ray.tmax) {
                intersection = {t, u, v};
                return true;
            }
        }

        return false;
    }

    Vec3 p0, e1, e2, n;
};


#endif //BVH_TRIANGLE_HPP
