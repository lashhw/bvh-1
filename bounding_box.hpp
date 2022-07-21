#ifndef BVH_BOUNDING_BOX_HPP
#define BVH_BOUNDING_BOX_HPP

struct BoundingBox {
    BoundingBox() : bounds {
        FLT_MAX, -FLT_MAX,
        FLT_MAX, -FLT_MAX,
        FLT_MAX, -FLT_MAX
    } { }

    void extend(const BoundingBox &other) {
        bounds[0] = std::min(bounds[0], other.bounds[0]);
        bounds[1] = std::max(bounds[1], other.bounds[1]);
        bounds[2] = std::min(bounds[2], other.bounds[2]);
        bounds[3] = std::max(bounds[3], other.bounds[3]);
        bounds[4] = std::min(bounds[4], other.bounds[4]);
        bounds[5] = std::max(bounds[5], other.bounds[5]);
    }

    float half_area() {
        float e1 = bounds[1] - bounds[0];
        float e2 = bounds[3] - bounds[2];
        float e3 = bounds[5] - bounds[4];
        return (e1 + e2) * e3 + e1 * e2;
    }

    float bounds[6];  // [xmin, xmax, ymin, ymax, zmin, zmax]
};

#endif //BVH_BOUNDING_BOX_HPP
