#ifndef BVH_VEC3_HPP
#define BVH_VEC3_HPP

struct Vec3 {
    Vec3() { }
    Vec3(float x, float y, float z) : x(x), y(y), z(z) { }

    Vec3 operator-() const { return Vec3(-x, -y, -z); }
    float& operator[](int i);

    Vec3& operator+=(const Vec3 &v2);
    Vec3& operator-=(const Vec3 &v2);
    Vec3& operator*=(const Vec3 &v2);
    Vec3& operator/=(const Vec3 &v2);
    Vec3& operator*=(float t);
    Vec3& operator/=(float t);

    float x, y, z;
};

float& Vec3::operator[](int i) {
    if (i == 0) return x;
    else if (i == 1) return y;
    else return z;
}

Vec3& Vec3::operator+=(const Vec3 &v2) {
    x += v2.x;
    y += v2.y;
    z += v2.z;
    return *this;
}

Vec3& Vec3::operator-=(const Vec3 &v2) {
    x -= v2.x;
    y -= v2.y;
    z -= v2.z;
    return *this;
}

Vec3& Vec3::operator*=(const Vec3 &v2) {
    x *= v2.x;
    y *= v2.y;
    z *= v2.z;
    return *this;
}

Vec3& Vec3::operator/=(const Vec3 &v2) {
    x /= v2.x;
    y /= v2.y;
    z /= v2.z;
    return *this;
}

Vec3& Vec3::operator*=(float t) {
    x *= t;
    y *= t;
    z *= t;
    return *this;
}

Vec3& Vec3::operator/=(float t) {
    float inv_t = 1.0f / t;
    return operator*=(inv_t);
}

Vec3 operator+(const Vec3 &v1, const Vec3 &v2) {
    return Vec3(v1.x+v2.x, v1.y+v2.y, v1.z+v2.z);
}

Vec3 operator-(const Vec3 &v1, const Vec3 &v2) {
    return Vec3(v1.x-v2.x, v1.y-v2.y, v1.z-v2.z);
}

Vec3 operator*(const Vec3 &v1, const Vec3 &v2) {
    return Vec3(v1.x*v2.x, v1.y*v2.y, v1.z*v2.z);
}

Vec3 operator/(const Vec3 &v1, const Vec3 &v2) {
    return Vec3(v1.x/v2.x, v1.y/v2.y, v1.z/v2.z);
}

Vec3 operator*(const Vec3 &v, float t) {
    return Vec3(v.x*t, v.y*t, v.z*t);
}

Vec3 operator*(float t, const Vec3 &v) {
    return v * t;
}

Vec3 operator/(const Vec3 &v, float t) {
    float inv_t = 1.0f / t;
    return v * inv_t;
}


float dot (const Vec3 &v1, const Vec3 &v2) {
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

Vec3 cross(const Vec3 &v1, const Vec3 &v2) {
    return Vec3(v1.y*v2.z - v1.z*v2.y,
                v1.z*v2.x - v1.x*v2.z,
                v1.x*v2.y - v1.y*v2.x);
}

#endif //BVH_VEC3_HPP