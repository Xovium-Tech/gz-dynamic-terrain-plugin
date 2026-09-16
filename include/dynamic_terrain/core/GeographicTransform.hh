#pragma once

#include <cmath>

namespace dynamic_terrain
{
// Terrain coordinates deliberately have no dependency on a simulator math ABI.
class Vec2
{
public:
    constexpr Vec2(double x = 0.0, double y = 0.0) : x_(x), y_(y) {}
    constexpr double X() const { return x_; }
    constexpr double Y() const { return y_; }
    void X(double value) { x_ = value; }
    void Y(double value) { y_ = value; }
    void Set(double x, double y) { x_ = x; y_ = y; }

private:
    double x_, y_;
};

class Vec3
{
public:
    constexpr Vec3(double x = 0.0, double y = 0.0, double z = 0.0)
        : x_(x), y_(y), z_(z) {}
    constexpr double X() const { return x_; }
    constexpr double Y() const { return y_; }
    constexpr double Z() const { return z_; }
    void X(double value) { x_ = value; }
    void Y(double value) { y_ = value; }
    void Z(double value) { z_ = value; }
    void Set(double x, double y, double z) { x_ = x; y_ = y; z_ = z; }

    Vec3 operator+(const Vec3 &v) const { return {x_ + v.x_, y_ + v.y_, z_ + v.z_}; }
    Vec3 operator-(const Vec3 &v) const { return {x_ - v.x_, y_ - v.y_, z_ - v.z_}; }
    Vec3 operator-() const { return {-x_, -y_, -z_}; }
    Vec3 operator*(double s) const { return {x_ * s, y_ * s, z_ * s}; }
    Vec3 operator/(double s) const { return {x_ / s, y_ / s, z_ / s}; }
    Vec3 &operator+=(const Vec3 &v) { return *this = *this + v; }
    Vec3 &operator-=(const Vec3 &v) { return *this = *this - v; }
    Vec3 &operator*=(double s) { return *this = *this * s; }
    Vec3 &operator/=(double s) { return *this = *this / s; }
    double Dot(const Vec3 &v) const { return x_ * v.x_ + y_ * v.y_ + z_ * v.z_; }
    Vec3 Cross(const Vec3 &v) const
    {
        return {y_ * v.z_ - z_ * v.y_, z_ * v.x_ - x_ * v.z_, x_ * v.y_ - y_ * v.x_};
    }
    double SquaredLength() const { return Dot(*this); }
    double Length() const { return std::sqrt(SquaredLength()); }
    double Distance(const Vec3 &v) const { return (*this - v).Length(); }
    Vec3 &Normalize()
    {
        const double length = Length();
        if (length > 0.0)
            *this /= length;
        return *this;
    }
    Vec3 Normalized() const { Vec3 result(*this); return result.Normalize(); }

    static const Vec3 Zero;
    static const Vec3 UnitX;
    static const Vec3 UnitY;
    static const Vec3 UnitZ;

private:
    double x_, y_, z_;
};

inline Vec3 operator*(double scalar, const Vec3 &v) { return v * scalar; }
inline const Vec3 Vec3::Zero{0.0, 0.0, 0.0};
inline const Vec3 Vec3::UnitX{1.0, 0.0, 0.0};
inline const Vec3 Vec3::UnitY{0.0, 1.0, 0.0};
inline const Vec3 Vec3::UnitZ{0.0, 0.0, 1.0};

// Implemented by each adapter using its native spherical-coordinate transform.
// Geodetic vectors carry latitude / longitude in degrees and elevation in metres.
class GeographicTransform
{
public:
    virtual ~GeographicTransform() = default;
    virtual Vec3 LocalFromGeodetic(double latitudeDeg, double longitudeDeg,
                                   double elevationM) const = 0;
    virtual Vec3 GeodeticFromLocal(const Vec3 &local) const = 0;
    virtual double LatitudeDeg() const = 0;
    virtual double LongitudeDeg() const = 0;
    virtual double ElevationReference() const = 0;
};
}
