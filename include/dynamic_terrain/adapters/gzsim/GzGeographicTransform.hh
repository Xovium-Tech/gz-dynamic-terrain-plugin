#pragma once

#include "dynamic_terrain/core/TerrainTypes.hh"
#include "dynamic_terrain/adapters/gzsim/GzMathCompat.hh"

#include <gz/math/SphericalCoordinates.hh>

#include <utility>

namespace dynamic_terrain
{
class GzGeographicTransform final : public GeographicTransform
{
public:
    explicit GzGeographicTransform(gz::math::SphericalCoordinates spherical)
        : spherical_(std::move(spherical)) {}

    Vec3 LocalFromGeodetic(double latDeg, double lonDeg,
                           double elevationM) const override
    {
        const auto result = gzPositionTransform(spherical_,
            {latDeg * kPi / 180.0, lonDeg * kPi / 180.0, elevationM},
            gz::math::SphericalCoordinates::SPHERICAL,
            kLocalCoordinateFrame);
        return {result.X(), result.Y(), result.Z()};
    }

    Vec3 GeodeticFromLocal(const Vec3 &local) const override
    {
        const auto result = gzPositionTransform(spherical_,
            {local.X(), local.Y(), local.Z()},
            kLocalCoordinateFrame,
            gz::math::SphericalCoordinates::SPHERICAL);
        return {result.X() * 180.0 / kPi, result.Y() * 180.0 / kPi, result.Z()};
    }

    double LatitudeDeg() const override { return spherical_.LatitudeReference().Degree(); }
    double LongitudeDeg() const override { return spherical_.LongitudeReference().Degree(); }
    double ElevationReference() const override { return spherical_.ElevationReference(); }

private:
    gz::math::SphericalCoordinates spherical_;
};
}
