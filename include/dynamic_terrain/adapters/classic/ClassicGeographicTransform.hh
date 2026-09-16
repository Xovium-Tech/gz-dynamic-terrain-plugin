#pragma once

#include "dynamic_terrain/core/TerrainTypes.hh"

#include <gazebo/common/SphericalCoordinates.hh>

#include <memory>

namespace dynamic_terrain
{
class ClassicGeographicTransform final : public GeographicTransform
{
public:
    explicit ClassicGeographicTransform(const gazebo::common::SphericalCoordinates &reference)
        : spherical_(std::make_unique<gazebo::common::SphericalCoordinates>(
              reference.GetSurfaceType(), reference.LatitudeReference(),
              reference.LongitudeReference(), reference.GetElevationReference(),
              reference.HeadingOffset())),
          cosine_(std::cos(-reference.HeadingOffset().Radian())),
          sine_(std::sin(-reference.HeadingOffset().Radian())) {}

    Vec3 LocalFromGeodetic(double latitude, double longitude, double elevation) const override
    {
        using Coordinates = gazebo::common::SphericalCoordinates;
        const auto enu = spherical_->PositionTransform(
            {latitude * kPi / 180.0, longitude * kPi / 180.0, elevation},
            Coordinates::SPHERICAL, Coordinates::GLOBAL);
        return {enu.X() * cosine_ - enu.Y() * sine_,
                enu.X() * sine_ + enu.Y() * cosine_, enu.Z()};
    }

    Vec3 GeodeticFromLocal(const Vec3 &local) const override
    {
        using Coordinates = gazebo::common::SphericalCoordinates;
        // Classic's legacy LOCAL inverse has a sign error. Convert through ENU
        // with the same inverse heading rotation used by modern Math's LOCAL2.
        const auto spherical = spherical_->PositionTransform(
            {local.X() * cosine_ + local.Y() * sine_,
             -local.X() * sine_ + local.Y() * cosine_, local.Z()},
            Coordinates::GLOBAL, Coordinates::SPHERICAL);
        return {spherical.X() * 180.0 / kPi, spherical.Y() * 180.0 / kPi, spherical.Z()};
    }

    double LatitudeDeg() const override { return spherical_->LatitudeReference().Degree(); }
    double LongitudeDeg() const override { return spherical_->LongitudeReference().Degree(); }
    double ElevationReference() const override { return spherical_->GetElevationReference(); }

private:
    std::unique_ptr<gazebo::common::SphericalCoordinates> spherical_;
    double cosine_, sine_;
};
}
