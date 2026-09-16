#pragma once

#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/config.hh>
#include <stdexcept>

namespace dynamic_terrain
{
// Math 8 corrected LOCAL to match LOCAL2; Math 9 removed the old LOCAL2 name.
// Harmonic must continue using LOCAL2 to avoid Math 7's inverse-heading bug.
#if GZ_MATH_MAJOR_VERSION >= 9
inline constexpr auto kLocalCoordinateFrame = gz::math::SphericalCoordinates::LOCAL;
#else
inline constexpr auto kLocalCoordinateFrame = gz::math::SphericalCoordinates::LOCAL2;
#endif

// Keep the pre-Math-9 radians / metric contract at this adapter boundary.
inline gz::math::Vector3d gzPositionTransform(
    const gz::math::SphericalCoordinates &spherical,
    const gz::math::Vector3d &position,
    gz::math::SphericalCoordinates::CoordinateType input,
    gz::math::SphericalCoordinates::CoordinateType output)
{
#if GZ_MATH_MAJOR_VERSION >= 9
    const auto coordinate = input == gz::math::SphericalCoordinates::SPHERICAL
        ? gz::math::CoordinateVector3::Spherical(
            gz::math::Angle(position.X()), gz::math::Angle(position.Y()), position.Z())
        : gz::math::CoordinateVector3::Metric(position);
    const auto result = spherical.PositionTransform(coordinate, input, output);
    if (!result)
        throw std::runtime_error("Gazebo geographic coordinate conversion failed");
    if (output == gz::math::SphericalCoordinates::SPHERICAL)
        return {result->Lat()->Radian(), result->Lon()->Radian(), *result->Z()};
    return result->AsMetricVector().value();
#else
    return spherical.PositionTransform(position, input, output);
#endif
}
}
