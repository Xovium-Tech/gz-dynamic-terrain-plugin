#include "dynamic_terrain/adapters/classic/ClassicGeographicTransform.hh"

#include <array>
#include <iostream>
#include <stdexcept>

namespace
{
ignition::math::Angle degrees(double value)
{
    ignition::math::Angle angle;
    angle.SetDegree(value);
    return angle;
}
}

int main()
{
    try
    {
        using namespace dynamic_terrain;
        using Spherical = gazebo::common::SphericalCoordinates;
        const Spherical goldenReference(Spherical::EARTH_WGS84, degrees(52.2297),
            degrees(21.0122), 123.4, degrees(37.0));
        const ClassicGeographicTransform golden(goldenReference);
        // Recorded from the modern adapter's corrected LOCAL frame. A wrong
        // heading convention could round-trip while still failing these values.
        const auto goldenLocal = golden.LocalFromGeodetic(52.23, 21.014, 150.0);
        if (goldenLocal.Distance({118.31408453594509, -47.354976912986743,
                                  26.598729318023192}) >= 1e-6)
            throw std::runtime_error("Classic does not match modern geographic position");
        const auto goldenGeodetic = golden.GeodeticFromLocal({60000, 80000, 250});
        if (std::abs(goldenGeodetic.X() - 53.12820373197799) >= 1e-9 ||
            std::abs(goldenGeodetic.Y() - 21.008808361469018) >= 1e-9 ||
            std::abs(goldenGeodetic.Z() - 1157.5304708508775) >= 1e-5)
            throw std::runtime_error("Classic does not match modern geographic inverse");
        const std::array<Vec3, 5> points{{
            {0, 0, 0}, {1000, 0, 50}, {0, 10000, -20},
            {60000, 80000, 250}, {-100000, 0, 5000}}};
        for (double latitude : {52.2297, 78.2232})
        {
            for (double heading : {0.0, 37.0, 90.0, -73.0})
            {
                const Spherical native(Spherical::EARTH_WGS84, degrees(latitude),
                    degrees(21.0122), 123.4, degrees(heading));
                const ClassicGeographicTransform transform(native);
                for (const auto &local : points)
                {
                    const auto geodetic = transform.GeodeticFromLocal(local);
                    const auto recovered = transform.LocalFromGeodetic(
                        geodetic.X(), geodetic.Y(), geodetic.Z());
                    if (recovered.Distance(local) >= 1e-3)
                        throw std::runtime_error("Classic geographic round trip failed");
                    // Classic's forward LOCAL transform is correct; only its
                    // inverse needs the adapter's explicit ENU heading rotation.
                    const auto expected = native.PositionTransform(
                        {geodetic.X() * kPi / 180.0, geodetic.Y() * kPi / 180.0,
                         geodetic.Z()}, Spherical::SPHERICAL, Spherical::LOCAL);
                    if (recovered.Distance({expected.X(), expected.Y(), expected.Z()}) >= 1e-7)
                        throw std::runtime_error("Classic forward numeric behavior changed");
                }
            }
        }
        std::cout << "Classic geographic tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
