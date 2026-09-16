#include "dynamic_terrain/adapters/gzsim/GzGeographicTransform.hh"

#include <gz/math/Angle.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Vector3.hh>

#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
class TestFailure : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

void check(bool condition, const char *expression, const char *file, int line)
{
    if (condition)
        return;
    std::ostringstream out;
    out << file << ':' << line << ": CHECK failed: " << expression;
    throw TestFailure(out.str());
}

void checkNear(double actual, double expected, double tolerance,
               const char *expression, const char *file, int line)
{
    if (std::isfinite(actual) && std::isfinite(expected) &&
        std::abs(actual - expected) <= tolerance)
        return;
    std::ostringstream out;
    out << std::setprecision(17) << file << ':' << line
        << ": CHECK_NEAR failed: " << expression << " (actual=" << actual
        << ", expected=" << expected << ", tolerance=" << tolerance << ')';
    throw TestFailure(out.str());
}

#define CHECK(expression) \
    check(static_cast<bool>(expression), #expression, __FILE__, __LINE__)
#define CHECK_NEAR(actual, expected, tolerance) \
    checkNear((actual), (expected), (tolerance), \
              #actual " ~= " #expected, __FILE__, __LINE__)

gz::math::Angle degrees(double value)
{
    gz::math::Angle result;
    result.SetDegree(value);
    return result;
}

void testLocal2RoundTrip()
{
    struct Reference
    {
        double latitudeDeg;
        double longitudeDeg;
        double elevationM;
    };

    const std::array<Reference, 2> references{{
        {52.2297, 21.0122, 123.4},
        {78.2232, 15.6469, 37.0},
    }};
    const std::array<double, 4> headingsDeg{{0.0, 37.0, 90.0, -73.0}};
    const std::array<gz::math::Vector3d, 7> localPoints{{
        {0.0, 0.0, 0.0},
        {1000.0, 0.0, 50.0},
        {0.0, 10000.0, -20.0},
        {60000.0, 80000.0, 250.0},
        {100000.0, 0.0, 1000.0},
        {-60000.0, 80000.0, -100.0},
        {-100000.0, 0.0, 5000.0},
    }};

    for (const auto &reference : references)
    {
        for (const double headingDeg : headingsDeg)
        {
            const gz::math::SphericalCoordinates spherical(
                gz::math::SphericalCoordinates::EARTH_WGS84,
                degrees(reference.latitudeDeg),
                degrees(reference.longitudeDeg),
                reference.elevationM, degrees(headingDeg));

            const dynamic_terrain::GzGeographicTransform transform(spherical);
            for (const auto &local : localPoints)
            {
                const auto expectedRadians = dynamic_terrain::gzPositionTransform(spherical,
                    local,
                    dynamic_terrain::kLocalCoordinateFrame,
                    gz::math::SphericalCoordinates::SPHERICAL);
                const auto geodetic = dynamic_terrain::geodeticFromLocal(
                    transform, {local.X(), local.Y(), local.Z()});

                CHECK_NEAR(geodetic.X(),
                           expectedRadians.X() * 180.0 / dynamic_terrain::kPi,
                           1e-10);
                CHECK_NEAR(geodetic.Y(),
                           expectedRadians.Y() * 180.0 / dynamic_terrain::kPi,
                           1e-10);
                CHECK_NEAR(geodetic.Z(), expectedRadians.Z(), 1e-6);

                const auto recovered = dynamic_terrain::localFromGeodetic(
                    transform, geodetic.X(), geodetic.Y(), geodetic.Z());
                CHECK(recovered.Distance({local.X(), local.Y(), local.Z()}) < 1e-3);

                const gz::math::Vector3d geodeticRadians{
                    geodetic.X() * dynamic_terrain::kPi / 180.0,
                    geodetic.Y() * dynamic_terrain::kPi / 180.0,
                    geodetic.Z()};
                const auto expectedLocal = dynamic_terrain::gzPositionTransform(spherical,
                    geodeticRadians,
                    gz::math::SphericalCoordinates::SPHERICAL,
                    dynamic_terrain::kLocalCoordinateFrame);
                CHECK(recovered.Distance({expectedLocal.X(), expectedLocal.Y(), expectedLocal.Z()}) < 1e-7);
            }
        }
    }
}

}

int main()
{
    try { testLocal2RoundTrip(); return 0; }
    catch (const std::exception &error) { std::cerr << error.what() << "\n"; return 1; }
}
