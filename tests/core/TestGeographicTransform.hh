#pragma once

#include "dynamic_terrain/core/TerrainTypes.hh"

// Deliberately simple, invertible test projection. Production adapters retain
// their simulator's geodesy; this fixture exercises core algorithms without it.
class TestGeographicTransform : public dynamic_terrain::GeographicTransform
{
public:
    TestGeographicTransform(double latitude = 52.2297, double longitude = 21.0122,
                            double elevation = 123.4, double heading = 37.0)
        : latitude_(latitude), longitude_(longitude), elevation_(elevation),
          heading_(heading * dynamic_terrain::kPi / 180.0) {}

    dynamic_terrain::Vec3 LocalFromGeodetic(double latitude, double longitude,
                                           double elevation) const override
    {
        const double east = (longitude - longitude_) * longitudeScale();
        const double north = (latitude - latitude_) * metresPerDegree;
        return {east * std::cos(heading_) + north * std::sin(heading_),
                -east * std::sin(heading_) + north * std::cos(heading_),
                elevation - elevation_};
    }
    dynamic_terrain::Vec3 GeodeticFromLocal(const dynamic_terrain::Vec3 &local) const override
    {
        const double east = local.X() * std::cos(heading_) - local.Y() * std::sin(heading_);
        const double north = local.X() * std::sin(heading_) + local.Y() * std::cos(heading_);
        return {latitude_ + north / metresPerDegree,
                longitude_ + east / longitudeScale(), elevation_ + local.Z()};
    }
    double LatitudeDeg() const override { return latitude_; }
    double LongitudeDeg() const override { return longitude_; }
    double ElevationReference() const override { return elevation_; }

private:
    double longitudeScale() const
    {
        return metresPerDegree * std::cos(latitude_ * dynamic_terrain::kPi / 180.0);
    }
    static constexpr double metresPerDegree = 111319.49079327358;
    double latitude_, longitude_, elevation_, heading_;
};
