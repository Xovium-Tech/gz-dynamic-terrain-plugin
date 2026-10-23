#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <map>
#include <stdexcept>
#include <vector>

namespace dynamic_terrain
{
// Inclusive Slippy-map tile rectangles. This header deliberately uses only the
// standard library so simulator adapters can share exactly the same partition.
struct CoverageRect
{
    int minX, minY, maxX, maxY, zoom;
};

struct CoverageTile
{
    int x, y, z;
    bool inner{false};
};

struct CoveragePlan
{
    // Coarse to fine, then north to south / west to east. The protected inner
    // rectangle is included, marked inner, and is never coarsened.
    std::vector<CoverageTile> leaves;
    // West, east, north, south: adjacent coarser leaf, or -1. A balanced
    // partition has at most one coarser neighbor along any leaf edge.
    std::vector<std::array<int, 4>> coarserNeighbors;
};

// Additional vertices are affine combinations of the original regular grid.
// Splitting the coarse triangles makes both sides use identical edge segments,
// rather than relying on rasterization of a floating-point T-junction.
struct CoverageVertexRecipe
{
    std::array<std::uint32_t, 3> indices;
    std::array<double, 3> weights;
};
struct CoverageTopology
{
    std::vector<CoverageVertexRecipe> appended;
    std::vector<std::uint32_t> indices;
};

inline double coverageBoundaryCoordinate(int point, int fineCells, int fineAxis,
                                         int fineZoom, int coarseAxis,
                                         int coarseZoom, int coarseCells)
{
    const auto scale = std::int64_t{1} << (fineZoom - coarseZoom);
    const auto offset = fineAxis - std::int64_t(coarseAxis) * scale;
    return static_cast<double>((offset * fineCells + point) * coarseCells) /
        static_cast<double>(scale * fineCells);
}

inline CoverageTopology stitchCoverageBoundary(int cells,
    const std::array<std::vector<double>, 4> &edgeCoordinates)
{
    if (cells < 1) throw std::invalid_argument("invalid coverage grid size");
    CoverageTopology result;
    const auto side = static_cast<std::uint32_t>(cells + 1);
    const auto baseVertices = side * side;
    using Edge = std::pair<std::uint32_t, std::uint32_t>;
    std::map<Edge, std::vector<std::uint32_t>> cuts;
    auto edgeIndex = [&](int edge, int along) -> std::uint32_t
    {
        if (edge == 0) return along * side;
        if (edge == 1) return along * side + cells;
        if (edge == 2) return along;
        return cells * side + along;
    };
    for (int edge = 0; edge < 4; ++edge)
    {
        auto coordinates = edgeCoordinates[edge];
        std::sort(coordinates.begin(), coordinates.end());
        coordinates.erase(std::unique(coordinates.begin(), coordinates.end()), coordinates.end());
        for (const double coordinate : coordinates)
        {
            if (!std::isfinite(coordinate) || coordinate < 0 || coordinate > cells)
                throw std::invalid_argument("coverage boundary coordinate outside grid");
            const int first = static_cast<int>(std::floor(coordinate));
            const double fraction = coordinate - first;
            if (fraction == 0) continue;
            const auto a = edgeIndex(edge, first), b = edgeIndex(edge, first + 1);
            cuts[{a, b}].push_back(baseVertices + static_cast<std::uint32_t>(result.appended.size()));
            result.appended.push_back({{a, b, a}, {1.0 - fraction, fraction, 0.0}});
        }
    }
    result.indices.reserve(static_cast<std::size_t>(cells) * cells * 6 + cuts.size() * 12);
    auto triangle = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c)
    {
        const std::array<std::uint32_t, 3> original{a, b, c};
        std::vector<std::uint32_t> perimeter;
        for (int i = 0; i < 3; ++i)
        {
            const auto from = original[i], to = original[(i + 1) % 3];
            perimeter.push_back(from);
            const auto found = cuts.find(std::minmax(from, to));
            if (found == cuts.end()) continue;
            if (from < to) perimeter.insert(perimeter.end(), found->second.begin(), found->second.end());
            else perimeter.insert(perimeter.end(), found->second.rbegin(), found->second.rend());
        }
        if (perimeter.size() == 3)
            result.indices.insert(result.indices.end(), original.begin(), original.end());
        else
        {
            const auto center = baseVertices + static_cast<std::uint32_t>(result.appended.size());
            result.appended.push_back({original, {1.0 / 3, 1.0 / 3, 1.0 / 3}});
            for (std::size_t i = 0; i < perimeter.size(); ++i)
                for (const auto index : {center, perimeter[i], perimeter[(i + 1) % perimeter.size()]})
                    result.indices.push_back(index);
        }
    };
    for (int y = 0; y < cells; ++y)
        for (int x = 0; x < cells; ++x)
        {
            const auto a = static_cast<std::uint32_t>(y * side + x), c = a + side;
            triangle(a, c, a + 1);
            triangle(a + 1, c, c + 1);
        }
    return result;
}

inline CoveragePlan planTerrainCoverage(const CoverageRect &inner,
                                        const CoverageRect &outer,
                                        std::size_t maxPages = 1024)
{
    auto valid = [](const CoverageRect &r)
    {
        return r.zoom >= 0 && r.zoom <= 25 && r.minX >= 0 && r.minY >= 0 &&
            r.maxX >= r.minX && r.maxY >= r.minY &&
            r.maxX < (1 << r.zoom) && r.maxY < (1 << r.zoom);
    };
    if (!valid(inner) || !valid(outer) || outer.zoom > inner.zoom || maxPages == 0)
        throw std::invalid_argument("invalid terrain coverage rectangles");

    using Box = std::array<std::int64_t, 4>;
    auto box = [&](const CoverageTile &tile) -> Box
    {
        const auto scale = std::int64_t{1} << (inner.zoom - tile.z);
        return {tile.x * scale, tile.y * scale,
                (tile.x + 1) * scale, (tile.y + 1) * scale};
    };
    const Box protectedBox{inner.minX, inner.minY,
                           std::int64_t(inner.maxX) + 1, std::int64_t(inner.maxY) + 1};
    const Box halo{std::max<std::int64_t>(0, protectedBox[0] - 1),
                   std::max<std::int64_t>(0, protectedBox[1] - 1),
                   std::min<std::int64_t>(1 << inner.zoom, protectedBox[2] + 1),
                   std::min<std::int64_t>(1 << inner.zoom, protectedBox[3] + 1)};
    auto intersects = [](const Box &a, const Box &b)
    {
        return a[0] < b[2] && b[0] < a[2] && a[1] < b[3] && b[1] < a[3];
    };
    const auto outerScale = std::int64_t{1} << (inner.zoom - outer.zoom);
    const Box outerBox{outer.minX * outerScale, outer.minY * outerScale,
                       (outer.maxX + 1) * outerScale, (outer.maxY + 1) * outerScale};
    if (outerBox[0] > halo[0] || outerBox[1] > halo[1] ||
        outerBox[2] < halo[2] || outerBox[3] < halo[3])
        throw std::invalid_argument("outer coverage must contain the fine halo");

    CoveragePlan plan;
    std::vector<CoverageTile> pending;
    for (int y = outer.minY; y <= outer.maxY; ++y)
        for (int x = outer.minX; x <= outer.maxX; ++x)
            pending.push_back({x, y, outer.zoom, false});
    while (!pending.empty())
    {
        auto tile = pending.back();
        pending.pop_back();
        const auto bounds = box(tile);
        if (tile.z < inner.zoom && intersects(bounds, halo))
        {
            for (int y = 0; y < 2; ++y)
                for (int x = 0; x < 2; ++x)
                    pending.push_back({tile.x * 2 + x, tile.y * 2 + y, tile.z + 1, false});
        }
        else
        {
            tile.inner = tile.z == inner.zoom && intersects(bounds, protectedBox);
            plan.leaves.push_back(tile);
        }
        if (pending.size() + plan.leaves.size() > maxPages)
            throw std::runtime_error("terrain coverage exceeds page limit");
    }

    auto adjacentSide = [](const Box &a, const Box &b)
    {
        const bool overlapY = std::max(a[1], b[1]) < std::min(a[3], b[3]);
        const bool overlapX = std::max(a[0], b[0]) < std::min(a[2], b[2]);
        if (a[0] == b[2] && overlapY) return 0;
        if (a[2] == b[0] && overlapY) return 1;
        if (a[1] == b[3] && overlapX) return 2;
        if (a[3] == b[1] && overlapX) return 3;
        return -1;
    };
    // Balance only outside the protected rectangle. Each split reduces a
    // finite zoom difference; the explicit leaf cap also bounds work/memory.
    for (;;)
    {
        std::size_t split = plan.leaves.size();
        for (std::size_t i = 0; i < plan.leaves.size() && split == plan.leaves.size(); ++i)
            for (std::size_t j = i + 1; j < plan.leaves.size(); ++j)
            {
                const auto &a = plan.leaves[i];
                const auto &b = plan.leaves[j];
                if (std::abs(a.z - b.z) <= 1 || adjacentSide(box(a), box(b)) < 0)
                    continue;
                split = a.z < b.z ? i : j;
                break;
            }
        if (split == plan.leaves.size()) break;
        if (plan.leaves.size() + 3 > maxPages)
            throw std::runtime_error("balanced terrain coverage exceeds page limit");
        const auto tile = plan.leaves[split];
        plan.leaves.erase(plan.leaves.begin() + split);
        for (int y = 0; y < 2; ++y)
            for (int x = 0; x < 2; ++x)
                plan.leaves.push_back({tile.x * 2 + x, tile.y * 2 + y, tile.z + 1, false});
    }
    std::sort(plan.leaves.begin(), plan.leaves.end(), [](const auto &a, const auto &b)
    {
        if (a.z != b.z) return a.z < b.z;
        return a.y != b.y ? a.y < b.y : a.x < b.x;
    });
    plan.coarserNeighbors.resize(plan.leaves.size(), {-1, -1, -1, -1});
    for (std::size_t i = 0; i < plan.leaves.size(); ++i)
        for (std::size_t j = 0; j < i; ++j)
        {
            if (plan.leaves[j].z >= plan.leaves[i].z) continue;
            const int side = adjacentSide(box(plan.leaves[i]), box(plan.leaves[j]));
            if (side >= 0) plan.coarserNeighbors[i][side] = static_cast<int>(j);
        }
    return plan;
}
}
