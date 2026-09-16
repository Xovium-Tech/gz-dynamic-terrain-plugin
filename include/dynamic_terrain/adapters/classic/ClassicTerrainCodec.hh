#pragma once

#include "dynamic_terrain/core/TerrainData.hh"
#include "ClassicTerrain.pb.h"

#include <memory>
#include <optional>
#include <string>

namespace dynamic_terrain
{
constexpr std::size_t kClassicPageMessageLimit = 51u * 1024u * 1024u;
constexpr std::size_t kClassicManifestMessageLimit = 4u * 1024u * 1024u;
classic_msgs::Snapshot EncodeClassicSnapshot(const TerrainSnapshot &, const Config &, bool includePages = true);
classic_msgs::Page EncodeClassicPage(const TerrainPage &, bool includeMesh);
std::optional<TerrainPage> DecodeClassicPage(const classic_msgs::Page &, std::string &error);
classic_msgs::TextureUpdate EncodeClassicTexture(const TextureUpdate &);
std::shared_ptr<const TerrainSnapshot> DecodeClassicSnapshot(
    const classic_msgs::Snapshot &, Config &, std::string &error, bool allowEmpty = false);
std::optional<TextureUpdate> DecodeClassicTexture(
    const classic_msgs::TextureUpdate &, std::string &error);
}
