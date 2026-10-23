#pragma once
#include "dynamic_terrain/core/TerrainBuilder.hh"
#include "dynamic_terrain/core/TerrainRenderSink.hh"
#include "GuiTerrain.pb.h"
#include <gz/transport/Node.hh>

namespace dynamic_terrain
{
class GuiTerrainSource
{
public:
    explicit GuiTerrainSource(const std::string &service);
    void SetSnapshot(const std::shared_ptr<const TerrainSnapshot> &snapshot);
    void ApplyTexture(const TextureUpdate &update);
private:
    struct State;
    std::shared_ptr<State> state_;
    gz::transport::Node node_;
};

preview::PageInfo guiPageInfo(const TerrainPage &page);
preview::Page encodeGuiPage(const TerrainPage &page);
TerrainPage decodeGuiPage(const preview::Page &packet);
// GUI snapshots carry geometry and imagery together. A cumulative append only
// creates new renderer pages, so existing pages also need their texture deltas.
void queueGuiSnapshot(TerrainRenderSink &renderer, const TerrainSnapshot *previous,
                      const std::shared_ptr<const TerrainSnapshot> &incoming);
constexpr int kGuiTextureSize = 512;
}
