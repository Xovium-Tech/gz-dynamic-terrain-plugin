#include "dynamic_terrain/adapters/gzsim/gui/GuiTerrain.hh"
#include "dynamic_terrain/adapters/SdfConfig.hh"
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/bytes.pb.h>
#include <sdf/Root.hh>
#include <sdf/Model.hh>
#include <iostream>
#include <chrono>
#include <stdexcept>

#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while (false)

namespace
{
class RecordingRenderer final : public dynamic_terrain::TerrainRenderSink
{
public:
    void QueueSnapshot(std::shared_ptr<const dynamic_terrain::TerrainSnapshot> value) override
    { snapshots.push_back(std::move(value)); calls.push_back("snapshot"); }
    void QueueTexture(dynamic_terrain::TextureUpdate value) override
    { updates.push_back(std::move(value)); calls.push_back("texture"); }
    std::uint64_t ActiveGeneration() const override { return 0; }
    bool HasActiveTerrain() const override { return false; }
    std::optional<dynamic_terrain::TileKey> ActiveCenterTile() const override { return {}; }
    std::vector<std::shared_ptr<const dynamic_terrain::TerrainSnapshot>> snapshots;
    std::vector<dynamic_terrain::TextureUpdate> updates;
    std::vector<std::string> calls;
};

void testGuiPublication(const dynamic_terrain::TerrainPage &near)
{
    using namespace dynamic_terrain;
    auto initial = std::make_shared<TerrainSnapshot>();
    initial->generation = 42;
    initial->coverageRevision = 1;
    initial->progressiveInitial = true;
    initial->progressiveComplete = false;
    initial->pages.push_back(near);
    RecordingRenderer renderer;
    queueGuiSnapshot(renderer, nullptr, initial);
    CHECK(renderer.snapshots.size() == 1 && renderer.updates.empty());

    // A near refinement and a new outer page arrive in the same GUI poll. The
    // snapshot creates the outer page; the separate update refines the near one.
    auto expanded = std::make_shared<TerrainSnapshot>(*initial);
    expanded->coverageRevision = 2;
    expanded->pages[0].textureName = "near_refined";
    expanded->pages[0].imageryZoom = near.imageryZoom + 1;
    auto outer = near;
    ++outer.key.x;
    outer.textureName = "outer_initial";
    expanded->pages.insert(expanded->pages.begin(), outer);
    queueGuiSnapshot(renderer, initial.get(), expanded);
    CHECK(renderer.calls == std::vector<std::string>({"snapshot", "snapshot", "texture"}));
    CHECK(renderer.snapshots.back() == expanded);
    CHECK(renderer.updates.size() == 1 && renderer.updates.back().changedPageCount == 1);
    const auto &refined = renderer.updates.back().pages.front();
    CHECK(refined.pageIndex == 1 && refined.pageKey == near.key);
    CHECK(refined.textureName == "near_refined" && refined.texture == near.texture);
    CHECK(refined.imageryZoom == near.imageryZoom + 1);

    // The completion marker may carry the final refinement with no page-count
    // change. Advancing GUI metadata must not swallow this last texture delta.
    auto completed = std::make_shared<TerrainSnapshot>(*expanded);
    completed->coverageRevision = 3;
    completed->progressiveComplete = true;
    completed->pages[1].textureName = "near_final";
    ++completed->pages[1].imageryZoom;
    queueGuiSnapshot(renderer, expanded.get(), completed);
    CHECK(renderer.snapshots.size() == 3 && renderer.updates.size() == 2);
    CHECK(renderer.snapshots.back()->progressiveComplete);
    CHECK(renderer.updates.back().pages.front().textureName == "near_final");
    queueGuiSnapshot(renderer, completed.get(), completed);
    CHECK(renderer.snapshots.size() == 3 && renderer.updates.size() == 2);

    auto imageryOnly = std::make_shared<TerrainSnapshot>(*completed);
    imageryOnly->pages[0].textureName = "outer_refined";
    ++imageryOnly->pages[0].imageryZoom;
    queueGuiSnapshot(renderer, completed.get(), imageryOnly);
    CHECK(renderer.snapshots.size() == 3 && renderer.updates.size() == 3);
    CHECK(renderer.updates.back().pages.front().pageKey == outer.key);

    auto staleImagery = std::make_shared<TerrainSnapshot>(*imageryOnly);
    staleImagery->pages[1] = near;
    queueGuiSnapshot(renderer, imageryOnly.get(), staleImagery);
    CHECK(renderer.updates.size() == 3);
    auto replacement = std::make_shared<TerrainSnapshot>(*imageryOnly);
    ++replacement->generation;
    replacement->pages[1].textureName = "replacement";
    queueGuiSnapshot(renderer, imageryOnly.get(), replacement);
    CHECK(renderer.snapshots.size() == 4 && renderer.updates.size() == 3);
    std::cout << "GUI append, completion, keyed refinement and replacement publication passed\n";
}
}

int main()
{
    using namespace dynamic_terrain;
    try
    {
        CHECK(!Config{}.visualGui);
        CHECK(!Config{}.visualProgressiveUploads);
        CHECK(parseTerrainConfig(nullptr).visualProgressiveUploads);
        for (const auto enabled : {false, true})
        {
            sdf::Root root;
            const std::string xml = "<sdf version='1.9'><model name='test'>"
                "<link name='base'/><plugin name='custom::DynamicTerrainConfig' filename='test'>"
                "<visual_gui>" + std::string(enabled ? "true" : "false") +
                "</visual_gui><visual_progressive_uploads>" + std::string(enabled ? "true" : "false") +
                "</visual_progressive_uploads></plugin></model></sdf>";
            CHECK(root.LoadSdfString(xml).empty());
            CHECK(parseTerrainConfig(root.Model()->Element()->GetElement("plugin")).visualGui == enabled);
            CHECK(parseTerrainConfig(root.Model()->Element()->GetElement("plugin")).visualProgressiveUploads == enabled);
        }
        auto snapshot = std::make_shared<TerrainSnapshot>();
        snapshot->generation = 7;
        snapshot->resourcePrefix = "gui_test";
        TerrainPage page;
        page.key = {3, 4, 14}; page.index = 0;
        page.submeshName = "geographic_page";
        page.textureName = "geographic_texture";
        page.textureSize = 2048;
        page.imageryZoom = 17;
        std::vector<unsigned char> pixels(2048u * 2048u * 3u);
        for (std::size_t i = 0; i < pixels.size(); i += 3)
        { pixels[i] = 19; pixels[i+1] = 83; pixels[i+2] = 191; }
        auto texture = std::make_shared<ImageData>();
        texture->width = 2048; texture->height = 2048; texture->rgb = pixels;
        page.texture = texture;
        auto mesh = std::make_shared<MeshData>();
        mesh->name = "geographic_mesh_g7"; mesh->submeshName = page.submeshName;
        for (int i = 0; i < 3; ++i)
        {
            mesh->positions.emplace_back(10000 + i * 10, 30000 + i * 20, 123.5 + i * 70);
            mesh->normals.emplace_back(0, 0, 1); mesh->texCoords.emplace_back(i / 2.0, 1 - i / 2.0);
            mesh->indices.push_back(i);
        }
        page.mesh = mesh;
        snapshot->pages.push_back(page);
        testGuiPublication(page);
        const auto packet = encodeGuiPage(page);
        const auto decoded = decodeGuiPage(packet);
        CHECK(decoded.textureSize == 512);
        CHECK(page.texture->width == 2048);
        CHECK(decoded.key == page.key);
        CHECK(decoded.textureName == page.textureName + "_gui512");
        const auto rgb = decoded.texture->rgb;
        CHECK(rgb.size() == 512u * 512u * 3u);
        for (std::size_t i = 0; i < rgb.size(); i += 3)
            CHECK(rgb[i] == 19 && rgb[i+1] == 83 && rgb[i+2] == 191);
        const auto sourceMesh = page.mesh;
        const auto resultMesh = decoded.mesh;
        for (unsigned i = 0; i < 3; ++i)
        {
            CHECK((sourceMesh->positions[i] - resultMesh->positions[i]).Length() < 1e-12);
            CHECK(sourceMesh->texCoords[i].X() == resultMesh->texCoords[i].X());
            CHECK(sourceMesh->texCoords[i].Y() == resultMesh->texCoords[i].Y());
            CHECK(sourceMesh->indices[i] == resultMesh->indices[i]);
        }
        auto bad = packet;
        bad.set_indices(0, 99);
        bool rejected = false;
        try { decodeGuiPage(bad); } catch (const std::exception &) { rejected = true; }
        CHECK(rejected);
        bad = packet; bad.mutable_info()->set_texture_size(8192);
        rejected = false;
        try { decodeGuiPage(bad); } catch (const std::exception &) { rejected = true; }
        CHECK(rejected);

        const std::string service = "/terrain_gui_test_" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count());
        GuiTerrainSource source(service);
        gz::transport::Node client;
        auto request = [&](const std::string &query, gz::msgs::Bytes &reply) {
            gz::msgs::StringMsg input; input.set_data(query);
            bool ok = false;
            CHECK(client.Request(service, input, 2000u, reply, ok));
            return ok;
        };
        source.SetSnapshot(snapshot);
        gz::msgs::Bytes reply;
        CHECK(request("", reply));
        preview::Manifest manifest;
        CHECK(manifest.ParseFromString(reply.data()));
        CHECK(manifest.version() == 1 && manifest.pages_size() == 1);
        CHECK(manifest.revision() == 1);
        CHECK(manifest.generation() == 7 && manifest.coverage_revision() == 0);
        CHECK(request("1 0", reply));
        preview::Page fromWire;
        CHECK(fromWire.ParseFromString(reply.data()));
        CHECK(fromWire.SerializeAsString() == packet.SerializeAsString());
        CHECK(!request("0 0", reply));
        CHECK(!request("1 99", reply));
        CHECK(!request("1 0 garbage", reply));
        TextureUpdate update;
        update.generation = 7;
        TexturePageUpdate item;
        item.pageKey = page.key; item.pageIndex = 0; item.submeshName = page.submeshName;
        item.texture = page.texture; item.textureName = "refined";
        item.textureSize = 2048; item.imageryZoom = 18;
        update.pages.push_back(item);
        source.ApplyTexture(update);
        CHECK(!request("1 0", reply));
        CHECK(request("", reply));
        CHECK(manifest.ParseFromString(reply.data()));
        CHECK(manifest.revision() == 2);
        CHECK(manifest.pages(0).texture_name() == "refined_gui512");
        update.generation = 6;
        source.ApplyTexture(update);
        CHECK(request("", reply));
        CHECK(manifest.ParseFromString(reply.data()));
        CHECK(manifest.revision() == 2);
        auto expanded = std::make_shared<TerrainSnapshot>(*snapshot);
        expanded->coverageRevision = 1;
        expanded->progressiveInitial = true;
        expanded->progressiveComplete = false;
        auto outer = page;
        outer.visualOnly = true;
        outer.index = 1;
        outer.key.x += 1;
        expanded->pages.push_back(outer);
        source.SetSnapshot(expanded);
        CHECK(request("", reply));
        CHECK(manifest.ParseFromString(reply.data()));
        CHECK(manifest.generation() == 7 && manifest.coverage_revision() == 1);
        CHECK(manifest.progressive_initial() && !manifest.progressive_complete());
        CHECK(manifest.pages_size() == 2 && manifest.pages(1).visual_only());
        CHECK(manifest.pages(0).texture_name() == "refined_gui512");
        const auto expandedRevision = manifest.revision();
        source.SetSnapshot(snapshot);
        CHECK(request("", reply));
        CHECK(manifest.ParseFromString(reply.data()));
        CHECK(manifest.revision() == expandedRevision);
        auto completed = std::make_shared<TerrainSnapshot>(*expanded);
        completed->coverageRevision = 2;
        completed->progressiveComplete = true;
        source.SetSnapshot(completed);
        CHECK(request("", reply));
        CHECK(manifest.ParseFromString(reply.data()));
        CHECK(manifest.progressive_initial() && manifest.progressive_complete());
        CHECK(manifest.coverage_revision() == 2);
        auto recentered = std::make_shared<TerrainSnapshot>(*snapshot);
        recentered->generation = 8;
        source.SetSnapshot(recentered);
        CHECK(request("", reply));
        CHECK(manifest.ParseFromString(reply.data()));
        CHECK(manifest.generation() == 8);
        CHECK(manifest.pages(0).texture_name() == "refined_gui512");
        CHECK(manifest.pages(0).imagery_zoom() == 18);
        // Reusing tile coordinates in another dataset must not reuse its pixels.
        recentered->resourcePrefix = "another_dataset";
        source.SetSnapshot(recentered);
        CHECK(request("", reply));
        CHECK(manifest.ParseFromString(reply.data()));
        CHECK(manifest.pages(0).texture_name() == "geographic_texture_gui512");
        std::cout << "GUI config, image/mesh codec and on-demand service tests passed\n";
    }
    catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
