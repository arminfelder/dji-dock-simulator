#include <docksim/config.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <random>
#include <tuple>

using namespace docksim;
using Catch::Matchers::ContainsSubstring;

namespace {

class TempFile {
public:
    explicit TempFile(std::string_view text) {
        std::random_device rd;
        path_ = std::filesystem::temp_directory_path() / std::format("docksim-{}.yaml", rd());
        std::ofstream(path_) << text;
    }
    TempFile(const TempFile&) = delete;
    TempFile& operator=(const TempFile&) = delete;
    TempFile(TempFile&&) = delete;
    TempFile& operator=(TempFile&&) = delete;
    ~TempFile() { std::filesystem::remove(path_); }
    [[nodiscard]] const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

std::string load_error(std::string_view text) {
    const TempFile file(text);
    try {
        std::ignore = load_config(file.path(), {});
    } catch (const ConfigError& e) {
        return e.what();
    }
    return {};
}

}  // namespace

TEST_CASE("defaults without a file") {
    CHECK(load_config(std::nullopt, {}).config == Config{});
}

TEST_CASE("the example file has only known keys") {
    const auto result =
        load_config(std::filesystem::path(DOCKSIM_SOURCE_DIR) / "config.example.yaml", {});
    CHECK(result.unknown_keys.empty());
    CHECK(result.config.media.image_size == std::pair{2000, 1500});
    CHECK(result.config.device.thing_version == "1.1.0");
}

TEST_CASE("unknown keys are reported") {
    const TempFile file("media:\n  renderr: synthetic\nfoo: 1\n");
    auto unknown = load_config(file.path(), {}).unknown_keys;
    std::ranges::sort(unknown);
    CHECK(unknown == std::vector<std::string>{"foo", "media.renderr"});
}

TEST_CASE("bad values are named") {
    const auto message = load_error("broker:\n  port: 70000\nlive:\n  size: big\n");
    CHECK_THAT(message, ContainsSubstring("broker.port"));
    CHECK_THAT(message, ContainsSubstring("live.size"));
}

TEST_CASE("a null section is an error") {
    CHECK_THAT(load_error("media: null\n"), ContainsSubstring("media"));
}

TEST_CASE("numeric ids become text") {
    const TempFile file("organization:\n  organization_id: 100\n");
    CHECK(load_config(file.path(), {}).config.organization.organization_id == "100");
}

TEST_CASE("env aliases and nested names") {
    const auto cfg = load_config(std::nullopt, {{"DOCKSIM_BROKER_PORT", "1883"},
                                                {"DOCKSIM_MEDIA__RENDERER", "synthetic"},
                                                {"DOCKSIM_FLIGHT__TICK", "0.1"},
                                                {"DOCKSIM_LOG_LEVEL", "debug"},
                                                {"DOCKSIM_DOCK_SN", ""}})
                         .config;
    CHECK(cfg.broker.port == 1883);
    CHECK(cfg.media.renderer == PhotoRenderer::synthetic);
    CHECK(cfg.flight.tick == 0.1);
    CHECK(cfg.log_level == LogLevel::debug);
    CHECK(cfg.device.dock_sn == "SIMDOCK0000001");
}

TEST_CASE("validate catches a value set in code") {
    Config cfg;
    cfg.live.size = "1280";
    CHECK_THROWS_AS(cfg.validate(), ConfigError);
}

TEST_CASE("license_key must be an RSA public key in PEM form") {
    CHECK_FALSE(Config{}.device.license_key);
    CHECK_THAT(load_error("device:\n  license_key: not a key\n"),
               ContainsSubstring("device.license_key") &&
                   ContainsSubstring("must be an RSA public key in PEM form"));
}

TEST_CASE("a missing file is reported") {
    CHECK_THROWS_WITH(load_config("/nonexistent/nope.yaml", {}), ContainsSubstring("cannot read"));
}

TEST_CASE("bitrates parse") {
    const auto [text, bps] = GENERATE(table<std::string, std::int64_t>(
        {{"6M", 6'000'000}, {"800k", 800'000}, {"2000000", 2'000'000}, {"1.5M", 1'500'000}}));
    CHECK(parse_bitrate(text) == bps);
}

TEST_CASE("frame sizes parse") {
    CHECK(parse_size("1280x720") == std::pair{1280, 720});
    const auto bad = GENERATE(as<std::string>{}, "1280", "x720", "1280x", "0x720", "-5x720",
                              "+5x720", "12a0x720", "1280x720x1", "99999999999x1", "");
    CAPTURE(bad);
    CHECK_FALSE(parse_size(bad));
}

TEST_CASE("frame() refuses a size that validate() never saw") {
    LiveConfig live;
    live.size = "1280";
    CHECK_THROWS_AS(live.frame(), ConfigError);
}

TEST_CASE("a bad bitrate is rejected") {
    CHECK_THAT(load_error("live:\n  bitrate: fast\n"), ContainsSubstring("live.bitrate"));
}

TEST_CASE("old names still load") {
    const TempFile file(
        "media:\n  render_device: cuda\n  rt_build_processes: true\nlive:\n  encoder: h264_nvenc\n"
        "  ffmpeg: ffmpeg\n");
    const auto result = load_config(file.path(), {});
    CHECK(result.unknown_keys.empty());
    CHECK(result.config.media.render_device == RenderDevice::gpu);
    CHECK(result.config.live.encoder == VideoEncoder::nvenc);
}
