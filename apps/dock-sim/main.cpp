#include <docksim/app.hpp>
#include <docksim/config.hpp>
#include <docksim/render.hpp>
#include <docksim/version.hpp>

#include <CLI/CLI.hpp>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <optional>
#include <print>
#include <string>
#include <tuple>

namespace {

struct Arguments {
    std::optional<std::filesystem::path> config;
    bool login = false;
    std::optional<std::string> dock_sn;
    std::optional<std::string> drone_sn;
    std::optional<std::string> log_level;
    bool log_wire = false;
};

std::optional<docksim::LogLevel> parse_level(std::string text) {
    std::ranges::transform(text, text.begin(),
                           [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    if (text == "DEBUG") return docksim::LogLevel::debug;
    if (text == "INFO") return docksim::LogLevel::info;
    if (text == "WARNING") return docksim::LogLevel::warning;
    if (text == "ERROR") return docksim::LogLevel::error;
    return std::nullopt;
}

int run(const Arguments& args) {
    docksim::Config cfg;
    try {
        auto loaded = docksim::load_config(args.config, docksim::process_environment());
        cfg = std::move(loaded.config);
        docksim::set_log_level(cfg.log_level, args.log_wire);
        for (const auto& key : loaded.unknown_keys) {
            std::println(stderr, "dock-sim: config key {} is unknown and has no effect", key);
        }
        if (args.dock_sn) cfg.device.dock_sn = *args.dock_sn;
        if (args.drone_sn) cfg.device.drone_sn = *args.drone_sn;
        if (args.log_level) {
            const auto level = parse_level(*args.log_level);
            if (!level)
                throw docksim::ConfigError("--log-level must be DEBUG, INFO, WARNING or ERROR");
            cfg.log_level = *level;
        }
        cfg.validate();
    } catch (const docksim::ConfigError& e) {
        std::println(stderr, "dock-sim: {}", e.what());
        return 2;
    }
    docksim::set_log_level(cfg.log_level, args.log_wire);

    if (args.login) {
        try {
            docksim::apply(cfg, docksim::provision(cfg.http));
        } catch (const std::exception& e) {
            std::println(stderr, "dock-sim: login failed: {}", e.what());
            return 1;
        }
    }
    if (!cfg.broker.username || cfg.broker.username->empty()) {
        std::println(stderr, "dock-sim: no MQTT username. Set broker.username or use --login.");
        return 2;
    }
    return docksim::run_dock(cfg, {.log_wire = args.log_wire});
}

}  // namespace

// The media section of the config file (or the defaults), with a few overrides.
namespace {

docksim::MediaConfig media_from(const std::optional<std::filesystem::path>& config) {
    return docksim::load_config(config, docksim::process_environment()).config.media;
}

}  // namespace

namespace {

int run_cli(int argc, char** argv) {
    CLI::App app{"DJI Dock 2 emulator that speaks the DJI Cloud API to a cloud backend",
                 "dock-sim"};
    app.set_version_flag("--version", std::string{docksim::version()});
    app.require_subcommand(0, 1);

    Arguments args;
    app.add_option("-c,--config", args.config, "path to the YAML configuration file")
        ->check(CLI::ExistingFile);
    app.add_flag("--login", args.login,
                 "get the MQTT credentials and the binding code from the backend first");
    app.add_option("--dock-sn", args.dock_sn, "override the dock serial number");
    app.add_option("--drone-sn", args.drone_sn, "override the drone serial number");
    app.add_option("--log-level", args.log_level, "DEBUG, INFO, WARNING or ERROR");
    app.add_flag("--log-wire", args.log_wire, "log every MQTT message in full (very verbose)");

    docksim::PhotoRequest photo;
    std::string size = "2000x1500";
    std::filesystem::path output;
    std::optional<std::string> device;
    std::optional<std::string> cache_dir;
    auto* render = app.add_subcommand("render", "render one simulated drone photo");
    render->add_option("--lat", photo.latitude)->required();
    render->add_option("--lon", photo.longitude)->required();
    render->add_option("--alt", photo.altitude, "metres above the ground at lat/lon");
    render->add_option("--heading", photo.heading, "degrees clockwise from north");
    render->add_option("--pitch", photo.pitch, "gimbal pitch, -90 is nadir");
    render->add_option("--time", photo.time, "ISO 8601, default now");
    render->add_option("--size", size, "WIDTHxHEIGHT");
    render->add_option("--dfov", photo.dfov, "diagonal field of view, degrees");
    render->add_option("--cache-dir", cache_dir, "tile and building cache");
    render->add_option("--device", device, "cpu, auto or rt");
    render->add_option("-o,--output", output, "JPEG or PNG file")->required();

    int frames = 20;
    auto* bench = app.add_subcommand("bench", "measure the render frame rate at 720p and 1080p");
    bench->add_option("--frames", frames, "frames per size");
    bench->add_option("--device", device, "cpu, auto or rt");

    CLI11_PARSE(app, argc, argv);

    if (*render || *bench) {
        try {
            auto media = media_from(args.config);
            if (cache_dir) media.cache_dir = *cache_dir;
            if (device) {
                const auto loaded =
                    docksim::load_config(std::nullopt, {{"DOCKSIM_MEDIA__RENDER_DEVICE", *device}});
                media.render_device = loaded.config.media.render_device;
            }
            docksim::set_log_level(docksim::LogLevel::info, false);
            if (*render) {
                const auto parsed = docksim::parse_size(size);
                if (!parsed) throw docksim::ConfigError("--size must be WIDTHxHEIGHT");
                std::tie(photo.width, photo.height) = *parsed;
                docksim::render_photo(media, photo, output);
            } else {
                for (const auto& r : docksim::bench(media, frames)) {
                    std::println("{} {}x{}: mean {:.1f} ms ({:.1f} fps), max {:.1f} ms", r.device,
                                 r.width, r.height, r.mean_ms, 1000.0 / r.mean_ms, r.max_ms);
                }
            }
        } catch (const std::exception& e) {
            std::println(stderr, "dock-sim: {}", e.what());
            return 1;
        }
        return 0;
    }
    return run(args);
}

}  // namespace

int main(int argc, char** argv) {
    // Errors inside the commands are reported there; this catches what escapes the CLI set-up.
    try {
        return run_cli(argc, argv);
    } catch (const std::exception& e) {
        std::println(stderr, "dock-sim: {}", e.what());
    } catch (...) {
        std::println(stderr, "dock-sim: unknown error");
    }
    return 1;
}
