#pragma once

#include "util/json.hpp"

#include <docksim/config.hpp>

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// Facade style per building from its OSM tags: colour, window pattern,
// weathering. Order of evidence: building:colour / colour, then
// building:material, then the building type, then a stable plaster tone from
// the OSM id.
namespace docksim::osm {

using Colour = std::array<int, 3>;

// Window pattern of a facade. The render kernels use the same numbers.
enum class Windows : std::uint8_t {
    none = 0,
    punched = 1,
    ribbon = 2,
    curtain = 3,
    industrial = 4,
    historic = 5,
    // 19th century European city block: tall windows, stucco trim, cornices.
    classic = 6,
};
inline constexpr int window_kinds = 7;

// Photo wall materials. The renderer holds one texture per key, in this order.
inline constexpr std::array<std::string_view, 8> material_keys{
    "plaster_old", "plaster", "concrete", "brick", "stone", "metal", "wood", "roof_tiles",
};

// Index into material_keys, or -1 for no texture (glass, empty name).
[[nodiscard]] int material_index(std::string_view material);

struct Facade {
    Colour colour{208, 198, 178};
    Windows windows = Windows::punched;
    // 0 clean .. 1 heavy rain streaks and stains.
    double weathering = 0.0;
    // A material_keys entry, or "" for no texture (glass).
    std::string material = "plaster";

    friend bool operator==(const Facade&, const Facade&) = default;
};

// Plaster tones for buildings without any colour evidence.
inline constexpr std::array<Colour, 6> plaster_tones{{
    {208, 198, 178},
    {228, 224, 214},
    {200, 174, 140},
    {186, 156, 146},
    {172, 172, 167},
    {214, 194, 152},
}};
// Plaster tones of 19th century Vienna: Schoenbrunn yellow, ochre, cream, grey, salmon.
inline constexpr std::array<Colour, 6> classic_plaster_tones{{
    {224, 196, 128},
    {214, 188, 150},
    {232, 222, 196},
    {206, 202, 192},
    {222, 184, 156},
    {200, 202, 182},
}};
// Storey height of classic blocks, metres. Footprints use it for building:levels.
inline constexpr double classic_level_m = 3.8;
// Before this year, a residential building gets classic facades.
inline constexpr int classic_until = 1919;
// From this many levels, or with a flat roof, an undated block counts as modern.
inline constexpr double modern_levels = 8;

// The colour and weathering that building:material gives.
[[nodiscard]] std::optional<std::pair<Colour, double>> material_look(std::string_view material);

// OSM colour value: #rgb, #rrggbb or a common English name.
[[nodiscard]] std::optional<Colour> parse_colour(std::string_view text);

// A stable tone per OSM id. Negative ids (extras) work too.
[[nodiscard]] Colour plaster(std::int64_t osm_id, std::span<const Colour> tones = plaster_tones);

// First year of start_date, construction_date or building:year.
[[nodiscard]] std::optional<int> year_built(const Json& tags);

// Residential block from before 1919: by its date, else by its shape, else the default.
[[nodiscard]] bool is_classic(const Json& tags, FacadeStyle default_style);

[[nodiscard]] Facade facade_from_tags(const Json& tags, std::int64_t osm_id,
                                      FacadeStyle default_style = FacadeStyle::classic);

// One row of the style table the renderer uploads: r, g, b, window pattern,
// weathering, material index or -1.
using StyleRow = std::array<float, 6>;

// Small integer id per distinct facade, shared by all building grids. The
// grids keep id + 1 in the low byte of their style channel; 0 means no
// building. Beyond max_styles distinct facades, a new one reuses the nearest
// colour with the same window pattern. Thread-safe.
class StyleTable {
public:
    static constexpr int max_styles = 254;

    [[nodiscard]] int id(const Facade& facade);
    // The facade of a style id; the default plaster for an unknown id.
    [[nodiscard]] Facade facade(int style) const;
    // Bumped by every new style. The renderer re-uploads the table when it changes.
    [[nodiscard]] std::uint64_t version() const;
    [[nodiscard]] std::size_t size() const;

    struct Snapshot {
        std::uint64_t version;
        // At least one row: row 0 is a fallback when the table is empty.
        std::vector<StyleRow> rows;
    };
    [[nodiscard]] Snapshot snapshot() const;

private:
    [[nodiscard]] int nearest(const Facade& facade) const;

    mutable std::mutex mutex_;
    std::vector<Facade> styles_;
    std::uint64_t version_ = 0;
};

// The table that footprints and grids use.
[[nodiscard]] StyleTable& styles();

}  // namespace docksim::osm
