#include "osm/facades.hpp"

#include "osm/tags.hpp"
#include "util/numeric.hpp"

#include <algorithm>
#include <charconv>
#include <limits>
#include <regex>

namespace docksim::osm {

namespace {

using namespace std::string_view_literals;

struct MaterialLook {
    std::string_view name;
    Colour colour;
    double weathering;
};

constexpr std::array<MaterialLook, 11> materials{{
    {.name = "concrete", .colour = {140, 138, 132}, .weathering = 0.6},
    {.name = "reinforced_concrete", .colour = {140, 138, 132}, .weathering = 0.6},
    {.name = "brick", .colour = {150, 78, 56}, .weathering = 0.1},
    {.name = "stone", .colour = {190, 178, 152}, .weathering = 0.3},
    {.name = "sandstone", .colour = {196, 176, 132}, .weathering = 0.3},
    {.name = "limestone", .colour = {206, 198, 176}, .weathering = 0.3},
    {.name = "glass", .colour = {96, 124, 142}, .weathering = 0.0},
    {.name = "metal", .colour = {156, 160, 164}, .weathering = 0.2},
    {.name = "steel", .colour = {156, 160, 164}, .weathering = 0.2},
    {.name = "wood", .colour = {134, 98, 62}, .weathering = 0.2},
    {.name = "timber_framing", .colour = {214, 200, 170}, .weathering = 0.2},
}};

constexpr std::array<std::pair<std::string_view, Colour>, 18> named_colours{{
    {"white", {232, 230, 224}},
    {"gray", {150, 150, 146}},
    {"grey", {150, 150, 146}},
    {"lightgray", {196, 196, 190}},
    {"lightgrey", {196, 196, 190}},
    {"darkgray", {100, 100, 98}},
    {"darkgrey", {100, 100, 98}},
    {"black", {50, 50, 50}},
    {"red", {160, 70, 56}},
    {"brown", {130, 90, 64}},
    {"beige", {214, 200, 168}},
    {"cream", {230, 220, 190}},
    {"yellow", {222, 196, 110}},
    {"orange", {214, 140, 80}},
    {"pink", {214, 168, 160}},
    {"blue", {110, 136, 170}},
    {"green", {120, 150, 110}},
    {"tan", {200, 176, 140}},
}};

// building:material values to material_keys.
constexpr std::array<std::pair<std::string_view, std::string_view>, 10> material_textures{{
    {"concrete", "concrete"},
    {"reinforced_concrete", "concrete"},
    {"brick", "brick"},
    {"stone", "stone"},
    {"sandstone", "stone"},
    {"limestone", "stone"},
    {"metal", "metal"},
    {"steel", "metal"},
    {"wood", "wood"},
    {"glass", ""},
}};

constexpr std::array no_windows{"bunker"sv, "garage"sv,  "garages"sv, "shed"sv,
                                "roof"sv,   "carport"sv, "silo"sv,    "storage_tank"sv};
constexpr std::array industrial{"industrial"sv,  "warehouse"sv, "factory"sv,       "hangar"sv,
                                "manufacture"sv, "barn"sv,      "farm_auxiliary"sv};
constexpr std::array office{"office"sv, "commercial"sv, "retail"sv,    "supermarket"sv,
                            "hotel"sv,  "hospital"sv,   "university"sv};
constexpr std::array historic{"church"sv, "cathedral"sv, "chapel"sv,    "castle"sv,
                              "palace"sv, "monastery"sv, "synagogue"sv, "mosque"sv};
constexpr std::array concrete_types{"bunker"sv, "silo"sv, "parking"sv};
constexpr std::array residential{"yes"sv, "apartments"sv, "residential"sv, "terrace"sv,
                                 "dormitory"sv};

// Knuth multiplicative hash: spreads consecutive OSM ids over the tones.
constexpr std::int64_t knuth = 2654435761;

template <std::size_t N>
bool in(const std::array<std::string_view, N>& set, std::string_view value) {
    return std::ranges::find(set, value) != set.end();
}

std::optional<double> first_number(std::string_view text) {
    static const std::regex number{R"(\d+(?:\.\d+)?)"};
    std::match_results<std::string_view::const_iterator> m;
    if (!std::regex_search(text.begin(), text.end(), m, number)) return std::nullopt;
    return num::parse_double(m.str());
}

std::string_view texture_for(Windows windows) {
    switch (windows) {
        case Windows::none:
            return "concrete";
        case Windows::curtain:
            return "";
        case Windows::industrial:
            return "metal";
        case Windows::historic:
            return "stone";
        case Windows::classic:
            return "plaster_old";
        default:
            return "plaster";
    }
}

}  // namespace

int material_index(std::string_view material) {
    if (material.empty()) return -1;
    const auto* it = std::ranges::find(material_keys, material);
    return it == material_keys.end() ? -1 : static_cast<int>(it - material_keys.begin());
}

std::optional<std::pair<Colour, double>> material_look(std::string_view material) {
    const auto* it = std::ranges::find(materials, material, &MaterialLook::name);
    if (it == materials.end()) return std::nullopt;
    return std::pair{it->colour, it->weathering};
}

std::optional<Colour> parse_colour(std::string_view text) {
    std::string value = lower(trim(text));
    std::erase(value, ' ');
    if (value.empty()) return std::nullopt;
    if (const auto* it =
            std::ranges::find(named_colours, value, [](const auto& p) { return p.first; });
        it != named_colours.end()) {
        return it->second;
    }
    std::string_view digits = value;
    if (digits.starts_with('#')) digits.remove_prefix(1);
    if ((digits.size() != 3 && digits.size() != 6) ||
        !std::ranges::all_of(digits, [](unsigned char c) { return std::isxdigit(c) != 0; })) {
        return std::nullopt;
    }
    std::string full;
    if (digits.size() == 3) {
        for (const char c : digits) full += std::string(2, c);
    } else {
        full = digits;
    }
    Colour out{};
    for (std::size_t i = 0; i < 3; ++i)
        std::from_chars(full.data() + 2 * i, full.data() + 2 * i + 2, out[i], 16);
    return out;
}

Colour plaster(std::int64_t osm_id, std::span<const Colour> tones) {
    // Exact integer maths, as the Python reference: the product needs 128 bits,
    // the shift floors and the modulo is non-negative.
    __extension__ using wide = __int128;
    const wide product = static_cast<wide>(osm_id) * knuth;
    const wide shifted = product >> 7;
    const auto n = static_cast<wide>(tones.size());
    const wide index = ((shifted % n) + n) % n;
    return tones[static_cast<std::size_t>(index)];
}

std::optional<int> year_built(const Json& tags) {
    static const std::regex year{R"(\b(1[0-9]{3}|20[0-9]{2})\b)"};
    for (const auto* key : {"start_date", "construction_date", "building:year"}) {
        const auto value = tag_or(tags, key, "");
        std::smatch m;
        if (std::regex_search(value, m, year)) return std::stoi(m[1].str());
    }
    return std::nullopt;
}

bool is_classic(const Json& tags, FacadeStyle default_style) {
    if (const auto year = year_built(tags)) return *year < classic_until;
    if (const auto levels = first_number(tag_or(tags, "building:levels", ""));
        levels && *levels >= modern_levels) {
        return false;
    }
    if (tag(tags, "roof:shape") == "flat") return false;
    return default_style == FacadeStyle::classic;
}

Facade facade_from_tags(const Json& tags, std::int64_t osm_id, FacadeStyle default_style) {
    std::string kind = lower(tag_or(tags, "building", "no"));
    if (kind == "no") kind = lower(tag_or(tags, "building:part", "yes"));
    const std::string material = lower(tag_or(tags, "building:material", ""));

    Windows windows = Windows::punched;
    if (in(no_windows, kind)) {
        windows = Windows::none;
    } else if (material == "glass") {
        windows = Windows::curtain;
    } else if (in(industrial, kind)) {
        windows = Windows::industrial;
    } else if (in(office, kind)) {
        windows = Windows::ribbon;
    } else if (in(historic, kind) || has_tag(tags, "historic")) {
        windows = Windows::historic;
    } else if (in(residential, kind) && is_classic(tags, default_style)) {
        windows = Windows::classic;
    }

    double weathering = 0.0;
    std::optional<Colour> colour;
    if (const auto look = material_look(material)) {
        colour = look->first;
        weathering = look->second;
    } else if (in(concrete_types, kind)) {
        const auto concrete = material_look("concrete").value();
        colour = concrete.first;
        weathering = concrete.second;
    }
    auto explicit_colour = parse_colour(tag_or(tags, "building:colour", ""));
    if (!explicit_colour) explicit_colour = parse_colour(tag_or(tags, "colour", ""));
    if (explicit_colour) colour = explicit_colour;
    if (!colour) {
        if (windows == Windows::industrial) {
            colour = Colour{156, 160, 164};
        } else if (windows == Windows::classic) {
            colour = plaster(osm_id, classic_plaster_tones);
        } else {
            colour = plaster(osm_id);
        }
    }
    if (tag(tags, "historic") == "ruins" || tag(tags, "ruins") == "yes")
        weathering = std::max(weathering, 0.9);

    std::string texture;
    if (const auto* it =
            std::ranges::find(material_textures, material, [](const auto& p) { return p.first; });
        it != material_textures.end()) {
        texture = it->second;
    } else if (in(concrete_types, kind)) {
        texture = "concrete";
    } else {
        texture = texture_for(windows);
    }
    return {.colour = *colour, .windows = windows, .weathering = weathering, .material = texture};
}

int StyleTable::id(const Facade& facade) {
    const std::scoped_lock lock(mutex_);
    if (const auto it = std::ranges::find(styles_, facade); it != styles_.end()) {
        return static_cast<int>(it - styles_.begin());
    }
    if (std::cmp_greater_equal(styles_.size(), max_styles)) return nearest(facade);
    styles_.push_back(facade);
    ++version_;
    return static_cast<int>(styles_.size()) - 1;
}

Facade StyleTable::facade(int style) const {
    const std::scoped_lock lock(mutex_);
    if (style >= 0 && std::cmp_less(style, styles_.size()))
        return styles_[static_cast<std::size_t>(style)];
    return {.colour = plaster_tones[0],
            .windows = Windows::punched,
            .weathering = 0.0,
            .material = "plaster"};
}

std::uint64_t StyleTable::version() const {
    const std::scoped_lock lock(mutex_);
    return version_;
}

std::size_t StyleTable::size() const {
    const std::scoped_lock lock(mutex_);
    return styles_.size();
}

StyleTable::Snapshot StyleTable::snapshot() const {
    std::vector<Facade> copy;
    std::uint64_t version = 0;
    {
        const std::scoped_lock lock(mutex_);
        copy = styles_;
        version = version_;
    }
    if (copy.empty()) copy.push_back({plaster_tones[0], Windows::punched, 0.0, "plaster"});
    Snapshot out{.version = version, .rows = {}};
    for (const auto& s : copy) {
        out.rows.push_back(
            {static_cast<float>(s.colour[0]), static_cast<float>(s.colour[1]),
             static_cast<float>(s.colour[2]), static_cast<float>(std::to_underlying(s.windows)),
             static_cast<float>(s.weathering), static_cast<float>(material_index(s.material))});
    }
    return out;
}

int StyleTable::nearest(const Facade& facade) const {
    double best = std::numeric_limits<double>::infinity();
    int index = 0;
    for (std::size_t i = 0; i < styles_.size(); ++i) {
        const auto& other = styles_[i];
        double d = other.windows == facade.windows ? 0.0 : 1e6;
        for (std::size_t c = 0; c < 3; ++c) {
            const double diff = other.colour[c] - facade.colour[c];
            d += diff * diff;
        }
        if (d < best) {
            best = d;
            index = static_cast<int>(i);
        }
    }
    return index;
}

StyleTable& styles() {
    static StyleTable table;
    return table;
}

}  // namespace docksim::osm
