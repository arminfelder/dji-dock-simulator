#include "osm/facades.hpp"
#include "osm/footprints.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <tuple>

using namespace docksim;
using namespace docksim::osm;

TEST_CASE("osm: the flak tower is weathered concrete without windows") {
    const Json tags = {
        {"building", "bunker"}, {"building:material", "concrete"}, {"historic", "ruins"}};
    const auto facade = facade_from_tags(tags, 263795490);
    CHECK(facade.windows == Windows::none);
    CHECK(facade.colour == material_look("concrete")->first);
    CHECK(facade.weathering >= 0.9);
}

TEST_CASE("osm: an explicit colour wins over the material") {
    const Json tags = {
        {"building", "bunker"}, {"building:material", "concrete"}, {"colour", "gray"}};
    CHECK(facade_from_tags(tags, 1).colour == Colour{150, 150, 146});
    CHECK(facade_from_tags({{"building:colour", "#f00"}}, 1).colour == Colour{255, 0, 0});
    CHECK(parse_colour("#A0b0C0") == Colour{160, 176, 192});
    CHECK_FALSE(parse_colour("#12"));
}

TEST_CASE("osm: the building type picks the window pattern") {
    CHECK(facade_from_tags({{"building", "office"}}, 1).windows == Windows::ribbon);
    CHECK(facade_from_tags({{"building", "warehouse"}}, 1).windows == Windows::industrial);
    CHECK(facade_from_tags({{"building", "church"}}, 1).windows == Windows::historic);
    CHECK(facade_from_tags({{"building", "yes"}, {"building:material", "glass"}}, 1).windows ==
          Windows::curtain);
    CHECK(facade_from_tags({{"building", "house"}}, 1).windows == Windows::punched);
}

TEST_CASE("osm: residential era from date, shape and default") {
    const auto kind = [](Json tags, FacadeStyle style = FacadeStyle::classic) {
        tags["building"] = "apartments";
        return facade_from_tags(tags, 1, style).windows;
    };
    CHECK(kind(Json::object()) == Windows::classic);
    CHECK(kind(Json::object(), FacadeStyle::modern) == Windows::punched);
    CHECK(kind({{"start_date", "1872"}}, FacadeStyle::modern) == Windows::classic);
    CHECK(kind({{"start_date", "~1965"}}) == Windows::punched);
    CHECK(kind({{"building:levels", "12"}}) == Windows::punched);
    CHECK(kind({{"roof:shape", "flat"}}) == Windows::punched);
    const auto colour = facade_from_tags({{"building", "apartments"}}, 7).colour;
    CHECK(std::ranges::contains(classic_plaster_tones, colour));
}

TEST_CASE("osm: classic levels are taller") {
    const Json item = {{"id", 1},
                       {"tags", {{"building", "apartments"}, {"building:levels", "5"}}},
                       {"outer", Json::array({Json::array({Json::array({48.0, 16.0})})})},
                       {"inner", Json::array()}};
    CHECK(from_cache(item, {}).height == 5 * classic_level_m + 1.0);
    CHECK(from_cache(item, {.default_style = FacadeStyle::modern}).height == 5 * 3.0 + 1.0);
    CHECK(from_cache(item, {}).levels == 5);
    Json untagged = item;
    untagged["tags"].erase("building:levels");
    CHECK(from_cache(untagged, {}).levels == 0);
}

TEST_CASE("osm: a part uses its part type") {
    CHECK(facade_from_tags({{"building", "no"}, {"building:part", "garage"}}, 1).windows ==
          Windows::none);
}

TEST_CASE("osm: plain buildings get a stable plaster colour") {
    const auto first = facade_from_tags({{"building", "yes"}}, 42, FacadeStyle::modern);
    CHECK(first == facade_from_tags({{"building", "yes"}}, 42, FacadeStyle::modern));
    CHECK(std::ranges::contains(plaster_tones, first.colour));
    CHECK(first.weathering == 0.0);
}

TEST_CASE("osm: plaster tone matches the Python hash for large and negative ids") {
    // (id * 2654435761 >> 7) % 6 in Python integers: 263795490 -> 5, -263795549 -> 5.
    CHECK(plaster(263795490) == plaster_tones[5]);
    CHECK(plaster(-263795549) == plaster_tones[5]);
    CHECK(plaster(-7) == plaster_tones[0]);
    CHECK(plaster(-1) == plaster_tones[2]);
    CHECK(plaster(13'000'000'000) == plaster_tones[2]);
    CHECK(plaster(99) == plaster_tones[4]);
}

TEST_CASE("osm: a full style table reuses the nearest style") {
    StyleTable table;
    for (int k = 0; k < StyleTable::max_styles; ++k)
        std::ignore = table.id({.colour = {k, 0, 0},
                                .windows = Windows::punched,
                                .weathering = 0.0,
                                .material = "plaster"});
    const auto version = table.version();
    CHECK(table.id({{10, 1, 0}, Windows::punched, 0.0, "plaster"}) == 10);
    CHECK(table.version() == version);
    CHECK(table.snapshot().rows.size() == StyleTable::max_styles);
}

TEST_CASE("osm: style table rows carry the material index") {
    StyleTable table;
    CHECK(table.snapshot().rows.size() == 1);
    std::ignore = table.id(
        {.colour = {1, 2, 3}, .windows = Windows::classic, .weathering = 0.5, .material = "brick"});
    std::ignore = table.id(
        {.colour = {1, 2, 3}, .windows = Windows::curtain, .weathering = 0.0, .material = ""});
    const auto rows = table.snapshot().rows;
    CHECK(rows[0] == StyleRow{1, 2, 3, 6, 0.5F, 3});
    CHECK(rows[1][5] == -1.0F);
}
