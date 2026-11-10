#include <docksim/version.hpp>

#include <catch2/catch_test_macros.hpp>

TEST_CASE("version matches the project version") {
    CHECK(docksim::version() == "0.1.0");
}
