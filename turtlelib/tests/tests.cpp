#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/rigid2d.hpp"
using turtlelib::Transform2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;
using Catch::Matchers::WithinRel;

#define FLOAT_TOL 1e-8

TEST_CASE("constructors and getters", "[transform]") {
    SECTION("default constructor") {
        Transform2D T;

        REQUIRE_THAT(T.translation().x, WithinRel(0.0, FLOAT_TOL));
        REQUIRE_THAT(T.translation().y, WithinRel(0.0, FLOAT_TOL));
        REQUIRE_THAT(T.rotation(), WithinRel(0.0, FLOAT_TOL));
    }
}