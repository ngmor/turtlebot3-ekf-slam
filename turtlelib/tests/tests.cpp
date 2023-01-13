#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/rigid2d.hpp"
using turtlelib::Transform2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;
using Catch::Matchers::WithinRel;

#define FLOAT_TOL 1e-8

TEST_CASE("constructors and getters", "[transform]") { //Nick Morales
    SECTION("default constructor") {
        Transform2D T;

        REQUIRE_THAT(T.translation().x, WithinRel(0.0, FLOAT_TOL));
        REQUIRE_THAT(T.translation().y, WithinRel(0.0, FLOAT_TOL));
        REQUIRE_THAT(T.rotation(), WithinRel(0.0, FLOAT_TOL));
    }
    SECTION("pure translation constructor") {
        double x = 7.2, y = -5.6;
        Transform2D T {Vector2D{x,y}};

        REQUIRE_THAT(T.translation().x, WithinRel(x, FLOAT_TOL));
        REQUIRE_THAT(T.translation().y, WithinRel(y, FLOAT_TOL));
        REQUIRE_THAT(T.rotation(), WithinRel(0.0, FLOAT_TOL));
    }
    SECTION("pure rotation constructor") {
        double rot = 1.23;
        Transform2D T {rot};

        REQUIRE_THAT(T.translation().x, WithinRel(0.0, FLOAT_TOL));
        REQUIRE_THAT(T.translation().y, WithinRel(0.0, FLOAT_TOL));
        REQUIRE_THAT(T.rotation(), WithinRel(rot, FLOAT_TOL));
    }
    SECTION("rotation and translation constructor") {
        double x = -5.7, y = 7.2, rot = -0.58;
        Transform2D T{Vector2D{x,y}, rot};

        REQUIRE_THAT(T.translation().x, WithinRel(x, FLOAT_TOL));
        REQUIRE_THAT(T.translation().y, WithinRel(y, FLOAT_TOL));
        REQUIRE_THAT(T.rotation(), WithinRel(rot, FLOAT_TOL));
    }
}