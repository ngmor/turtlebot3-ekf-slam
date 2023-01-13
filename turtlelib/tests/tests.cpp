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

TEST_CASE("transform a vector", "[transform]") { //Nick Morales
    double x = 3.4, y = 2.2, rot = -0.86;
    Transform2D Tab{Vector2D{x,y}, rot};
    Vector2D vb {-1.1, 2.9};

    Vector2D va = Tab(vb);

    //Vector is transformed
    REQUIRE_THAT(va.x, WithinRel(4.88006221741585, FLOAT_TOL));
    REQUIRE_THAT(va.y, WithinRel(4.92569547686056, FLOAT_TOL));

    //Transform is not modified
    REQUIRE_THAT(Tab.translation().x, WithinRel(x, FLOAT_TOL));
    REQUIRE_THAT(Tab.translation().y, WithinRel(y, FLOAT_TOL));
    REQUIRE_THAT(Tab.rotation(), WithinRel(rot, FLOAT_TOL));
}

TEST_CASE("transform a twist", "[transform]") { //Nick Morales
    double x = -9.1, y = 6.4, rot = 1.57;
    Transform2D Tab{Vector2D{x,y}, rot};
    Twist2D Vb {-1.2,3.3, 2.0};

    Twist2D Va = Tab(Vb);

    //Vector is transformed
    REQUIRE_THAT(Va.w, WithinRel(-1.2, FLOAT_TOL));
    REQUIRE_THAT(Va.x, WithinRel(-9.67737148771825, FLOAT_TOL));
    REQUIRE_THAT(Va.y, WithinRel(-7.61840839290348, FLOAT_TOL));

    //Transform is not modified
    REQUIRE_THAT(Tab.translation().x, WithinRel(x, FLOAT_TOL));
    REQUIRE_THAT(Tab.translation().y, WithinRel(y, FLOAT_TOL));
    REQUIRE_THAT(Tab.rotation(), WithinRel(rot, FLOAT_TOL));
}