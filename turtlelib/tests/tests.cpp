#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/rigid2d.hpp"
#include <string>
#include <sstream>
using turtlelib::Transform2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;
using turtlelib::deg2rad;
using turtlelib::PI;
using turtlelib::normalize_angle;
using Catch::Matchers::WithinRel;
using std::stringstream;
using std::string;

constexpr double FLOAT_TOL = 1e-8;

TEST_CASE("angle normalization", "[normalize_angle]") { //Nick Morales
    REQUIRE_THAT(normalize_angle(PI), WithinRel(PI, FLOAT_TOL));
    REQUIRE_THAT(normalize_angle(-PI), WithinRel(PI, FLOAT_TOL));
    REQUIRE_THAT(normalize_angle(0.0), WithinRel(0.0, FLOAT_TOL));
    REQUIRE_THAT(normalize_angle(-PI/4.0), WithinRel(-PI/4.0, FLOAT_TOL));
    REQUIRE_THAT(normalize_angle(3.0*PI/2.0), WithinRel(-PI/2.0, FLOAT_TOL));
    REQUIRE_THAT(normalize_angle(-5.0*PI/2.0), WithinRel(-PI/2.0, FLOAT_TOL));
}

TEST_CASE("vector addition", "[vector]") { //Nick Morales
    double x1 = 0.44, y1 = 2.14;
    Vector2D V1 {x1,y1};
    double x2 = -1.37, y2 = 0.53;
    Vector2D V2 {x2,y2};


    SECTION("in-place modification") {
        V1+=V2;

        //T1 modified
        REQUIRE_THAT(V1.x, WithinRel(x1 + x2, FLOAT_TOL));
        REQUIRE_THAT(V1.y, WithinRel(y1 + y2, FLOAT_TOL));

        //T2 not modified
        REQUIRE_THAT(V2.x, WithinRel(x2, FLOAT_TOL));
        REQUIRE_THAT(V2.y, WithinRel(y2, FLOAT_TOL));

    }

    SECTION("return to separate variable") {
        Vector2D Vres = V1+V2;

        //T1 not modified
        REQUIRE_THAT(V1.x, WithinRel(x1, FLOAT_TOL));
        REQUIRE_THAT(V1.y, WithinRel(y1, FLOAT_TOL));

        //T2 not modified
        REQUIRE_THAT(V2.x, WithinRel(x2, FLOAT_TOL));
        REQUIRE_THAT(V2.y, WithinRel(y2, FLOAT_TOL));

        //Result
        REQUIRE_THAT(Vres.x, WithinRel(x1 + x2, FLOAT_TOL));
        REQUIRE_THAT(Vres.y, WithinRel(y1 + y2, FLOAT_TOL));
    }
}

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

TEST_CASE("inverse", "[transform]") { //Nick Morales
    double x = 0.22, y = -0.22, rot = 2.44;
    Transform2D T{Vector2D{x,y}, rot};

    Transform2D Tinv = T.inv();

    //Vector is transformed
    REQUIRE_THAT(Tinv.translation().x, WithinRel(0.310035044090672, FLOAT_TOL));
    REQUIRE_THAT(Tinv.translation().y, WithinRel(-0.0260436448235487, FLOAT_TOL));
    REQUIRE_THAT(Tinv.rotation(), WithinRel(-2.44, FLOAT_TOL));

    //Transform is not modified
    REQUIRE_THAT(T.translation().x, WithinRel(x, FLOAT_TOL));
    REQUIRE_THAT(T.translation().y, WithinRel(y, FLOAT_TOL));
    REQUIRE_THAT(T.rotation(), WithinRel(rot, FLOAT_TOL));
}

TEST_CASE("transform composition", "[transform]") { //Nick Morales
    double x1 = -3.72, y1 = -4.67, rot1 = 2.85;
    Transform2D T1{Vector2D{x1,y1}, rot1};
    double x2 = 4.93, y2 = 1.28, rot2 = 2.97;
    Transform2D T2{Vector2D{x2,y2}, rot2};
    double xres = -8.80986293693519, yres = -4.47870106321921, rotres = -0.4631853071795862;


    SECTION("in-place modification") {
        T1*=T2;

        //T1 modified
        REQUIRE_THAT(T1.translation().x, WithinRel(xres, FLOAT_TOL));
        REQUIRE_THAT(T1.translation().y, WithinRel(yres, FLOAT_TOL));
        REQUIRE_THAT(T1.rotation(), WithinRel(rotres, FLOAT_TOL));

        //T2 not modified
        REQUIRE_THAT(T2.translation().x, WithinRel(x2, FLOAT_TOL));
        REQUIRE_THAT(T2.translation().y, WithinRel(y2, FLOAT_TOL));
        REQUIRE_THAT(T2.rotation(), WithinRel(rot2, FLOAT_TOL));

    }

    SECTION("return to separate variable") {
        Transform2D Tres = T1*T2;

        //T1 not modified
        REQUIRE_THAT(T1.translation().x, WithinRel(x1, FLOAT_TOL));
        REQUIRE_THAT(T1.translation().y, WithinRel(y1, FLOAT_TOL));
        REQUIRE_THAT(T1.rotation(), WithinRel(rot1, FLOAT_TOL));

        //T2 not modified
        REQUIRE_THAT(T2.translation().x, WithinRel(x2, FLOAT_TOL));
        REQUIRE_THAT(T2.translation().y, WithinRel(y2, FLOAT_TOL));
        REQUIRE_THAT(T2.rotation(), WithinRel(rot2, FLOAT_TOL));

        //Result
        REQUIRE_THAT(Tres.translation().x, WithinRel(xres, FLOAT_TOL));
        REQUIRE_THAT(Tres.translation().y, WithinRel(yres, FLOAT_TOL));
        REQUIRE_THAT(Tres.rotation(), WithinRel(rotres, FLOAT_TOL));
    }
}

TEST_CASE("output stream", "[transform]") { //Nick Morales
    stringstream ss;
    
    SECTION("input 1") {
        Transform2D T{Vector2D{-3.38, -2.22}, deg2rad(117)};

        ss << T;

        REQUIRE (ss.str() == "deg: 117 x: -3.38 y: -2.22");
    }

    SECTION("input 2") {
        Transform2D T{Vector2D{2.84, -2.97}, deg2rad(117)};

        ss << T;

        REQUIRE (ss.str() == "deg: 117 x: 2.84 y: -2.97");
    }
}

TEST_CASE("input stream", "[transform]") { //Nick Morales
    stringstream ss;
    string line;
    Transform2D T;

    SECTION("with labels") {
        line = "deg: -24 x: -3.24 y: 3.83";
        ss << line;
        ss >> T;

        REQUIRE_THAT(T.translation().x, WithinRel(-3.24, FLOAT_TOL));
        REQUIRE_THAT(T.translation().y, WithinRel(3.83, FLOAT_TOL));
        REQUIRE_THAT(T.rotation(), WithinRel(deg2rad(-24), FLOAT_TOL));
    }

    SECTION("without labels") {
        line = "142 1.73 -2.23";
        ss << line;
        ss >> T;

        REQUIRE_THAT(T.translation().x, WithinRel(1.73, FLOAT_TOL));
        REQUIRE_THAT(T.translation().y, WithinRel(-2.23, FLOAT_TOL));
        REQUIRE_THAT(T.rotation(), WithinRel(deg2rad(142), FLOAT_TOL));
    }
}