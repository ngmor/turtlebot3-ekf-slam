#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <cmath>

namespace turtlelib
{

    double normalize_angle(double rad) {
        //Bound rotation between -pi and pi
        while (rad > PI) {
            rad -= 2*PI;
        }
        while (rad <= -PI) {
            rad += 2*PI;
        }

        return rad;
    }


    /* VECTOR2D START */

    Vector2D & Vector2D::operator+=(const Vector2D & rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D & rhs) {
        lhs += rhs;
        return lhs;
    }

    Vector2D & Vector2D::operator-=(const Vector2D & rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D & rhs) {
        lhs -= rhs;
        return lhs;
    }

    Vector2D & Vector2D::operator*=(const double scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Vector2D operator*(Vector2D vector, const double scalar) {
        vector*=scalar;
        return vector;
    }

    Vector2D operator*(const double scalar, Vector2D vector) {
        vector*=scalar;
        return vector;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
        os << '[' << v.x << ' ' << v.y << ']';
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v) {
        //remove leading whitespace
        is >> std::ws;

        char c = is.peek();

        if (c == '[') {
            //Remove leading bracket
            c = is.get();
        }

        //Get actual data
        is >> v.x >> v.y;

        c = is.peek();

        if (c == ']') {
            //Remove trailing bracket
            c = is.get();
        }

        return is;
    }

    Vector2D normalize(const Vector2D & v) {
        double mag = std::sqrt(std::pow(v.x,2) + std::pow(v.y,2));

        return Vector2D {
            v.x / mag,
            v.y / mag
        };
    }

    /* VECTOR2D END */



    /* TWIST2D START */

    std::ostream & operator<<(std::ostream & os, const Twist2D & V) {
        os << '[' << V.w << ' ' << V.x << ' ' << V.y << ']';
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & V) {
        //remove leading whitespace
        is >> std::ws;

        char c = is.peek();

        if (c == '[') {
            //Remove leading bracket
            c = is.get();
        }

        //Get actual data
        is >> V.w >> V.x >> V.y;

        c = is.peek();

        if (c == ']') {
            //Remove trailing bracket
            c = is.get();
        }

        return is;
    }

    /* TWIST2D END */
    
    
    
    /* TRANSFORM2D START */

    /// Private variables are already initialized as an identity transformation
    Transform2D::Transform2D() {}

    Transform2D::Transform2D(Vector2D trans): trans_{trans} {}

    Transform2D::Transform2D(double rot): rot_{normalize_angle(rot)} {}

    Transform2D::Transform2D(Vector2D trans, double rot): trans_{trans}, rot_{normalize_angle(rot)} {}

    Vector2D Transform2D::operator()(Vector2D v) const {
        return Vector2D {
            v.x*std::cos(rot_) - v.y*std::sin(rot_) + trans_.x,
            v.x*std::sin(rot_) + v.y*std::cos(rot_) + trans_.y
        };
    }

    Twist2D Transform2D::operator()(Twist2D V) const {
        return Twist2D {
            V.w,
            V.w*trans_.y + V.x*std::cos(rot_) - V.y*std::sin(rot_),
            -V.w*trans_.x + V.x*std::sin(rot_) + V.y*std::cos(rot_)
        };
    }

    Transform2D Transform2D::inv() const {
        return Transform2D {
            //translation
            Vector2D {
                -trans_.x*std::cos(rot_) - trans_.y*std::sin(rot_),
                -trans_.y*std::cos(rot_) + trans_.x*std::sin(rot_)
            },
            //rotation
            -rot_
        };
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs) {
        
        //Output translation adds current translation to incoming translation
        //modified by current rotation
        this->trans_.x += rhs.translation().x*std::cos(this->rot_)
                        - rhs.translation().y*std::sin(this->rot_);
        this->trans_.y += rhs.translation().x*std::sin(this->rot_)
                        + rhs.translation().y*std::cos(this->rot_);
        
        //Output rotation just adds the angles together
        this->rot_ += rhs.rotation();

        //Bound rotation
        rot_ = normalize_angle(rot_);

        return *this;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs) {
        lhs*=rhs;
        return lhs;
    }

    Vector2D Transform2D::translation() const { return trans_; }

    double Transform2D::rotation() const { return rot_; }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf) {
        os << "deg: " << rad2deg(tf.rotation())
           << " x: " << tf.translation().x
           << " y: " << tf.translation().y;
        return os;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf) {
        double deg, x, y;
        
        //remove leading whitespace
        is >> std::ws;

        char c = is.peek();

        //remove any characters that aren't digits
        while (!(std::isdigit(c) || (c == '-') || (c == '.'))) {
            c = is.get();
            c = is.peek();
        }

        //Get rotation
        is >> deg;


        c = is.peek();

        //remove any characters that aren't digits
        while (!(std::isdigit(c) || (c == '-') || (c == '.'))) {
            c = is.get();
            c = is.peek();
        }

        //Get x translation
        is >> x;


        c = is.peek();

        //remove any characters that aren't digits
        while (!(std::isdigit(c) || (c == '-') || (c == '.'))) {
            c = is.get();
            c = is.peek();
        }

        //Get y translation
        is >> y;

        //Init transform
        tf = Transform2D{
            Vector2D{x,y},
            deg2rad(deg)
        };

        return is;
    }

    /* TRANSFORM2D END */
}