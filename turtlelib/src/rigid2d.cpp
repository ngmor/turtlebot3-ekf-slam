#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <cmath>

namespace turtlelib
{

    double normalize_angle(double rad) {
        //Bound rotation between -pi and pi
        while (rad > PI) {
            rad -= 2.0*PI;
        }
        while (rad <= -PI) {
            rad += 2.0*PI;
        }

        return rad;
    }


    ////////// VECTOR2D START //////////

    double Vector2D::magnitude() const {
        return std::sqrt(std::pow(x,2) + std::pow(y,2));
    }

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

        auto c = is.peek();

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
        const auto mag = v.magnitude();

        return {
            v.x / mag,
            v.y / mag
        };
    }

    double dot(const Vector2D & lhs, const Vector2D & rhs) {
        return lhs.x*rhs.x + lhs.y*rhs.y;
    }

    double angle(const Vector2D & start, const Vector2D & end) {
        //https://www.mathworks.com/matlabcentral/answers/180131-how-can-i-find-the-angle-between-two-vectors-including-directional-information
        return std::atan2(start.x*end.y - start.y*end.x, dot(start, end));
    }

    ////////// VECTOR2D END //////////



    ////////// TWIST2D START //////////

    std::ostream & operator<<(std::ostream & os, const Twist2D & V) {
        os << '[' << V.w << ' ' << V.x << ' ' << V.y << ']';
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & V) {
        //remove leading whitespace
        is >> std::ws;

        auto c = is.peek();

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

    ////////// TWIST2D END //////////
    
    
    
    ////////// TRANSFORM2D START //////////

    /// Private variables are already initialized as an identity transformation
    Transform2D::Transform2D() {}

    Transform2D::Transform2D(Vector2D trans): Transform2D(trans, 0.0) {}

    Transform2D::Transform2D(double rot): Transform2D(Vector2D {0.0, 0.0}, rot) {}

    Transform2D::Transform2D(Vector2D trans, double rot)
    : trans_{trans}, rot_{normalize_angle(rot)}
    {
        cache_trig();
    }

    void Transform2D::cache_trig() {
        rot_sin_ = std::sin(rot_);
        rot_cos_ = std::cos(rot_);
    }

    Vector2D Transform2D::operator()(Vector2D v) const {
        return {
            v.x*rot_cos_ - v.y*rot_sin_ + trans_.x,
            v.x*rot_sin_ + v.y*rot_cos_ + trans_.y
        };
    }

    Twist2D Transform2D::operator()(Twist2D V) const {
        return {
            V.w,
            V.w*trans_.y + V.x*rot_cos_ - V.y*rot_sin_,
            -V.w*trans_.x + V.x*rot_sin_ + V.y*rot_cos_
        };
    }

    Transform2D Transform2D::inv() const {
        return {
            //translation
            {
                -trans_.x*rot_cos_ - trans_.y*rot_sin_,
                -trans_.y*rot_cos_ + trans_.x*rot_sin_
            },
            //rotation
            -rot_
        };
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs) {
        
        //Output translation adds current translation to incoming translation
        //modified by current rotation
        trans_.x += rhs.translation().x*rot_cos_
                  - rhs.translation().y*rot_sin_;
        trans_.y += rhs.translation().x*rot_sin_
                  + rhs.translation().y*rot_cos_;
        
        //Output rotation just adds the angles together
        rot_ += rhs.rotation();

        //Bound rotation
        rot_ = normalize_angle(rot_);

        //Cache trig values
        cache_trig();

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

        auto c = is.peek();

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
            {x,y},
            deg2rad(deg)
        };

        return is;
    }

    Transform2D integrate_twist(Twist2D twist) {
        if (almost_equal(twist.w, 0.0)) {
            return {
                {twist.x, twist.y},
                0.0
            };
        } else {
            //Frames:
            //b = body frame before motion
            //bp = body frame after motion
            //s = frame at center of rotation (COR) aligned with b frame
            //sp = frame at center of rotation (COR) aligned with bp frame
            
            //Purely rotational motion at COR to get from s to sp
            Transform2D Tssp {twist.w};

            //Use adjoint between known twists in s and b frames to solve for Tsb
            Transform2D Tsb {
                {
                    twist.y / twist.w,
                    -twist.x / twist.w
                },
                0.0 //no rotation between s and b frames, they are aligned
            };

            //Tbbp = TbsTsspTspbp
            //Tsb = Tspbp
            //so
            //Tbbp = Tsb^-1 * Tssp * Tsb
            return Tsb.inv()*Tssp*Tsb;
        }
    }

    ////////// TRANSFORM2D END //////////
}