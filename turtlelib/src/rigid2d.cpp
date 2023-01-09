#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <cmath>

namespace turtlelib
{

    /* VECTOR2D START */

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
        os << '[' << v.x << ' ' << v.y << ']';
        return os;
    }

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param v [out] - output vector
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
    };

    /* VECTOR2D END */



    /* TRANSFORM2D START */

    /// \brief Create an identity transformation
    /// Private variables are already initialized as an identity transformation
    Transform2D::Transform2D() {}

    // \brief create a transformation that is a pure translation
    /// \param trans - the vector by which to translate
    Transform2D::Transform2D(Vector2D trans): trans_{trans} {}

    /// \brief create a pure rotation
    /// \param rot - angle of the rotation, in radians
    Transform2D::Transform2D(double rot): rot_{rot} {}

    /// \brief Create a transformation with a translational and rotational
    /// component
    /// \param trans - the translation
    /// \param rot - the rotation, in radians
    Transform2D::Transform2D(Vector2D trans, double rot): trans_{trans}, rot_{rot} {}

    /// \brief apply a transformation to a Vector2D
    /// \param v - the vector to transform
    /// \return a vector in the new coordinate system
    Vector2D Transform2D::operator()(Vector2D v) const {
        return Vector2D {
            v.x*std::cos(rot_) - v.y*std::sin(rot_) + trans_.x,
            v.x*std::sin(rot_) - v.y*std::cos(rot_) + trans_.y
        };
    }

    /* TRANSFORM2D END */
}