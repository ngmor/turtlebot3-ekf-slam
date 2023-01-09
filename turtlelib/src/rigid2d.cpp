#include "turtlelib/rigid2d.hpp"
#include <iostream>

namespace turtlelib
{
    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
        os << '[' << v.x << ' ' << v.y << ']';
        return os;
    }
}