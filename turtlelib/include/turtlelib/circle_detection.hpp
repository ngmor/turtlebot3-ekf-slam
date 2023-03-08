#ifndef CIRCLE_DETECTION_INCLUDE_GUARD_HPP
#define CIRCLE_DETECTION_INCLUDE_GUARD_HPP
/// \file
/// \brief Functions for circle detection.

#include "rigid2d.hpp"
#include <vector>

namespace turtlelib
{
    /// \brief fit a circle from a cluster of points
    /// \param points - points to fit a circle on, represented as vectors from the origin
    /// \return the best fit circle
    Circle2D fit_circle(std::vector<Vector2D> points);
}

#endif