#include <armadillo>
#include "turtlelib/circle_detection.hpp"



namespace turtlelib
{
    Circle2D fit_circle(const std::vector<Vector2D> & points) {
        //If no points, return default circle object
        if (points.empty()) {
            return {};
        }

        // Collect points into x and y vectors
        std::vector<double> x;
        std::vector<double> y;
        x.reserve(points.size());
        y.reserve(points.size());

        for (const auto & point : points) {
            x.push_back(point.x);
            y.push_back(point.y);
        }

        //Calculate centroid of points

        return {};
    }
}