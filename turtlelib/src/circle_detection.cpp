#include <armadillo>
#include "turtlelib/circle_detection.hpp"



namespace turtlelib
{
    Circle2D fit_circle(const std::vector<Vector2D> & points) {
        //based on:
        //https://projecteuclid.org/journals/electronic-journal-of-statistics/volume-3/issue-none/Error-analysis-for-circle-fitting-algorithms/10.1214/09-EJS419.full

        //If no points, return default circle object
        if (points.empty()) {
            return {};
        }

        const auto num_points = points.size();

        // Collect points into x and y vectors
        std::vector<double> x_coords;
        std::vector<double> y_coords;
        x_coords.reserve(num_points);
        y_coords.reserve(num_points);

        for (const auto & point : points) {
            x_coords.push_back(point.x);
            y_coords.push_back(point.y);
        }

        //Calculate centroid of points
        const Vector2D centroid {mean(x_coords), mean(y_coords)};

        //Shift coordinates so that the centroid is at the origin
        for (auto & x : x_coords) {
            x -= centroid.x;
        }
        for (auto & y : y_coords) {
            y -= centroid.y;
        }

        //Compute z values
        std::vector<double> z_vals (num_points, 0.0);

        for (size_t i = 0; i < num_points; i++) {
            z_vals[i] = x_coords[i]*x_coords[i] + y_coords[i]*y_coords[i];
        }

        //Compute the mean of z
        const auto z_mean = mean(z_vals);



        return {};
    }
}