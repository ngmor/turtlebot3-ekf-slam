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

        //Collect points into x and y vectors
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

        std::vector<double> z_vals (num_points, 0.0);
        arma::mat Z {num_points, 4, arma::fill::zeros};

        for (size_t i = 0; i < num_points; i++) {
            //Init references
            auto & x = x_coords[i];
            auto & y = y_coords[i];
            auto & z = z_vals[i];

            //Shift coordinates so that the centroid is at the origin
            x -= centroid.x;
            y -= centroid.y;

            //Compute z values
            z = x*x + y*y;

            //Form the data matrix Z from the data points
            Z(i, 0) = z;
            Z(i, 1) = x;
            Z(i, 2) = y;
            Z(i, 3) = 1.0;
        }

        //Compute the mean of z
        const auto z_mean = mean(z_vals);

        //Form the moment matrix M
        arma::mat M = (1.0 / num_points) * Z.t() * Z;

        return {};
    }
}