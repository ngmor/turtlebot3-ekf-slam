#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Modeling kinematics of a differential drive robot.

#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{
    /// \brief configuration information for a differential drive robot
    struct DiffDriveConfig
    {
        /// \brief location of robot in the world frame
        Transform2D location {Vector2D{0,0},0};

        /// \brief left wheel position
        double wheel_left = 0.0;

        /// \brief right wheel position
        double wheel_right = 0.0;

    };

    class DiffDrive
    {
    private:

        double wheel_track_ = 0.0;
        double wheel_radius_ = 0.0;

        /// \brief initial configuration
        DiffDriveConfig config_init_ {{Vector2D{0,0},0}, 0, 0};

        /// \brief current configuration
        DiffDriveConfig config_ {{Vector2D{0,0},0}, 0, 0};

    public:

        /// \brief create a differential drive robot starting at the origin
        /// \param wheel_track - distance between the robot's wheels, must be > 0
        /// \param wheel_radius - wheel radius, must be > 0
        DiffDrive(double wheel_track, double wheel_radius);

        /// \brief create a differential drive robot with a custom starting location
        /// \param wheel_track - distance between the robot's wheels, must be > 0
        /// \param wheel_radius - wheel radius, must be > 0
        /// \param start_location - starting location transformation
        DiffDrive(double wheel_track, double wheel_radius, Transform2D start_location);

        /// \brief create a differential drive robot with custom starting wheel positions
        /// \param wheel_track - distance between the robot's wheels, must be > 0
        /// \param wheel_radius - wheel radius, must be > 0
        /// \param start_position_left_wheel - left wheel starting position
        /// \param start_position_right_wheel - right wheel starting position
        DiffDrive(double wheel_track, double wheel_radius,
            double start_position_left_wheel, double start_position_right_wheel);
        
        /// \brief create a differential drive robot with a custom starting location
        /// and custom starting wheel position
        /// \param wheel_track - distance between the robot's wheels, must be > 0
        /// \param wheel_radius - wheel radius, must be > 0
        /// \param start_location - starting location transformation
        /// \param start_position_left_wheel - left wheel starting position
        /// \param start_position_right_wheel - right wheel starting position
        DiffDrive(double wheel_track, double wheel_radius, Transform2D start_location,
            double start_position_left_wheel, double start_position_right_wheel);

        /// \brief reset current configuration to initial configuration
        void reset();
    };
}

#endif