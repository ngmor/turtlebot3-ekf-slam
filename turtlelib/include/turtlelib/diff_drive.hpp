#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Modeling kinematics of a differential drive robot.

#include "turtlelib/rigid2d.hpp"
#include <exception>
#include <string>


namespace turtlelib
{

    /// \brief wheel data (positions, velocities, etc)
    struct Wheel
    {
        /// \brief left wheel data
        double left = 0.0;

        /// \brief right wheel data
        double right = 0.0;

    };

    /// \brief configuration information for a differential drive robot
    struct DiffDriveConfig
    {
        /// \brief location of robot in the world frame
        Transform2D location {Vector2D{0,0},0};

        /// \brief wheel positions
        Wheel wheel_pos {0,0};

    };

    class DiffDrive
    {
    private:

        /// \brief distance between the robot's wheels, must be > 0
        double wheel_track_ = 0.0;

        /// \brief wheel radius, must be > 0
        double wheel_radius_ = 0.0;

        /// \brief initial configuration
        DiffDriveConfig config_init_ {{Vector2D{0,0},0}, Wheel{0, 0}};

        /// \brief current configuration
        DiffDriveConfig config_ {{Vector2D{0,0},0}, Wheel{0, 0}};

        /// \brief calculate and store coefficients for kinematics
        /// based on robot's physical parameters
        void calc_kinematic_coeff();

        /// \brief inverse kinematics angular velocity coefficient
        double coeff_ik_w_ = 0.0;

        /// \brief inverse kinematics linear x velocity coefficient
        double coeff_ik_x_ = 0.0;

        /// \brief forward kinematics angular velocity coefficient
        double coeff_fk_w_ = 0.0;

        /// \brief forward kinematics linear x velocity coefficient 
        double coeff_fk_x_ = 0.0;

        

    public:

        /// \brief create a differential drive robot starting at the origin
        /// \param wheel_track - distance between the robot's wheels, must be > 0
        /// \param wheel_radius - wheel radius, must be > 0
        DiffDrive(double wheel_track, double wheel_radius);

        /// \brief create a differential drive robot with a custom starting location
        /// \param wheel_track - distance between the robot's wheels, must be > 0
        /// \param wheel_radius - wheel radius, must be > 0
        /// \param start_location - starting location transformation
        DiffDrive(double wheel_track, double wheel_radius, const Transform2D & start_location);

        /// \brief create a differential drive robot with custom starting wheel positions
        /// \param wheel_track - distance between the robot's wheels, must be > 0
        /// \param wheel_radius - wheel radius, must be > 0
        /// \param start_wheel_pos - starting position of the wheels
        DiffDrive(double wheel_track, double wheel_radius, const Wheel & start_wheel_pos);
        
        /// \brief create a differential drive robot with a custom starting location
        /// and custom starting wheel position
        /// \param wheel_track - distance between the robot's wheels, must be > 0
        /// \param wheel_radius - wheel radius, must be > 0
        /// \param start_location - starting location transformation
        /// \param start_wheel_pos - starting position of the wheels
        DiffDrive(double wheel_track, double wheel_radius, const Transform2D & start_location,
            const Wheel & start_wheel_pos);

        /// \brief reset current configuration to initial configuration
        void reset();

        /// \brief the current config of the robot
        /// \return the current config structure
        DiffDriveConfig config() const;

        /// \brief use forward kinematics to update the configuration of the robot
        // given new wheel positions
        /// \param new_wheel_pos new wheel positions with which to calculate new configuration
        /// \return the updated config (also stored internally)
        DiffDriveConfig update_config(const Wheel & new_wheel_pos);

        /// \brief use inverse kinematics to get the required wheel velocities to
        /// produce a given body twist
        /// \param twist the requested body twist
        /// \return wheel velocities
        Wheel get_required_wheel_vel(const Twist2D & twist) const;
    };

    /// \brief custom invalid diff drive setup parameters exception 
    class InvalidDiffDriveSetup : public std::exception
    {
    private:
        /// \brief error message
        std::string msg_;
    public:
        /// \brief construct a custom invalid differential drive setup exception message
        /// \param wheel_track 
        /// \param wheel_radius 
        InvalidDiffDriveSetup(double wheel_track, double wheel_radius);

        /// \brief return exception message
        /// \return - exception message pointer
        const char * what() const throw ();
    };
}

#endif