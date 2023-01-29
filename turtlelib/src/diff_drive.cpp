#include "turtlelib/diff_drive.hpp"
#include <string_view>

namespace turtlelib
{
    /* DIFF DRIVE START */

    DiffDrive::DiffDrive(double wheel_track, double wheel_radius)
    : DiffDrive(
        wheel_track,
        wheel_radius,
        Transform2D {},     //start_location
        0.0,                //start_position_left_wheel
        0.0                 //start_position_right_wheel
    ) {}

    DiffDrive::DiffDrive(double wheel_track, double wheel_radius, Transform2D start_location)
    : DiffDrive(
        wheel_track,
        wheel_radius,
        start_location,
        0.0,                //start_position_left_wheel
        0.0                 //start_position_right_wheel
    ) {}

    DiffDrive::DiffDrive(double wheel_track, double wheel_radius,
        double start_position_left_wheel, double start_position_right_wheel)
    : DiffDrive(
        wheel_track,
        wheel_radius,
        Transform2D {},     //start_location
        start_position_left_wheel,
        start_position_right_wheel
    ) {}

    DiffDrive::DiffDrive(
        double wheel_track,
        double wheel_radius,
        Transform2D start_location,
        double start_position_left_wheel,
        double start_position_right_wheel
    )
    : wheel_track_{wheel_track},
      wheel_radius_{wheel_radius},
      config_init_{
        start_location,
        start_position_left_wheel,
        start_position_right_wheel
      },
      config_{config_init_}
    {
        // Throw exception if robot parameters are not valid
        if ((wheel_track_ <= 0) || (wheel_radius_ <= 0)) {
            throw InvalidDiffDriveSetup(wheel_track_, wheel_radius_);
        }
    }

    void DiffDrive::reset() {
        config_ = config_init_;
    }

    /* DIFF DRIVE END */

    InvalidDiffDriveSetup::InvalidDiffDriveSetup(double wheel_track, double wheel_radius)
    : msg_{
        "Invalid differential drive robot dimensions, must be greater than 0:\nWheel Track: " +
        std::to_string(wheel_track) + "\nWheel Radius: " + std::to_string(wheel_radius)
    } {}

    const char * InvalidDiffDriveSetup::what() const throw () {return msg_.c_str();}
}