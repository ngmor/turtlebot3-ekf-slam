#include "turtlelib/diff_drive.hpp"
#include <string_view>

namespace turtlelib
{
    DiffDrive::DiffDrive(double wheel_track, double wheel_radius)
    : wheel_track_{wheel_track}, wheel_radius_{wheel_radius}
    {
        if ((wheel_track_ <= 0) || (wheel_radius_ <= 0)) {
            throw InvalidDiffDriveSetup(wheel_track_, wheel_radius_);
        }
    }

    InvalidDiffDriveSetup::InvalidDiffDriveSetup(double wheel_track, double wheel_radius)
    : msg_{
        "Invalid differential drive robot dimensions, must be greater than 0:\nWheel Track: " +
        std::to_string(wheel_track) + "\nWheel Radius: " + std::to_string(wheel_radius)
    } {}

    const char * InvalidDiffDriveSetup::what() const throw () {return msg_.c_str();}
}