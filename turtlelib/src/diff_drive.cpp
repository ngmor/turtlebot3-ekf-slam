#include "turtlelib/diff_drive.hpp"
#include <string_view>

namespace turtlelib
{
    /* DIFF DRIVE START */

    void DiffDrive::calc_kinematic_coeff()
    {
        coeff_ik_w_ = wheel_track_ / (2.0*wheel_radius_);
        coeff_ik_x_ = 1.0 / (wheel_radius_);
        coeff_fk_w_ = wheel_radius_ / wheel_track_;
        coeff_fk_x_ = wheel_radius_ / 2.0;
    }

    DiffDrive::DiffDrive(double wheel_track, double wheel_radius)
    : DiffDrive(
        wheel_track,
        wheel_radius,
        Transform2D {},     //start_location
        Wheel {0,0}         //start_wheel_pos
    ) {}

    DiffDrive::DiffDrive(double wheel_track, double wheel_radius, 
        const Transform2D & start_location)
    : DiffDrive(
        wheel_track,
        wheel_radius,
        start_location,
        Wheel {0,0}         //start_wheel_pos
    ) {}

    DiffDrive::DiffDrive(double wheel_track, double wheel_radius, 
        const Wheel & start_wheel_pos)
    : DiffDrive(
        wheel_track,
        wheel_radius,
        Transform2D {},     //start_location
        start_wheel_pos
    ) {}

    DiffDrive::DiffDrive(
        double wheel_track,
        double wheel_radius,
        const Transform2D & start_location,
        const Wheel & start_wheel_pos
    )
    : wheel_track_{wheel_track},
      wheel_radius_{wheel_radius},
      config_init_{
        start_location,
        start_wheel_pos
      },
      config_{config_init_}
    {
        // Throw exception if robot parameters are not valid
        if ((wheel_track_ <= 0) || (wheel_radius_ <= 0)) {
            throw InvalidDiffDriveSetup(wheel_track_, wheel_radius_);
        }

        calc_kinematic_coeff();
    }

    void DiffDrive::reset() {
        config_ = config_init_;
    }

    DiffDriveConfig DiffDrive::config() const {return config_;}

    DiffDriveConfig DiffDrive::update_config(const Wheel & new_wheel_pos) {
        //Calculate change in wheel position
        Wheel wheel_delta {
            new_wheel_pos.left - config_.wheel_pos.left,    //left
            new_wheel_pos.right - config_.wheel_pos.right   //right
        };

        //Calculate produced body twist
        Twist2D body_twist {
            coeff_fk_w_*(-wheel_delta.left + wheel_delta.right),    //w
            coeff_fk_x_*(wheel_delta.left + wheel_delta.right),     //x
            0.0                                                     //y
        };

        //TODO integrate twist

        //TODO Calculate new config

        //TODO - update wheel positions in config
        return config();
    }

    Wheel DiffDrive::get_required_wheel_vel(const Twist2D & twist) const
    {
        //TODO throw error if twist y velocity is nonzero

        return Wheel {
            -coeff_ik_w_*twist.w + coeff_ik_x_*twist.x,     //left
            coeff_ik_w_*twist.w + coeff_ik_x_*twist.x       //right
        };
    }

    /* DIFF DRIVE END */

    InvalidDiffDriveSetup::InvalidDiffDriveSetup(double wheel_track, double wheel_radius)
    : msg_{
        "Invalid differential drive robot dimensions, must be greater than 0:\nWheel Track: " +
        std::to_string(wheel_track) + "\nWheel Radius: " + std::to_string(wheel_radius)
    } {}

    const char * InvalidDiffDriveSetup::what() const throw () {return msg_.c_str();}
}