#include <armadillo>
#include "rclcpp/rclcpp.hpp"

class NuSlam : public rclcpp::Node
{
public:
  NuSlam()
  : Node("nuslam")
  {

  }
private:

};

/// \brief Run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSlam>());
  rclcpp::shutdown();
  return 0;
}