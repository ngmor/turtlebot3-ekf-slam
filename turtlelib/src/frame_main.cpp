#include <iostream>
#include "turtlelib/rigid2d.hpp"
using std::cout;
using std::cin;
using namespace turtlelib; //TODO - good practice?

int main () {

    Transform2D t_ab, t_bc;

    std::cout << "Enter transform T_{a,b}:\n";
    std::cin >> t_ab;
    std::cout << "Enter transform T_{b,c}:\n";
    std::cin >> t_bc;
    std::cout << "T_{a,b}: " << t_ab << '\n'
              << "T_{b,a}: " << t_ab.inv() << '\n'
              << "T_{b,c}: " << t_bc << '\n'
              << "T_{c,b}: " << t_bc.inv() << '\n'
              << "T_{a,c}: " << t_ab*t_bc << '\n'
              << "T_{c,a}: " << (t_ab*t_bc).inv() << '\n';

}