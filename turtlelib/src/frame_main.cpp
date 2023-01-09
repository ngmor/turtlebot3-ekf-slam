#include <iostream>
#include "turtlelib/rigid2d.hpp"
using std::cout;
using std::cin;
using namespace turtlelib; //TODO - good practice?

int main () {

    //Transform2D
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
    
    //Vector2D
    Vector2D v_b;

    std::cout << "Enter vector v_b:\n";
    std::cin >> v_b;
    std::cout << "v_bhat: " << normalize(v_b) << '\n'
              << "v_a: " << t_ab(v_b) << '\n'
              << "v_b: " << v_b << '\n'
              << "v_c: " << (t_bc.inv())(v_b) << '\n';

    //Twist2D
    Twist2D V_b;

    std::cout << "Enter twist V_b:\n";
    std::cin >> V_b;
    std::cout << "V_a " << t_ab(V_b) << '\n'
              << "V_b " << V_b << '\n'
              << "V_c " << (t_bc.inv())(V_b) << '\n';
}