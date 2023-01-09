#include <iostream>
#include "turtlelib/rigid2d.hpp"
using std::cout;
using std::cin;
using namespace turtlelib; //TODO - good practice?

int main () {
    cout << "Hello World!" << '\n';

    /*
    //Vector2D
    Vector2D v_a;
    int test;

    cout << "Enter v_a and an int: " << '\n';
    cin >> v_a >> test;
    cout << "v_a: " << v_a << '\n';
    cout << "int: " << test << '\n';
    */

    //Transform2D
    Transform2D tf {Vector2D{1,2},PI/2};

    cout << tf << '\n';
    cin >> tf;
    cout << tf << '\n';
}