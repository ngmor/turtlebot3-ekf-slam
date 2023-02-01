# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

## Components
- `rigid2d` - Handles 2D rigid body transformations
- `frame_main` - Perform some rigid body computations based on user input
- `diff_drive` - Models the kinematics of a differential drive robot.

## Conceptual Questions
1. We need to be able to ~normalize~ `Vector2D` objects (i.e., find the unit vector in the direction of a given `Vector2D`):
    - Propose three different designs for implementing the ~normalize~ functionality

    - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

    - Which of the methods would you implement and why?

    1. Make `normalize()` a public member function of `Vector2D` that updates the member varaibles.
        
        Pros:
        
        As per C++ Core Guideline C.4, `normalize()` needs direct access to the representation of a structure (especially if it will modify in-place), so it is a candidate for a member function. This would also give a clear interface that could be replicated on other structures like `Twist2D` - just call the `normalize()` method to change a structure into a normalized structure.

        Cons:
        
        Calling `normalize()` on a `Vector2D` implies the creation of an invariant: that the vector is now a unit vector. There is nothing preserving this invariant for the future, since the members of Vector2D are all public and can be edited independently. It's conceivable that a programmer could call `normalize()` and then expect the vector to represent a unit vector forever after that, even though there is nothing enforcing that invariant. This could lead to bugs. If we wanted to get around this we could follow C++ Core Guidelines C.2 and change `Vector2D` into a class that perhaps has a method for returning a unit vector representation. This might be desirable, but also might introduce more complication than necessary for now.

        Another problem with making this a member function is that it would modify the `Vector2D` object in-place. There would be no option for preserving the original vector with its magnitude, which you might want to do. You could get around this by changing this member function so it doesn't modify in-place, it only returns a normalized `Vector2D`, but that is a bit clunky of an interface.

    2. Make `normalize()` a helper function for `Vector2D` that accepts a reference to a `Vector2D` as an in-out variable.
        
        Pros:

        No copying occurs when passing the `Vector2D` in by reference, which can make the operation less expensive while still modifying the structure in-place.

        Cons:

        Similarly to option 1, this design modifies the object in-place without the option to preserve the original vector. Also C++ Core Guideline F.20 prefers to, by default, return values to output parameters rather than pass by reference.

    3. Make `normalize()` a helper function for `Vector2D` that accepts a const reference `Vector2D` as input and returns a `Vector2D` as an output.

        Pros:

        Now we finally have the option to retain our original vector if desired (`uv = normalize(v)`) or modify our original vector (`v = normalize(v)`), depending on the programmer's intent. Since we are not in-place modifying the const reference that is passed in, we don't actually need direct access to the reperesentation of the structure (so long as we have some way of publicly getting the component values of the vector to do our normalization calculations). Therefore we can follow C++ Core Guideline C.4 and make this not a member function to reduce coupling.

        Since we are passing by const reference, we avoid copying the structure which may help if it grows more expensive (C++ Core Guideline F.16).
        
        We are still placing this helper function in the same namespace as `Vector2D`, which according to C++ Core Guideline C.5 allows argument dependent lookup. If we wanted to also implement the normalize functionality for other structures like `Twist2D`, we could simply overload the function, giving users a clear and consistent interface for normalizing their structures.

        Cons:

        The `normalize()` function isn't quite as strongly associated with the `Vector2D` class now, since users may not know it exists if just looking at documentation for the structure. However, from C++ Core Guidelines C.4 reducing coupling is not a bad thing, and as per C++ Core Guideline C.5 since it's in the same namespace, it is still closely associated.


    For the pros presented above, I went with option 3.


2. What is the difference between a class and a struct in C++?

    The only difference between a class and a struct in C++ is that class members are private by default, while struct members are public by default. Otherwise they have the same functionality.

3. Why is `Vector2D` a struct and `Transform2D` a class (refer to at least 2 specific C++ core guidelines in your answer)?

    According to C++ Core Guideline C.2, a class should be used if a data structure has an invariant, while a struct should be used if all data members can vary independently. `Vector2D` is a great example of good usage of a struct, since the different elements of a vector are not bound to eachother and can vary independently. Depending on its implementation, however, `Transform2D` may have an invariant. If it's stored as a transformation matrix for example, there are constraints on the relationships between the different members of that matrix that must be observed for the matrix to be valid. This is a good use for a class.

    C++ Core Guideline C.8 suggests using a class rather than a struct if any member is non-public. The representation of `Transform2D` is stored privately (as per above, to help preserve invariants), and therefore it should be a class. `Vector2D` has all individually changeable public members, so it can be a struct.

4. Why are some of the constructors in `Transform2D` `explicit` (refer to a specific C++ core guideline in your answer)?

    The `Transform2D` constructors that are `explicit` are all single-argument constructors. The `explicit` keyword prevents the constructor from implicitly converting the type of the input to a different type. Since single argument constructors are overloaded, it's ambiguous whether passing one argument into the constructor would cause a certain constructor to be called, or cause an implicit conversion and then a different constructor to be called. This can't happen if implicit conversions are not allowed.

    C++ Core Guideline C.46 states to use the `explicit` keyword for single-argument constructors to avoid unintended conversions like this.


5. Why is `Transform2D::inv()` declared `const` while `Transform2D::operator*=()` is not?

    C++ Core Guideline Con.2 says to make member functions `const` by default, unless they change the object's observable state. This practice communicates more precise design intent, is more readable, catches more errors in the compiler, and allows more optimization.

    With this in mind, `Transform2D::inv()` returns the inverse of a transformation without modifying the original transformation, so it can be left as `const` for all the benefits above. However, `Transform2D::operator*=()` modifies the transformation in-place, and therefore cannot be left as `const`.

## Collaboration
I worked alone on this library.