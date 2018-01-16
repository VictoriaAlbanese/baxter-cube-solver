////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: baxter_class.cpp
//
// Purpose: Declares a class which represents baxter the robot 
// as a whole and all his joints and sensors
//
////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <string>
#include <cmath>
#include "arm_class.hpp"
#include "iks_class.hpp"
#include "square_detector_class.hpp"
#include "face_display_class.hpp"

#define INITIALIZE 0
#define OVER_CUBE 1
#define CHECK_SQUARE 2
#define FIX_ORIENTATION 3
#define FIX_POSITION 4
#define LOWERING 5
#define PICKUP 6
#define TEARDOWN 7 
#define DONE 8

using std::string;

class Baxter 
{
    private:

        // members
        ros::NodeHandle handle;
        int state;
        bool first;
        bool count;

        // functions
        void move_on(string message, int new_state); 
        void initialize_arms();
        void find_cube(); 
        void check_squares();
        void fix_orientation();
        void fix_position();
        void lower_arm(); 
        void grab_cube(); 
        void reset_arms();
       
    public:

        // members
        FaceDisplay display;
        Arm left_arm;
        Arm right_arm;
        IKS ik_solver;
        SquareDetector detector;

        // functions
        Baxter();
        Baxter(ros::NodeHandle handle);
        bool arms_ready() { return (this->left_arm.done() && this->right_arm.done()); };
        int get_state() { return this->state; }
        void pickup_cube();
};

////////////////////////////////////////////////////////////////

