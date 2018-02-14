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
#include "color_reader_class.hpp"

#define R_AWAY (0.0)
#define R_SOFT_HOLD (0.03)
#define R_FIRM_HOLD (0.05)
#define L_AWAY (0.0)
#define L_SOFT_HOLD (-0.01)
#define L_FIRM_HOLD (-0.04)

#define INITIALIZE 0
#define OVER_CUBE 1
#define CHECK_SQUARE 2
#define FIX_ORIENTATION 3
#define FIX_POSITION 4
#define LOWERING 5
#define PICKUP 6
#define INSPECT_CUBE 7 
#define READ_BOTTOM 7
#define READ_TOP 8
#define READ_BACK 9
#define READ_FRONT 10
#define TEARDOWN 11
#define DONE 12

using std::string;

class Baxter 
{
    private:

        // members
        ros::NodeHandle handle;
        int state;
        bool first;
        int count;

        // functions
        void move_on(string message, int new_state); 
        void initialize_arms();
        void find_cube(); 
        void check_squares();
        void fix_orientation();
        void fix_position();
        void lower_arm(); 
        void grab_cube(); 
        void read_bottom(); 
        void read_top();
        void read_back();
        void read_front();
        void reset_arms();

    public:

        // members
        FaceDisplay display;
        Arm left_arm;
        Arm right_arm;
        IKS left_iks;
        IKS right_iks;
        SquareDetector detector;
        ColorReader reader;

        // functions
        Baxter();
        Baxter(ros::NodeHandle handle);
        bool arms_ready() { return (this->left_arm.done() && this->right_arm.done()); };
        int get_state() { return this->state; }
        void pickup_cube();
        void inspect_cube();
};

////////////////////////////////////////////////////////////////

