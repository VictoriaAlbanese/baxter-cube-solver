////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: baxter_class.hpp
//
// Purpose: Declares a class which represents baxter the robot 
// as a whole and all his joints and sensors
//
////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <cmath>
#include <string>

#include "arm_class.hpp"
#include "color_reader_class.hpp"
#include "face_display_class.hpp"
#include "iks_class.hpp"
#include "square_detector_class.hpp"

#define INITIALIZE 0
#define OVER_CUBE 1
#define CHECK_SQUARE 2
#define FIX_ORIENTATION 3
#define FIX_POSITION 4
#define LOWERING 5
#define PICKUP 6
#define RESET_ARMS 7

#define INSPECT_CUBE 8
#define READ_BOTTOM 8
#define READ_TOP 9
#define SWAP_HANDS 10
#define READ_BACK 11
#define READ_FRONT 12

#define TURN_DEMO 13
#define RIGHT_TURN_CW 13
#define LEFT_TURN_CW 14
#define RIGHT_TURN_CCW 15
#define LEFT_TURN_CCW 16

#define TEARDOWN 17
#define DONE -2
#define INCREMENT -3

#define SAME 0
#define OPPOSITE 1

using std::string;

class Baxter 
{
    private:

        // members
        ros::NodeHandle handle;
        int state;
        bool first;
        bool action_complete;
        int count;
        Arm * holding_arm;
        Arm * other_arm;

        // functions
        void move_on(string message, int new_state); 
        
        void initialize_arms();
        void find_cube(); 
        void check_squares();
        void fix_orientation();
        void fix_position();
        void lower_arm(); 
        void grab_cube(); 
        void reset_arms(int new_state);
        
        void read_bottom(); 
        void read_top();
        void read_back();
        void read_front();
        void swap_hands();
        bool bring_arm_center(Arm * arm);
        bool bring_arm_up(Arm * arm);
        
        bool change_hands();
        void lr_turn(bool side, float direction);
        void increment(bool direction, bool side, float to_here = -1);
        void turning_report(bool side, float direction);

    public:

        // members
        FaceDisplay display;
        Arm left_arm;
        Arm right_arm;
        SquareDetector detector;
        ColorReader reader;

        // functions
        Baxter();
        Baxter(ros::NodeHandle handle);
        bool arms_ready() { return (this->holding_arm->done() && this->other_arm->done()); };
        int get_state() { return this->state; }
        void pickup_cube();
        void inspect_cube();
        void turning_demo();
};

////////////////////////////////////////////////////////////////

