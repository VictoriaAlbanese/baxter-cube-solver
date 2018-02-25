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

#define INSPECT_CUBE 7 
#define READ_BOTTOM 7
#define READ_TOP 8
#define READ_BACK 9
#define READ_FRONT 10

#define TURN_DEMO 11
#define RIGHT_TURN_CW 11
#define LEFT_TURN_CW 12
#define RIGHT_TURN_CCW 13
#define LEFT_TURN_CCW 14

#define TEARDOWN -1
#define DONE -2
#define INCREMENT -3

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
        void reset_arms();
        
        void read_bottom(); 
        void read_top();
        void read_back();
        void read_front();
        bool bring_arms_center();
        
        bool change_hands();
        void lr_turn(bool side, float direction);
        void increment(bool direction, bool do_it = false);
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

