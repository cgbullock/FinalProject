#ifndef BEHAVIORS
#define BEHAVIORS
 
#include <Romi32U4.h>
 
class Behaviors{
    private:
        bool test = true;
        int threshold = 600;
        int threshold_pick_up = 1500;
        int data[3] = {0};
        enum ROBOT_STATE {IDLE, DRIVE, OBJECT, SONAR1, SONAR2};
        ROBOT_STATE robot_state = IDLE; //initial state: IDLE
         
    public:
        void Init(void);
        void Stop(void);
        void Run(void);
        boolean DetectCollision(void);
        boolean DetectBeingPickedUp(void);
};
 
#endif