#include "robot.h"
#include <Arduino.h>

static const uint8_t BUTTON_B_PIN = PD5; 

void Robot::InitializeRobot(void)
{
    pinMode(BUTTON_B_PIN, INPUT_PULLUP);
    chassis.InititalizeChassis();

    /**
     * TODO: Set pin 13 HIGH when navigating and LOW when destination is reached.
     * Need to set as OUTPUT here.
     */
}

void Robot::EnterIdleState(void)
{
    chassis.Stop();

    Serial.println("-> IDLE");
    robotState = ROBOT_IDLE;
}

void Robot::DriveForwardCm(float distance_cm)
{
    Pose dest;
    dest.x = currPose.x + distance_cm * cosf(currPose.theta);
    dest.y = currPose.y + distance_cm * sinf(currPose.theta);
    dest.theta = currPose.theta;
    SetDestination(dest);
}

/**
 * The main loop for your robot. Process both synchronous events (motor control),
 * and asynchronous events (distance readings, etc.).
*/
void Robot::RobotLoop(void) 
{
     /**
     * Run the chassis loop, which handles low-level control.
     */
    Twist velocity;
    if(chassis.ChassisLoop(velocity))
    {
        // We do FK regardless of state
        UpdatePose(velocity);
        
        static bool prevB = false;
        bool bNow = (digitalRead(BUTTON_B_PIN));  

        if (bNow && !prevB) { // rising edge (new press)
            if (robotState == ROBOT_IDLE) {
                DriveForwardCm(60.0f);
    }
}
prevB = bNow;

        /**
         * Here, we break with tradition and only call these functions if we're in the 
         * DRIVE_TO_POINT state. CheckReachedDestination() is expensive, so we don't want
         * to do all the maths when we don't need to.
         * 
         * While we're at it, we'll toss DriveToPoint() in, as well.
         */ 
        if(robotState == ROBOT_DRIVE_TO_POINT)
        {
            DriveToPoint();
            if(CheckReachedDestination()) HandleDestination();
        }
    }
}
