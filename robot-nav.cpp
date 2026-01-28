/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"

static float wrapPi(float a)
{
    while (a > 3.14159265f)  a -= 2.0f * 3.14159265f;
    while (a < -3.14159265f) a += 2.0f * 3.14159265f;
    return a;
}

void Robot::UpdatePose(const Twist& twist)
{
    /**
     * TODO: Add your FK algorithm to update currPose here.
     */

     const float dS = twist.u;
     const float dTh = twist.omega;

     const float Th0 = currPose.theta;

     if (fabs(dTh) < 1e-6f)
     {
        currPose.x += dS * cosf(Th0);
        currPose.y += dS * sinf(Th0);
     } else {
        const float R = dS / dTh;
        currPose.x += R * (sinf(Th0 + dTh) - sinf(Th0));
        currPose.y += -R * (cosf(Th0 + dTh) - cosf(Th0));
        currPose.theta += dTh;
     }
     currPose.theta = wrapPi(currPose.theta);

#ifdef __NAV_DEBUG__
    TeleplotPrint("x", currPose.x);
    TeleplotPrint("y", currPose.y);
    TeleplotPrint("theta", currPose.theta);
#endif

}

/**
 * Sets a destination in the lab frame.
 */
void Robot::SetDestination(const Pose& dest)
{
    /**
     * TODO: Turn on LED, as well.
     */
    Serial.print("Setting dest to: ");
    Serial.print(dest.x);
    Serial.print(", ");
    Serial.print(dest.y);
    Serial.print('\n');

    destPose = dest;
    robotState = ROBOT_DRIVE_TO_POINT;
}

bool Robot::CheckReachedDestination(void)
{
    bool retVal = false;
    /**
     * TODO: Add code to check if you've reached destination here.
     */

    return retVal;
}

void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        /**
         * TODO: Add your IK algorithm here. 
         */

#ifdef __NAV_DEBUG__
        // Print useful stuff here.
#endif

        /**
         * TODO: Call chassis.SetMotorEfforts() to command the motion, based on your calculations above.
         */
    }
}

void Robot::HandleDestination(void)
{
    /**
     * TODO: Stop and change state. Turn off LED.
     */
}
