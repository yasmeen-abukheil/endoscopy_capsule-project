/*
.
*/
#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <cassert>


//#include "helper.h"

#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

#include <sensor_msgs/JointState.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include <pthread.h>


static double sphereRadius = 5.0;

/* Charge (positive/negative) */
int charge = 1;


hduVector3Dd forceField(hduVector3Dd pos);

static HHD ghHD = HD_INVALID_HANDLE;
static HDSchedulerHandle gSchedulerCallback = HD_INVALID_HANDLE;
//*******************************************************************************
 //Given the position is space, calculates the (modified) coulomb force.
//*******************************************************************************/
hduVector3Dd forceField(hduVector3Dd pos)
{
    double dist = pos.magnitude();

    hduVector3Dd forceVec(0,0,0);
   fprintf(stderr, "%f\ndist\n", dist);

    // if two charges overlap...
    if(dist < sphereRadius*2.0)
    {
        // Attract the charge to the center of the sphere.
        forceVec =  -10*pos;
        fprintf(stderr, "%f\nforce\n", forceVec);
    }
    else
    {
        hduVector3Dd unitPos = normalize(pos);
        fprintf(stderr, "%f\nnormalize(pos)\n", unitPos);

        forceVec = -1200.0*unitPos/(dist*dist);
    }
    forceVec *= charge;
    return forceVec;
}

/******************************************************************************
 This handler gets called when the process is exiting. Ensures that HDAPI is
 properly shutdown
******************************************************************************/
void exitHandler()
{
    hdStopScheduler();
    hdUnschedule(gSchedulerCallback);

    if (ghHD != HD_INVALID_HANDLE)
    {
        hdDisableDevice(ghHD);
        ghHD = HD_INVALID_HANDLE;
    }
}

HDCallbackCode HDCALLBACK CoulombCallback(void *data)
{
    HHD hHD = hdGetCurrentDevice();

    hdBeginFrame(hHD);

    hduVector3Dd pos;
    hdGetDoublev(HD_CURRENT_POSITION,pos);
    fprintf(stderr, "%f\npos[1]\n", pos[0]);
    fprintf(stderr, "%f\npos[2]\n", pos[1]);
    fprintf(stderr, "%f\npos[3]\n", pos[2]);

    hduVector3Dd forceVec;
   forceVec = forceField(pos);
    hdSetDoublev(HD_CURRENT_FORCE, forceVec);

    hdEndFrame(hHD);

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error during scheduler callback");
        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }

    return HD_CALLBACK_CONTINUE;
}

void CoulombForceField()
{

    std::cout << "haptics callback" << std::endl;
    gSchedulerCallback = hdScheduleAsynchronous(
        CoulombCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }

    std::cout << "graphics callback" << std::endl;

    while(1) ;
   // glutMainLoop(); // Enter GLUT main loop.
}

int main(int argc, char** argv)
{

    HDErrorInfo error;

    printf("Starting application\n");

    atexit(exitHandler);

    // Initialize the device.  This needs to be called before any other
    // actions on the device are performed.
    ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }

    printf("Found device %s\n",hdGetString(HD_DEVICE_MODEL_TYPE));

    hdEnable(HD_FORCE_OUTPUT);
    hdEnable(HD_MAX_FORCE_CLAMPING);

    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }

    ros::init(argc, argv, "omni_haptic_node");




    while (ros::ok()) {
        CoulombForceField();
    }



    ROS_INFO("Ending Session....");



    return 0;




}//main

