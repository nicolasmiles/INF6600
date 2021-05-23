#define S_FUNCTION_NAME ctrlauto
#include "ttkernel.cpp"
#include "mex.h"

#include <time.h>

#define PI 3.141592

#define MAX_STEP_DIST 5000.0
#define NEW_DEST_MAX_DIST_M 50000
#define STATION_MAX_DIST_M 5000
#define PX_TO_M 10

typedef struct
{
    double x;
    double y;
} coord_t;

coord_t wp* = new coord_t;
coord_t powerStation* = new coord_t;
unsigned int speedCons = 50;

/* Generates a step between actual position and the destination*/
/* Returns the simulated time of execution */
double getNextStep(int32_t destX, int32_t destY, 
                 int32_t posCourX, int32_t posCourY,
                 int32_t &stepX, int32_t &stepY)
{
    int nb_operations = 0;
    double d = sqrt((destX-posCourX)*(destX-posCourX) + 
                      (destY-posCourY)*(destY-posCourY));
    nb_operations++;
    if(d <= 100)
    {
        stepX = destX;
        stepY = destY;
        nb_operations+=2;
        return nb_operations / 1000000;
    }
    nb_operations++;
    
    float alpha = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float tmp = (MAX_STEP_DIST/static_cast <float> (d));
    nb_operations+=2;
    if(tmp < 1)
    {
        alpha = alpha * tmp;
        nb_operations++;
    }
    nb_operations++;
    
    if(alpha>0.5)
    {
        alpha = 1 - alpha;
        nb_operations++;
    }
    nb_operations++;

    float beta = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2*d*alpha))) - d*alpha;

    stepX = alpha*(destX-posCourX) + posCourX;

    stepY = alpha*(destY-posCourY) + posCourY;

    stepX += static_cast <float> (beta*(posCourY-destY))/static_cast <float> (d);
    stepY += beta*(destX-posCourX)/d;
    nb_operations += 5;
    return nb_operations / 1000000;
}

/* Generates a new destination */
/* Returns the simulated time of execution */
double gen_dest(coord_t actualPos, coord_t &dest)
{
    int nb_operations = 0;
    float randVal = static_cast<float> (rand()) / static_cast<float>(RAND_MAX);
    dest.x = (int) (actualPos.x + NEW_DEST_MAX_DIST_M*randVal/3);
    randVal = static_cast<float> (rand()) / static_cast<float>(RAND_MAX);
    dest.y = (int) (actualPos.y + NEW_DEST_MAX_DIST_M*randVal/3);
    nb_operations = 20;
    return nb_operations / 1000000;
}

/* Generates a new waypoint between actual position and the destination*/ 
/* Returns the simulated time of execution */
double gen_wp(coord_t actualPos, coord_t dest, coord_t &wp)
{
    int nb_operations = 0;
    
    int32_t stepX;
    int32_t stepY;
    
    mexPrintf("gen_waypoint seg 1\n");
    getNextStep((int)dest.x, (int) dest.y, 
                (int) actualPos.x, (int) actualPos.y,
                stepX, stepY);
    wp.x = (double) stepX;
    wp.y = (double) stepY;
    nb_operations = 20;
    return nb_operations / 1000000;
}

/* Computes the position of the closest station */ 
/* Returns the simulated time of execution */
double getClosestStation(coord_t actualPos, coord_t &stationPos)
{
    mexPrintf("data_ptr->actualPos.x: %f\n", actualPos.x);
    mexPrintf("data_ptr->actualPos.y: %f\n", actualPos.y);
    
    int nb_operations = 0;
    float randVal = static_cast<float> (rand()) / static_cast<float>(RAND_MAX);
    
    stationPos.x = actualPos.x + STATION_MAX_DIST_M*randVal/PX_TO_M/3;
    randVal = static_cast<float> (rand()) / static_cast<float>(RAND_MAX);
    stationPos.y = actualPos.y + STATION_MAX_DIST_M*randVal/PX_TO_M/3;
    nb_operations = 24;
    mexPrintf("Closest station: (%f, %f)\n", stationPos.x, stationPos.y);
    return nb_operations / 1000000;
}

/********************* A COMPLETER ************************************/


double wait()//plus beau à faire? eventtrigger sur le timer?sleep?
{
    int begin = sys.time();
    while(begin+60*1000000<sys.time());
    return 2/1000000;
}

double computeOrientation()
{
    double orientation = 0;
    double orientation += arctan((actualPos.x - wp->x)/(actualPos.y - wp->y))*180/PI;
    if(wp.y - actualPos.y <0)
    {
        orientation += 180;
        return 4/1000000;
    }
    return 3/1000000;
}

double norm(int x1, int y1, int x2, int y2)
{
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

double checkPos()
{
    if(norm(actualPos.x, actualPos.y, lastPos->x, lastPos->y)>=9)
    {
        ttCreateJob("takePic");
        lastPos = actualPos;
    }
    if(norm(actualPos.x,actualPos.y, wp->x, wp->y)<=10)
    {
        ttCreateJob("updateStep");
        return 2/1000000;
    }
    return 1/1000000;
}

double updateStep()
{
    double nbOp = 0;
    if(wp->x == powerStation.x && wp->y == powerStation.y)
    {
        speedCons = 0;
        //trigger event charger la batterie
        return 3/1000000;
    }
    if(wp->x == dest.x && wp->y == dest.y)
    {
        speedCons = 0;
        //declancher timer
        nbOp += gen_dest(actualPos, dest)+1/1000000;
        speedCons = 50;
    }
    return gen_wp(actualPos, wp) + nbOp + 2/1000000;
}
double takePic()
{
    picCommand = true;
    //démarrer l'analyse
    return 1/1000000;
}

double checkPic()
{
  //  picCommande = !successAnalysis;
    return 1/1000000;
}

double activate_battery_saver()
{
    double nbOp += getClosestStation(actualPos, powerStation);
    wp = powerStation;
    speedCons = 30;
    return nbOp + 2/1000000;
}

double exit_charge_station()
{
    gen_wp(actualPos, wp);
    speedCons = 50;
    return 1/1000000;
}

// Data structure used for the task data
struct TaskData 
{
    double exectime;
    //TODO
};

// Kernel init function    
void init()
{
    TaskData *data = new TaskData;
    ttSetUserData(data);
    data->exectime = 0.1;
    ttInitKernel("prioFP");
    ttCreatePeriodicTask("checkPos", 0, 0.1, checkPos, data);
    ttCreatePeriodicTask("computeOrientation", 0,1,computeOrientation, data);
    ttCreateTask("takePic", 0.5, takePic, data);
    ttCreateTask("updateStep", 70, updateStep, data);
    //TODO
    ttCreateHandler("battery_10pc", 1, activate_battery_saver, data);
    ttCreateHandler("battery_80pc", 1, exit_charge_station, data);
            
//    ttAttachTriggerHandler(TRIGGER_BAT_10PC, "battery_10pc");
  //  ttAttachTriggerHandler(TRIGGER_BAT_80PC, "battery_80pc");
            
}

// Kernel cleanup function
void cleanup() 
{
    //TODO
    delete(TaskData*)ttGetUserData();
}
