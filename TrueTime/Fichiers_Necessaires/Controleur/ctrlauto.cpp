#define S_FUNCTION_NAME ctrlauto
#include "ttkernel.cpp"
#include "mex.h"

#include <time.h>
#include <math.h>

#define PI 3.141592

#define MAX_STEP_DIST 5000.0
#define NEW_DEST_MAX_DIST_M 50000
#define STATION_MAX_DIST_M 5000
#define PX_TO_M 10

#define TRIGGER_BAT_10PC 1
#define TRIGGER_BAT_80PC 2

#define BATTERY_LEVEL 1
#define ANALYSE_CHECK 2
#define POS_X 3
#define POS_Y 4
#define REAL_ORIENTATION 5
#define REAL_SPEED

#define CHARGE 1
#define ANALYSE 2
#define ORIENTATION 3
#define SPEED 4

#define VITESSE_MAX_KMH 50;
#define VITESSE_RED_KMH 30;


typedef struct
{
    double x;
    double y;
} coord_t;

struct TaskData
{
    double exectime;
    coord_t wp;
    coord_t powerStation;
    coord_t lastPos;
    coord_t actualPos;
    coord_t dest;
    unsigned int speedCons;
    double orientation;
    bool waiting;
    //TODO
};

double nbOp;

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
/* si l'orientation se fait séparément, pas la solution retenue
double computeOrientation(int seg, void* data)
{
    switch(seg)
    {
        case 1:
            ((TaskData*)data)->orientation = atan((((TaskData*)data)->actualPos.x - ((TaskData*)data)->wp.x)/(((TaskData*)data)->actualPos.y - ((TaskData*)data)->wp.y))*180/PI;
            if(((TaskData*)data)->wp.y - ((TaskData*)data)->actualPos.y <0)
            {
                ((TaskData*)data)->orientation += 180;
                return 3/1000000;
            }
            return 2/1000000;
        case 2:
            ttAnalogOut(ORIENTATION, ((TaskData*)data)->orientation);
       //     return 1/1000;
            return -1;
        default:
            return -1;
    }
}*/
/*
double updateSpeed(int seg, void * data)
{
    switch(seg)
    {
        case 1:
            ttAnalogOut(SPEED,((TaskData*)data)->speedCons);
        //    return 1/1000;
            return -1;
        default:
            return -1;
    }
}*/
double norm(int x1, int y1, int x2, int y2)
{
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

double updatePos(int seg, void* data)
{
    switch (seg)
    {
        case 1:
            if(((TaskData*)data)->waiting)
            {
                return -1;
            }
            return 1/1000000;
        case 2:
            ((TaskData*)data)->actualPos.x = ttAnalogIn(POS_X);
            ((TaskData*)data)->actualPos.y = ttAnalogIn(POS_Y);
            return 2/1000;
        case 3:
            if(norm(((TaskData*)data)->actualPos.x,((TaskData*)data)->actualPos.y, ((TaskData*)data)->wp.x, ((TaskData*)data)->wp.y)<=10)
            {
                if(((TaskData*)data)->wp.x == ((TaskData*)data)->dest.x && ((TaskData*)data)->wp.y == ((TaskData*)data)->dest.y)
                {
                    ((TaskData*)data)->waiting = true;
                    ((TaskData*)data)->speedCons = 0;
               //     ttCreateJob("updateSpeed"); //si on fait une tache pour la vitesse
                    ttCreateTimer("newDest",ttCurrentTime() + 60,"newDest");
                    return 5/1000000;
                }
                else
                {
                    ttCreateJob("updateStep");
                    return 2/1000000;
                }
            }
            return 1/1000000;
      /*  case 4:
            ttCreateJob("computeOrientation");
        //    return 1/100000;
            return -1;*/
        case 4:
            ((TaskData*)data)->orientation = atan((((TaskData*)data)->actualPos.x - ((TaskData*)data)->wp.x)/(((TaskData*)data)->actualPos.y - ((TaskData*)data)->wp.y))*180/PI;
            if(((TaskData*)data)->wp.y - ((TaskData*)data)->actualPos.y <0)
            {
                ((TaskData*)data)->orientation += 180;
                return 3/1000000;
            }
            return 2/1000000;
        default:
            ttAnalogOut(ORIENTATION, ((TaskData*)data)->orientation);
            ttAnalogOut(SPEED,((TaskData*)data)->speedCons);
            return -1;
    }
}

double updateStep(int seg, void* data)
{
    switch (seg)
    {
        case 1:
            if(((TaskData*)data)->wp.x == ((TaskData*)data)->powerStation.x && ((TaskData*)data)->wp.y == ((TaskData*)data)->powerStation.y)
            {
                ((TaskData*)data)->speedCons = 0;
          //      ttCreateJob("updateSpeed");
                ttAnalogOut(SPEED,((TaskData*)data)->speedCons);
                ttAnalogOut(CHARGE, 1);
                return -1;
            }
            return 1/1000000;
        case 2:
           /* return*/ gen_wp(((TaskData*)data)->actualPos,((TaskData*)data)->dest, ((TaskData*)data)->wp);
            return -1;
        default:
            return -1;

    }
}

double newDest(int seg, void* data)
{
    switch(seg)
    {
        case 1:
            nbOp = gen_dest(((TaskData*)data)->actualPos, ((TaskData*)data)->dest);
            ((TaskData*)data)->speedCons = VITESSE_MAX_KMH;
        //    ttCreateJob("updateSpeed");
            nbOp+= gen_wp(((TaskData*)data)->actualPos,((TaskData*)data)->dest, ((TaskData*)data)->wp);
            ((TaskData*)data)->waiting = false;
            return nbOp + 2/1000000;
        default:
            ttAnalogOut(SPEED,((TaskData*)data)->speedCons);
            return -1;
    }
}
double cameraControler(int seg, void* data)
{
    switch (seg)
    {
        case 1:
            if(ttAnalogIn(ANALYSE_CHECK) != 1)
            {       
                ttAnalogOut(ANALYSE, 0);
                return -1;
            }
            return 1/1000;
        case 2:
            if(((TaskData*)data)->waiting)
            {
                return -1;
            }
            return 1/1000000;
        case 3:
            ((TaskData*)data)->actualPos.x = ttAnalogIn(POS_X);
            ((TaskData*)data)->actualPos.y = ttAnalogIn(POS_Y);
            return 2/100;
        case 4:
            if(norm(((TaskData*)data)->actualPos.x, ((TaskData*)data)->actualPos.y, ((TaskData*)data)->lastPos.x, ((TaskData*)data)->lastPos.y)<9)
            {
                return -1;
            }
      //      mexPrintf("Prise de photo\n");
            ((TaskData*)data)->lastPos = ((TaskData*)data)->actualPos;
            return 2/1000000;
        default:
            ttAnalogOut(ANALYSE, 1);
            return -1;
    }
}

double activate_battery_saver(int seg, void* data)
{
    switch (seg)
    {
        case 1:
            nbOp = getClosestStation(((TaskData*)data)->actualPos, ((TaskData*)data)->powerStation);
            ((TaskData*)data)->wp = ((TaskData*)data)->powerStation;
            ((TaskData*)data)->speedCons = VITESSE_RED_KMH;
       //     ttCreateJob("updateSpeed");
            return nbOp+2/1000000;
        default:
            ttAnalogOut(SPEED,((TaskData*)data)->speedCons);//peut etre pas nécessaire?
            return -1;
    }
}

double exit_charge_station(int seg, void* data)
{
    switch(seg)
    {
        case 1:
            nbOp = gen_wp(((TaskData*)data)->actualPos,((TaskData*)data)->dest, ((TaskData*)data)->wp);
            ((TaskData*)data)->speedCons = VITESSE_MAX_KMH;
       //     ttCreateJob("updateSpeed");
            return nbOp + 1/1000000;
        default:
            ttAnalogOut(CHARGE,0);
            ttAnalogOut(SPEED,((TaskData*)data)->speedCons);//peut etre pas necessaire?
            return -1;
    }
}

// Data structure used for the task data

// Kernel init function
void init()
{
    TaskData *data = new TaskData;
    ttSetUserData(data);
    data->exectime = 0.1;
    data->lastPos.x = 1;
    data->lastPos.y = 1;
    data->actualPos.x = 1;
    data->actualPos.y = 1;
    data->wp.x = 1;
    data->wp.y = 1;
    data->waiting = false;
    gen_dest(data->actualPos, data->dest);
  //  gen_wp(data->actualPos, data->dest, data->wp);
    data->speedCons = VITESSE_MAX_KMH;
    ttInitKernel(prioFP);
    ttCreatePeriodicTask("updatePos", 0, 0.1, updatePos, data);
    ttSetPriority(2,"updatePos");
 //   ttCreateTask("updateSpeed", 1, updateSpeed, data);
  //  ttSetPriority(1,"updateSpeed");
  //  ttCreateTask("computeOrientation", 1,computeOrientation, data);
  //  ttSetPriority(1,"computeOrientation");
    ttCreatePeriodicTask("cameraControler", 0,0.1,cameraControler, data);
    ttSetPriority(10,"cameraControler");
    ttCreateTask("updateStep", 1,updateStep, data);
    ttSetPriority(5,"updateStep");
    //TODO
    ttCreateHandler("battery_10pc", 15, activate_battery_saver, data);
    ttCreateHandler("battery_80pc", 1, exit_charge_station, data);
    
    ttAttachTriggerHandler(TRIGGER_BAT_10PC, "battery_10pc");
    ttAttachTriggerHandler(TRIGGER_BAT_80PC, "battery_80pc"); 
    ttCreateHandler("newDest",1,newDest,data);
//    ttCurrentTime(0);
   // ttCreateJob("updateSpeed");
}

// Kernel cleanup function
void cleanup()
{
    //TODO
    delete (TaskData*)ttGetUserData();
}
