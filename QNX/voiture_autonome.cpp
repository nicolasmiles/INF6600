#include "genMap.h"
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <mqueue.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/syspage.h>
#include <sys/neutrino.h>
using namespace std;
#include <iostream>

#define CTRL_CAM_PRIO 	1
#define CTRL_DEST_PRIO 	2
#define CTRL_NAV_PRIO 	3
#define CAM_PRIO 		4
#define BAT_PRIO 		5
#define SPEED_PRIO 		6
#define ORIENT_PRIO 	7
#define NAV_PRIO		8
#define BAT_10_PER_PRIO	9
#define BAT_80_PER_PRIO	10
#define AFFICHAGE_PRIO	1

#define MAX_NUM_MSG 	50

#define MSG_PRIO		1

#define MQ_CAM 			"File_Camera"
#define MQ_BAT 			"File_Batterie"
#define MQ_NAV 			"File_Navigation"

#define PI				3.14
mqd_t msgQIdCam, msgQIdBat, msgQIdNav ;


unsigned int speed;
unsigned int acceleration = 1;
int realSpeed;
double orientation;
double realOrientation;
double w = 0.01;
coord_t actualPos;
coord_t lastPos;
coord_t wp;
coord_t dest;
coord_t station;
bool stop;
bool charge;
double battery;

pthread_mutex_t speedMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t orientationMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t realSpeedMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t realOrientationMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t actualPosMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t lastPosMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t wpMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t destMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t stationMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t stopMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t chargeMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t batteryMutex = PTHREAD_MUTEX_INITIALIZER;

sem_t alert_10_sem, alert_80_sem, photo_sem, wp_sem;

double distance(coord_t p1, coord_t p2)
{
	return abs(sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y)));
}

void *Tnavigation(void *arg)
{
	while(1)
	{
		double period = 0.1;
		pthread_mutex_lock(&realSpeedMutex);
		pthread_mutex_lock(&realOrientationMutex);
		pthread_mutex_lock(&actualPosMutex);
		actualPos.x += realSpeed *sin(realOrientation)*period;
		actualPos.y += realSpeed *cos(realOrientation)*period;
		pthread_mutex_unlock(&realSpeedMutex);
		pthread_mutex_unlock(&realOrientationMutex);
		pthread_mutex_lock(&wpMutex);
		if (distance(actualPos, wp) < 10)
		{
			pthread_mutex_lock(&destMutex);
			if (wp == dest)
			{
				//stop the car
				pthread_mutex_lock(&speedMutex);
				speed = 0;
				pthread_mutex_unlock(&speedMutex);
				//wait 1 min /!\ 
				genDest(actualPos, &dest);
			}
			pthread_mutex_unlock(&destMutex);
			sem_post(&wp_sem);
		}
		pthread_mutex_unlock(&wpMutex);
		pthread_mutex_lock(&lastPosMutex);
		if (distance(actualPos, lastPos) < 1)
		{
			sem_post(&photo_sem);
			lastPos = actualPos;
		}
		pthread_mutex_unlock(&lastPosMutex);
		pthread_mutex_unlock(&actualPosMutex);
		sleep(period);
	}
	return NULL;
}

void *Tspeed(void *arg)
{
	while (1)
	{
		double period = 0.1;
		pthread_mutex_lock(&realSpeedMutex);
		pthread_mutex_lock(&speedMutex);
		if (realSpeed > speed)
		{
			realSpeed -= acceleration;
			if (realSpeed < 0)
				realSpeed = 0;
		}
		else if (realSpeed < speed)
		{
			realSpeed += acceleration;
		}
		pthread_mutex_unlock(&realSpeedMutex);
		pthread_mutex_unlock(&speedMutex);
		sleep(period);
	}
}

void *Torientation(void *arg)
{
	while (1)
	{
		double period = 0.1;
		pthread_mutex_lock(&realOrientationMutex);
		pthread_mutex_lock(&orientationMutex);
		if (realOrientation - orientation > PI)
		{
			realOrientation -= w;
		}
		else if (realOrientation != orientation)
		{
			realOrientation += w;
		}
		realOrientation %= 2 * PI;
		pthread_mutex_unlock(&realOrientationMutex);
		pthread_mutex_unlock(&orientationMutex);
		sleep(period); //utiliser des timers à la place /!\
	}
}


void *Talimentation(void *arg)
{
	while(1)
	{
		double period = 0.1;
		double consumption = 0.004;
		double loading = 0.4;
		double evol;
		pthread_mutex_lock(&chargeMutex);
		if(charge)
			evol = loading;
		else
		{
			pthread_mutex_lock(&speedMutex);
			evol = -consumption*speed;
			pthread_mutex_unlock(&speedMutex);
		}
		pthread_mutex_lock(&batteryMutex);
		battery += evol*period;
		if(battery>=80 && charge)
			sem_post(&alert_80_sem);
		else if(battery<=80 && !charge)
			sem_post(&alert_10_sem);//rewait?
		pthread_mutex_unlock(&chargeMutex);
		pthread_mutex_unlock(&batteryMutex);
		sleep(period);
	}
	return NULL;
}

void *Talert_10(void *arg)
{
	while(1)
	{
		sem_wait(&alert_10_sem);
		pthread_mutex_lock(&speedMutex);
		speed = 30;
		pthread_mutex_unlock(&speedMutex);
		pthread_mutex_lock(&actualPosMutex);
		pthread_mutex_lock(&stationMutex);
		getClosestStation(actualPos,&station);
		pthread_mutex_unlock(&actualPosMutex);
		pthread_mutex_lock(&wpMutex);
		wp = station;
		pthread_mutex_unlock(&wpMutex);
		pthread_mutex_unlock(&stationMutex);
	}
	return NULL;
}

void *Talert_80(void *arg)
{
	while(1)
	{
		sem_wait(&alert_80_sem);
		pthread_mutex_lock(&speedMutex);
		speed = 50;
		pthread_mutex_unlock(&speedMutex);
		pthread_mutex_lock(&wpMutex);
		pthread_mutex_lock(&actualPosMutex);
		pthread_mutex_lock(&destMutex);
		genWp(actualPos,dest,&wp);
		pthread_mutex_unlock(&actualPosMutex);
		pthread_mutex_unlock(&wpMutex);
		pthread_mutex_unlock(&destMutex);
	}
	return NULL;
}

void *Tcameras(void *arg)
{
	while(1)
	{
		sem_wait(&photo_sem);
		pthread_mutex_lock(&actualPosMutex);
		takePhoto(actualPos);
		pthread_mutex_unlock(&coord_t_actualMutex);
	}
	return NULL;
}

void *Tdestination
{
	while (1)
	{
		sem_wait(&wp_sem);
		if (wp == station)
		{
			//arrived at the power station
			pthread_mutex_lock(&chargeMutex);
			charge = true;
			pthread_mutex_unlock(&chargeMutex);
			pthread_mutex_lock(&speedMutex);
			speed = 0;
			pthread_mutex_unlock(&speedMutex);
		}
		else
		{
			pthread_mutex_lock(&wpMutex);
			pthread_mutex_lock(&actualPosMutex);
			pthread_mutex_lock(&destMutex);
			genWp(actualPos, dest, &wp);
			pthread_mutex_unlock(&actualPosMutex);
			pthread_mutex_unlock(&wpMutex);
			pthread_mutex_unlock(&destMutex);
		}
}

void Q_init(void)
{
	sem_init(&alert_10_sem,0,0);
	sem_init(&alert_80_sem,0,0);
	sem_init(&photo_sem, 0, 0);
	sem_init(&wp_sem, 0, 0);
}
int main() {
	cout << "Hello World!!!" << endl; // prints Hello World!!!
	return 0;
}
