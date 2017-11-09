#ifndef __SENSORS_H__
#define __SENSORS_H__

#include <Wire.h>
#include <NewPing.h>
#include <Adafruit_L3GD20.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_Sensor.h>
#include "IRSeeker.h"

//Compass
Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(40273);
int orient, beginorient;

//IR
#define POSSESSTHRES 240 //Calib
#define LOSSTHRES 200 //Calib
#define FARCLOSE 150 //Calib

enum ballstate
{
    LOST,
    FAR,
    CLOSE,
    POSSESSION
};

struct InfraredResult irdata;
int balldist, lastballdist = 0;
char balldir, lastballdir = 1;
byte ballstate;
unsigned long tlastseen, losttime, tlaststraight, straighttime;

//Ultrasone
#define MAXUSVAL 300

#define USLPIN 4
#define USBPIN 6
#define USRPIN 5

NewPing usl(USLPIN, USLPIN, MAXUSVAL);
NewPing usb(USBPIN, USBPIN, MAXUSVAL);
NewPing usr(USRPIN, USRPIN, MAXUSVAL);

int xpos, ypos, usl0, usb0, usr0, xcenter;
int uslval, usbval, usrval;

//Field dimensions (in cm)
#define STADIUMWIDTH 182
#define STADIUMLENGTH 244

#define FIELDWIDTH 122
#define FIELDLENGTH 184

#define GOALWIDTH 60
#define GOALWALLDIST 61
#define GOALDEPTH 10

//Sensor global
void SetupSensors()
{
    sensors_event_t mag_event;
    sensors_vec_t orientationvector;
    
    InfraredSeeker::Initialize(); //Includes wire.begin();
    Serial.println("IR Set up");
    Serial.flush();
    
    if(!mag.begin())
    {
        //There was a problem detecting the LSM303 ... check your connections 
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        Serial.flush();
    }
    
    Serial.println("Mag Set up");
    Serial.flush();

    mag.getEvent(&mag_event);
    if (dof.magGetOrientation(SENSOR_AXIS_Y, &mag_event, &orientationvector))
    {
        beginorient = (int) orientationvector.heading; //Which one of them is the question. Pitch, yawn or azimsdjfhskdhf?
    }
    
    
    Serial.println("Mag first read");
    Serial.flush();
    
    usl0 = usl.ping_median();
    usb0 = usb.ping_median();
    usr0 = usr.ping_median();
    Serial.println("US Set up");
    Serial.flush();
    delay(500);
    
    #ifdef AUTOCENTER
    xcenter = (usl0 + usr0) >> 1;
    #endif
}

void UpdateSensorValues()
{
    sensors_event_t mag_event;
    sensors_vec_t orientationvector;
    
    mag.getEvent(&mag_event);
    if (dof.magGetOrientation(SENSOR_AXIS_Y, &mag_event, &orientationvector))
    {
        orient = (int) orientationvector.heading; //Which one of them is the question. Pitch, yawn or azimsdjfhskdhf?
        orient -= beginorient;
        while(orient < -180) orient += 360;
        while(orient > 180) orient -=360;
    }

    irdata = InfraredSeeker::ReadAC();
    balldir = irdata.Direction;
    balldist = irdata.Strength;
    if(balldist == 0)
    {
        losttime = millis() - tlastseen;
        if(losttime > 100) ballstate = LOST; //Calib
        balldir = lastballdir;
        balldist = lastballdist;
    }
    else
    {
        tlastseen = millis();
        losttime = 0;
        lastballdir = balldir;
        lastballdist = balldist;
        if(balldir == 5 && balldist > POSSESSTHRES) ballstate = POSSESSION;
        else if((balldist > LOSSTHRES && ballstate != POSSESSION) || balldist <= LOSSTHRES)
        {
            if(balldist > FARCLOSE) ballstate = CLOSE;
            else ballstate = FAR;
        }
    }
    balldir -= 5;
    balldir = -balldir; //Optional flip so that rightward values are positive
    if(balldir == 0)
    {
        if(!tlaststraight) tlaststraight = millis();
        straighttime = tlaststraight - millis();
    }
    else
    {
        tlaststraight = 0;
        straighttime = 0;
    }
    uslval = usl.ping_cm();
    usbval = usb.ping_cm();
    usrval = usr.ping_cm();
    xpos = uslval - usl0;
    /*
    #ifdef AUTOCENTER
    if(uslval > usrval) xpos = uslval - xcenter;
    else xpos = xcenter - usrval;
    #else
    if(uslval > usrval) xpos = uslval - usl0;
    else xpos = usr0 - usrval;
    #endif
    */
}

void TransmitSensorValues()
{
    char msg[50];
    sprintf(msg, "Dir:%d\tDist:%d\tOrient:%d\t USL:%d\tUSB:%d\tUSR:%d\n", balldir, balldist, orient, uslval, usbval, usrval);
    Cereal.print(msg);
}

#endif
