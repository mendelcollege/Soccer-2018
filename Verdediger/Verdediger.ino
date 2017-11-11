//Serial
#define Cereal Serial //I am a funny guy

//Program options
#define AUTOCENTER

#define ENABLESWITCHES

//#define MOTORTEST
    //#define TESTDIRECTIONAL
    #define DriveFunction RotationalDrive

/*
//Handydandy sign function
template <typename type> type sign(type value)
{ 
 return type((value > 0) - ( value < 0)); 
}
*/

char sign(int i)
{
  if(i > 0) return 1;
  if(i < 0) return -1;
  else return 0;
}

//Libs
#include "Sensors.h" //Includes wire.begin() for Driving.h
#include "Driving.h"

//Switchpins
#define SENSORLOGWITCHPIN 2
#define MOTORSWITCHPIN 3

//Behaviour
#define DEFLECTTHRES 1000
#define RETURNTHRES 3000
#define DEFLECTTIME 1000
#define MINBACKDIST 20 //Calib
#define OPTIMALBACKDIST 35//Calib
#define MAXBACKDIST 50 //Calib
#define MAXCENTEROFFSET 25 //Calib (or GOALWIDTH/2)

enum behaviour
{
    GUARD,
    TRACK,
    RETURN,
    DEFLECT
};

enum behaviour currentbehaviour = GUARD;
byte sidesign;

void Guard()
{
        char correctionspeed = constrain(-orient, -25, 25);//Wow lots of complicatings
        if(usbval < MINBACKDIST)
        {
            RotationalDrive(0, 100, correctionspeed);
        }
        else if(ballstate == LOST)
        {
            if(losttime > RETURNTHRES) currentbehaviour = RETURN;
            else if(usbval > MAXBACKDIST) RotationalDrive(0, -100, correctionspeed);
            else StopAllMotors();
        }
        else if(balldir == 0)
        {
           if(ballstate != FAR) RotationalDrive(0, 100, correctionspeed);
        }
        else //if(balldir != 0)
        {
            if(abs(xpos) > MAXCENTEROFFSET && sign(balldir) == sign(xpos)) //Wil verder dan zijkant goal
            {
                if(usbval > OPTIMALBACKDIST)
                {
                    if(abs(balldir) > 3) RotationalDrive(-30 * sign(balldir), -100, correctionspeed);
                    else RotationalDrive(0, -100, correctionspeed);
                }
                else // if(usbval <= OPTIMALBACKDIST)
                {
                    StopAllMotors();
                    //currentbehaviour = TRACK;
                    sidesign = sign(balldir);
                }
            }
            else //if((abs(xpos) <= MAXCENTEROFFSET)
            {
                    switch(abs(balldir))
                    {
                        case 1:
                            if(usbval > OPTIMALBACKDIST) RotationalDrive(100 * sign(balldir), 0, correctionspeed);
                            else RotationalDrive(100 * sign(balldir), 20, correctionspeed);
                            break;
                        case 2:
                            RotationalDrive(100 * sign(balldir), 0, correctionspeed);
                            break;
                        case 3:
                            RotationalDrive(0, -100, correctionspeed);
                            break;
                        case 4:
                            RotationalDrive(-20 * sign(balldir), -100, correctionspeed);
                            break;
                    }
            }
        }
}

void Guard2()

{
        char correctionspeed = constrain(-orient, -15, 15);//Wow lots of complicatings
        if(usbval < MINBACKDIST)
        {
            LinearDrive(0, 100, correctionspeed);
        }
        else if(usbval > MAXBACKDIST) RotationalDrive(0, 100, correctionspeed);
        else if(ballstate == LOST)
        {
            if(usbval > MAXBACKDIST) RotationalDrive(0, -100, correctionspeed);
            else StopAllMotors();
        }
        else if(balldir == 0)
        {
           RotationalDrive(0, 100, correctionspeed);
        }
        else //if(balldir != 0)
        {
            if(abs(xpos) > MAXCENTEROFFSET && sign(balldir) == sign(xpos)) //Wil verder dan zijkant goal
            {
                
                {
                    RotationalDrive(0, -100, correctionspeed);
                }
            }
            else //if((abs(xpos) <= MAXCENTEROFFSET)
            {
                    switch(abs(balldir))
                    {
                        case 1:
                             RotationalDrive(100 * sign(balldir), 0, correctionspeed);
                            
                            break;
                        case 2:
                            RotationalDrive(100 * sign(balldir), 0, correctionspeed);
                            break;
                        case 3:
                            RotationalDrive(0, -100, correctionspeed);
                            break;
                        case 4:
                            RotationalDrive(-20 * sign(balldir), -100, correctionspeed);
                            break;
                    }
            }
        }
}

void Track() //side sign: sign of balldir when trackbehaviour is started; left = -1, right = 1
{
    if(ballstate == LOST)
    {
        RotationalDrive(0, 0, constrain(-orient, -30, 30));
        if(losttime > RETURNTHRES) currentbehaviour = RETURN;
    }
    if(balldir == 0)
    {
        StopAllMotors();
        if(straighttime > DEFLECTTHRES) currentbehaviour = DEFLECT;
    }
    else
    {
        if(sign(balldir) == sidesign)
        {
            if(orient * sidesign < 80) RotationalDrive(0,0, 60 * sign(balldir));
            else StopAllMotors();
        }
        else
        {
            if(orient * sidesign > 20) RotationalDrive(0,0, 60 * sign(balldir));
            else currentbehaviour = GUARD;
        }
    }
}

void ReturnToBeginPos()
{
    if(ballstate != LOST) currentbehaviour = GUARD;
    else
    {
        if(abs(ypos) < 10 && abs(xpos) < 10) currentbehaviour = GUARD;
        ProportionalDrive(constrain(-xpos * 3, -100, 100), constrain(-ypos * 2, -100, 100), map(constrain(-orient, -90, 90), -90, 90, -70, 70), 100);
    }
}

void Deflect()
{
    static unsigned tdeflect;
    if(tdeflect == 0) tdeflect = millis();
    if(abs(balldir) > 2 || millis() - tdeflect >= DEFLECTTIME)
    {
        currentbehaviour = GUARD;
        tdeflect = 0;
    }
    else if(millis() - tdeflect > (DEFLECTTIME / 2)) LinearDrive(0, -100, 0);
    else if(balldir == 0) LinearDrive(0, 100, 0);
}

static unsigned long t0;

void setup()
{
    Cereal.begin(9600);
    SetupSensors();
    Cereal.println("Sensors Set up.");
    Cereal.flush();
    delay(500);
    SetupMotors();
    pinMode(SENSORLOGWITCHPIN, INPUT);
    pinMode(MOTORSWITCHPIN, INPUT);
    t0 = millis();
}

#ifdef MOTORTEST
#ifdef TESTDIRECTIONAL

void loop()
{
    static int i = 0;
    if(!t0) t0 = millis();
    if(digitalRead(SENSORLOGWITCHPIN)) if(UCSR0A & _BV(TXC0)) TransmitSensorValues();//Embedded lower level programming... Yay!
    if(digitalRead(MOTORSWITCHPIN))
    {
        StopAllMotors();
    }
    else
    {
        if(millis() - t0 >= 50)
        {
            t0 = 0;
            i++;
            if(i == 360) i = 0;
        }
    }
}

#else

void loop()
{
    if(digitalRead(SENSORLOGWITCHPIN)) TransmitSensorValues();//Embedded lower level programming... Yay!
    if(!digitalRead(MOTORSWITCHPIN)) StopAllMotors();
    else
    {
        LinearDrive(100, 0, 0);
    }
}

#endif
#else

void loop()
{
    UpdateSensorValues();
    if(digitalRead(SENSORLOGWITCHPIN))
        //if((UCSR0A & _BV(TXC0))) TransmitSensorValues();//Embedded lower level programming... Yay!
        TransmitSensorValues();
    if(!digitalRead(MOTORSWITCHPIN))
    {
        StopAllMotors();
    }
    else
    {
        switch(currentbehaviour)
        {
            case GUARD:
                Guard();
                break;
            case TRACK:
                Track();
                break;
            case RETURN:
                ReturnToBeginPos();
                break;
            case DEFLECT:
                Deflect();
                break;
        }
    }
}

#endif
