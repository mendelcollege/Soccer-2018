#ifndef __DRIVING_H__
#define __DRIVING_H__

#define USEGROVELIB

//MotorDriver

/* 
 *    M1             M2 
 *      \           /
 *      (1)       (1)
 *      md1       md2
 *      (2)       (2)
 *      /           \
 *    M4             M3
 * 
 * Positive rotational value should corespond to:
 *  - Counterclockwise wheelrotation
 *  - Clockwise robot rotation
 * 
 * Dirbits:
 *  - 10 Clockwise
 *  - 01 Anticlockwise
 *  - 0b[m2][m1]
 * 
 */

#define MD1ADDR 0x0f
#define MD2ADDR 0x0e

enum motor
{
    M1 = 0,
    M2 = 1,
    M3 = 2,
    M4 = 3,
    MOTORS = 4
};

#ifdef USEGROVELIB

#include <Grove_I2C_Motor_Driver.h>
I2CMotorDriver md1;
I2CMotorDriver md2;

#else
/**********I2C command definitions***********/
#define MotorSpeedSet               0x82
#define PWMFrequenceSet             0x84
#define DirectionSet                0xaa
#define MotorSetA                   0xa1
#define MotorSetB                   0xa5
#define Nothing                     0x01
/******************Motor ID******************/
#define MOTOR1                      0x01
#define MOTOR2                      0x02
/***************Motor Direction**************/
#define BothClockWise               0x0a //0b00001010
#define BothAntiClockWise           0x05 //0b00000101
#define M1CWM2ACW                   0x06 //0b00000110
#define M1ACWM2CW                   0x09 //0b00001001
/*****Use these for bitwise magic tricks*****/
#define M1CW                        0b00000010
#define M1ACW                       0b00000001
#define M2CW                        0b00001000
#define M2ACW                       0b00000100
/**************Motor Direction***************/
#define ClockWise                   0x0a
#define AntiClockWise               0x05
/************Prescaler Frequence*************/
#define F_31372Hz                   0x01
#define F_3921Hz                    0x02
#define F_490Hz                     0x03
#define F_122Hz                     0x04
#define F_30Hz                      0x05

//////////////////////////////////////////////////////////////////////
//Function to set the 2 DC motor speed
//motorSpeedA : the DC motor A speed; should be 0~100;
//motorSpeedB: the DC motor B speed; should be 0~100;

void MotorSpeedSetAB(byte addr, byte MotorSpeedA, byte MotorSpeedB)  {
  MotorSpeedA=map(MotorSpeedA,0,100,0,255);
  MotorSpeedB=map(MotorSpeedB,0,100,0,255);
  Wire.beginTransmission(addr); // transmit to device I2CMotorDriverAdd
  Wire.write(MotorSpeedSet);        // set pwm header 
  Wire.write(MotorSpeedA);              // send pwma 
  Wire.write(MotorSpeedB);              // send pwmb    
  Wire.endTransmission();    // stop transmitting
}
//set the prescale frequency of PWM, 0x03 default;
void MotorPWMFrequenceSet(byte addr, byte Frequence)  {    
  Wire.beginTransmission(addr); // transmit to device I2CMotorDriverAdd
  Wire.write(PWMFrequenceSet);        // set frequence header
  Wire.write(Frequence);              //  send frequence 
  Wire.write(Nothing);              //  need to send this byte as the third byte(no meaning)  
  Wire.endTransmission();    // stop transmitting
}
//set the direction of DC motor. 
void MotorDirectionSet(byte addr, byte Direction)  {     //  Adjust the direction of the motors 0b0000 I4 I3 I2 I1
  Wire.beginTransmission(addr); // transmit to device I2CMotorDriverAdd
  Wire.write(DirectionSet);        // Direction control header
  Wire.write(Direction);              // send direction control information
  Wire.write(Nothing);              // need to send this byte as the third byte(no meaning)  
  Wire.endTransmission();    // stop transmitting 
}

void SendMotorValues(int *speedarray)
{
    byte dirbits;
    //Controller 1
    dirbits = ((speedarray[M4] >> 7) ? M2CW : M2ACW) | ((speedarray[M1] >> 7) ? M1CW : M1ACW); //BITWISE OR OPERATION. YAY!!!
    MotorDirectionSet(MD1ADDR, dirbits);
    MotorSpeedSetAB(MD1ADDR, abs(speedarray[M1]), abs(speedarray[M4]));
    //Controller 2
    dirbits = ((speedarray[M3] >> 7) ? M2CW : M2ACW) | ((speedarray[M2] >> 7) ? M1CW : M1ACW);
    MotorDirectionSet(MD2ADDR, dirbits);
    MotorSpeedSetAB(MD2ADDR, abs(speedarray[M2]), abs(speedarray[M3]));
}

#endif  //== end of not using grovelib

//Drive Stuff
enum drivetype
{
    PROPORTIONAL,
    LINEAR,
    ROTATIONAL,
    DIRECTIONAL,
    COMPASS
};

struct drivestate
{
    char x;
    char y;
    char rot;
    byte pwr;
    int dir;
    byte type;
} currentdrivestate;

void ProportionalDrive(const char x, const char y, const char rot, const char pwr)
{
    int motorpwr[MOTORS], maxpwr = 0;
    motorpwr[M1] =  x + y + rot;
    motorpwr[M2] =  x - y + rot;
    motorpwr[M3] = -x - y + rot;
    motorpwr[M4] = -x + y + rot;
    maxpwr = + abs(x) + abs(y) + abs(rot);
    for(byte i = M1; i <= M4; i++) motorpwr[i] = motorpwr[i] * pwr / maxpwr;
    #ifdef USEGROVELIB
    md1.speed(MOTOR1, -motorpwr[M1]);
    md1.speed(MOTOR2, -motorpwr[M4]);
    md2.speed(MOTOR1, -motorpwr[M2]);
    md2.speed(MOTOR2, -motorpwr[M3]);
    #else
    SendMotorValues(motorpwr);
    #endif
    currentdrivestate.x = x;
    currentdrivestate.y = y;
    currentdrivestate.rot = rot;
    currentdrivestate.pwr = pwr;
    currentdrivestate.type = PROPORTIONAL;
}

void LinearDrive(const char x, const char y, char rot)
{
    int motorpwr[MOTORS], maxpwr;
    motorpwr[M1] =  x + y;
    motorpwr[M2] =  x - y;
    motorpwr[M3] = -x - y;
    motorpwr[M4] = -x + y;
    maxpwr = abs(x) + abs(y);
    if(maxpwr > 100) for(byte i = M1; i <= M4; i++) motorpwr[i] = motorpwr[i] * 100 / maxpwr;
    else
    {
        if(rot + maxpwr > 100) rot = 100 - maxpwr;
        if(rot - maxpwr < -100) rot = -100 + maxpwr;
        for(byte i = M1; i <= M4; i++) motorpwr[i] += rot;   
    }
    #ifdef USEGROVELIB
    md1.speed(MOTOR1, -motorpwr[M1]);
    md1.speed(MOTOR2, -motorpwr[M4]);
    md2.speed(MOTOR1, -motorpwr[M2]);
    md2.speed(MOTOR2, -motorpwr[M3]);
    #else
    SendMotorValues(motorpwr);
    #endif
    currentdrivestate.x = x;
    currentdrivestate.y = y;
    currentdrivestate.rot = rot;
    currentdrivestate.type = LINEAR;
}

void RotationalDrive(int x, int y, const char rot)
{
    int motorpwr[MOTORS], maxpwr, minpwr;
    motorpwr[M1] =  x + y;
    motorpwr[M2] =  x - y;
    motorpwr[M3] = -x - y;
    motorpwr[M4] = -x + y;
    maxpwr = abs(x) + abs(y);
    if(maxpwr + rot > 100 || -maxpwr + rot < -100) for(byte i = M1; i <= M4; i++) motorpwr[i] = motorpwr[i] * (100 * sign(rot) - rot) / 100 + rot; //Can u follow it?
    else for(byte i = M1; i <= M4; i++) motorpwr[i] += rot;
    #ifdef USEGROVELIB
    md1.speed(MOTOR1, -motorpwr[M1]);
    md1.speed(MOTOR2, -motorpwr[M4]);
    md2.speed(MOTOR1, -motorpwr[M2]);
    md2.speed(MOTOR2, -motorpwr[M3]);
    #else
    SendMotorValues(motorpwr);
    #endif
    currentdrivestate.x = x;
    currentdrivestate.y = y;
    currentdrivestate.rot = rot;
    currentdrivestate.type = ROTATIONAL;
}

void DirectionalDrive(int dir, const char dirvel, const char rotvel, const byte power)
{
    int x, y;
    float dirrad = dir * PI / 180;
    x = (sin(dirrad) * dirvel);
    y = (cos(dirrad) * dirvel);
    ProportionalDrive(x, y, rotvel, power);
    currentdrivestate.dir = dir;
    currentdrivestate.type = DIRECTIONAL;
}

void CompassDrive(int reldir, const char dirvel, const char rotvel, const byte power)
{
    int x, y;
    reldir -= orient;
    while(reldir <= -180) reldir += 360;
    while(reldir >   180) reldir -= 360;
    DirectionalDrive(reldir, dirvel, rotvel, power);
    currentdrivestate.dir = reldir;
    currentdrivestate.type = COMPASS;
}

void StopAllMotors()
{
        #ifdef USEGROVELIB
        md1.stop(MOTOR1);
        md1.stop(MOTOR2);
        md2.stop(MOTOR1);
        md2.stop(MOTOR2);
        #else
        LinearDrive(0,0,0);
        #endif
}

void SetupMotors()
{
    #ifdef USEGROVELIB
    md1.begin(MD1ADDR);
    md2.begin(MD2ADDR);
    #else
    MotorPWMFrequenceSet(MD1ADDR, F_3921Hz);
    MotorPWMFrequenceSet(MD2ADDR, F_3921Hz);
    #endif
    StopAllMotors();
}

#endif
