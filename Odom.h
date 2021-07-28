#ifndef Odometry_H
#define Odometry_H

#include "math.h"
#include "Arduino.h"

class Odometry
{
private:
    const int TWO_WHEEL_DIS = 400;                    //兩輪中心間距(mm)
    const float HELF_WHEEL_DIS = TWO_WHEEL_DIS / 2.0; //單輪中心間距(mm)
    const float sens = 10.0; // 里程計敏感值
    float PosX = 0;
    float PosY = 0;
    float Nr = 0;
    float theta = 0;
    float Rad_yaw = 0;
    float now_sens = 0;
    bool Odm_reset = false;
    void Odm_Reset();

public:
    float finalPosX = 0;
    float finalPosY = 0;
    void Odm_Calu(float deltaPos_R, float deltaPos_L, float _yaw);
};

#endif