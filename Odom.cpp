#include "Odom.h"

void Odometry::Odm_Reset()
{
    // Odm Reset
    PosX = 0;
    PosY = 0;
    finalPosX = 0;
    finalPosY = 0;

    Rad_yaw = 0;
    theta = 0;
}

void Odometry::Odm_Calu(float deltaPos_R, float deltaPos_L, float _yaw)
{
    if (true)
    {
        if (Odm_reset)
        {
            Odm_Reset();
            Odm_reset = false;
        }
    }
    now_sens = sens;
    now_sens = now_sens < 0 ? 0 : now_sens;
    if (deltaPos_R == 0 && deltaPos_L == 0) //stop case
    {
        PosX = 0;
        PosY = 0;
        theta = 0;
    }
    else if (deltaPos_R > now_sens && deltaPos_L > now_sens && fabs(deltaPos_R - deltaPos_L) < 2.0 * now_sens) //forward case
    {
        PosX = ((deltaPos_R + deltaPos_L) / 2.0) * cos(radians(_yaw));
        PosY = ((deltaPos_R + deltaPos_L) / 2.0) * sin(radians(_yaw));
        theta = radians(_yaw);
    }
    else if (deltaPos_R < -now_sens && deltaPos_L < -now_sens && fabs(deltaPos_R - deltaPos_L) < 2.0 * now_sens) //backward case
    {
        PosX = ((deltaPos_R + deltaPos_L) / 2.0) * cos(radians(_yaw));
        PosY = ((deltaPos_R + deltaPos_L) / 2.0) * sin(radians(_yaw));
        theta = radians(_yaw);
    }
    else if (deltaPos_R == -deltaPos_R || deltaPos_L == -deltaPos_L) //spin case
    {
        PosX = 0;
        PosY = 0;
    }
    else //Turn
    {
        Nr = 2.0 * TWO_WHEEL_DIS * (deltaPos_R / (deltaPos_R - deltaPos_L)) + HELF_WHEEL_DIS;
        if (deltaPos_R == 0 && deltaPos_L == 0)
            Nr = 0;
        PosX = -(Nr * (sin(Rad_yaw + radians(_yaw)) - sin(radians(_yaw))));
        PosY = -(Nr * (-cos(Rad_yaw + radians(_yaw)) + cos(radians(_yaw))));
    }

    finalPosX += PosX;
    finalPosY += PosY;

    // float CR = Pos_R * ADJUST_R_WHEEL_FACTOR;
    // float CL = Pos_L * ADJUST_L_WHEEL_FACTOR;

    // // float SR = CR - RightPosPre;
    // // float SL = CL - LeftPosPre;

    // // RightPosPre = CR;
    // // LeftPosPre = CL;

    // float now_sens = sens;
    // if (CR == 0 && CL == 0)
    // {
    //   PosX = 0;
    //   PosY = 0;
    // }
    // else if (CR > now_sens && CL > now_sens && (CR - CL) <= now_sens * 2 && (CL - CR) <= now_sens * 2)
    // {
    //   PosX = ((CR + CL) / 2) * cos(radians(_yaw));
    //   PosY =
    // }
}
