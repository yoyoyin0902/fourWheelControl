#ifndef RECEIVEPACKAGE_H
#define RECEIVEPACKAGE_H

#include "Arduino.h"
class ReceivePackage
{
private:
    unsigned char databuf[20]; // 用來儲存收進來的 data byte
    int count = 0;
    int connection = -1; //判斷連線
    int con_check = -1;  //判斷連線
    int con_count = 0;
    int mode = 0;

    char checksum;
    int HighByte_integer_vx = 0;
    int LowByte_integer_vx = 0;
    int HighByte_float_vx = 0;
    int LowByte_float_vx = 0;

    int HighByte_integer_vy = 0;
    int LowByte_integer_vy = 0;
    int HighByte_float_vy = 0;
    int LowByte_float_vy = 0;

    int HighByte_integer_w = 0;
    int LowByte_integer_w = 0;
    int HighByte_float_w = 0;
    int LowByte_float_w = 0;

    int interger_vx = 0;
    int float_vx = 0;
    int interger_vy = 0;
    int float_vy = 0;
    int interger_w = 0;
    int float_w = 0;

    double re_vx; 
    double re_vy;
    double re_w;

public:

    float vx; //上層給速度x
    float vy;     //上層給速度y(不用)
    float w;      //上層給w

    ReceivePackage();
    void CheckConnect(int target);
    ~ReceivePackage();
    void ReceiveData(unsigned char data, float *vx, float *vy, float *w);
};

#endif

