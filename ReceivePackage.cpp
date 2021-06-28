#include "ReceivePackage.h"

ReceivePackage::ReceivePackage() {}
ReceivePackage::~ReceivePackage()
{
}

void ReceivePackage::CheckConnect(int target)
{
    if (target == con_check)
    {
        con_count++;
        if (con_count > 10)
        {
            vx = 0;
            vy = 0;
            w = 0;
            con_count = 0;
        }
    }
    con_check = target;
}

void ReceivePackage::ReceiveData(unsigned char data, float *vx, float *vy, float *w)
{
    if (data == 'S' && count == 0)
    {
        databuf[count] = data;
        count++;
    }
    else if (data == 'T' && count == 1)
    {
        databuf[count] = data;
        count++;
    }
    else if (count > 1 && count < 17)
    {
        databuf[count] = data;
        count++;
    }
    else if (data == 'E' && count == 17)
    {
        databuf[count] = data;
        count++;
    }
    else if (data == 'N' && count == 18)
    {
        databuf[count] = data;
        count++;
    }
    else if (data == 'D' && count == 19)
    {
        databuf[count] = data;
        count = 0;

        checksum = databuf[2] + databuf[3] + databuf[4] + databuf[5] + databuf[6] + databuf[7] + databuf[8] + databuf[9] + databuf[10] + databuf[11] + databuf[12] + databuf[13];
        if (databuf[0] == 'S' && databuf[1] == 'T' && databuf[17] == 'E' && databuf[18] == 'N' && databuf[19] == 'D' && databuf[14] == checksum)
        {
            connection = databuf[15];
            mode = databuf[16];
            HighByte_integer_vx = databuf[2];
            LowByte_integer_vx = databuf[3];
            HighByte_float_vx = databuf[4];
            LowByte_float_vx = databuf[5];

            HighByte_integer_vy = databuf[6];
            LowByte_integer_vy = databuf[7];
            HighByte_float_vy = databuf[8];
            LowByte_float_vy = databuf[9];

            HighByte_integer_w = databuf[10];
            LowByte_integer_w = databuf[11];
            HighByte_float_w = databuf[12];
            LowByte_float_w = databuf[13];

            interger_vx = HighByte_integer_vx * 256 + LowByte_integer_vx;
            float_vx = HighByte_float_vx * 256 + LowByte_float_vx;

            interger_vy = HighByte_integer_vy * 256 + LowByte_integer_vy;
            float_vy = HighByte_float_vy * 256 + LowByte_float_vy;

            interger_w = HighByte_integer_w * 256 + LowByte_integer_w;
            float_w = HighByte_float_w * 256 + LowByte_float_w;

            re_vx = interger_vx + float_vx / 1000.0;
            re_vy = interger_vy + float_vy / 1000.0;
            re_w = interger_w + float_w / 1000.0;

            *vx = re_vx - 100; // V[m/s]
            *vy = re_vy - 100; // 不使用
            *w = re_w - 100;   // W[rad/s]

        }
    }
    else
    {
        memset(databuf, 0, sizeof(databuf));
        count = 0;
    }
}
