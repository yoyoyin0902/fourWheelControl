#ifndef IMU_H
#define IMU_H

#include "math.h"
#include "Arduino.h"

class IMU
{
private:
    void crc16_update(uint16_t *currectCRC, const uint8_t *src, uint32_t lengthInBytes);
    uint16_t getCRC(const void *src, int len);
public:
    byte rxBuf[256];
    uint16_t rxIndex = 0;

    float acc_x = 0;
    float acc_y = 0;
    float acc_z = 0;
    float gyr_x = 0;
    float gyr_y = 0;
    float gyr_z = 0;
    float mag_x = 0;
    float mag_y = 0;
    float mag_z = 0;
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    float quaternion_w = 0;
    float quaternion_x = 0;
    float quaternion_y = 0;
    float quaternion_z = 0;
    float final_yaw = 0;
    typedef struct
    {
        uint8_t data_ID;
        uint8_t ID;
        uint8_t reserved[6];
        uint32_t ms;
        float acc[3];
        float gyr[3];
        float mag[3];
        float eul[3];
        float quaternion[4];
    } IMUSOL_t;

    typedef struct
    {
        uint8_t premable;
        uint8_t type;
        uint16_t length;
        uint16_t crc;
        uint8_t payload[256];
    } dataFrame_t;

    void receiveIMU(uint8_t *revData);
};

#endif
