#include "IMU.h"

void IMU::crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j = 0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)

        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currectCrc = crc;
}

uint16_t IMU::getCRC(const void *src, int len)
{
    uint16_t CRC = 0x00;
    crc16_update(&CRC, src, 4);
    crc16_update(&CRC, src + 6, len - 6);
    return CRC;
}


void IMU::receiveIMU(uint8_t *revData)
{
    if (rxIndex >= 82)
    { //0x91 DATD_ID
        dataFrame_t frame;
        memcpy(&frame, &rxBuf[rxIndex - 82], 82);
        if (frame.premable == 0x5A && frame.type == 0xA5)
        {
            if (getCRC(&frame, 82) == frame.crc)
            {
                IMUSOL_t *payload = (IMUSOL_t *)frame.payload;
                acc_x = payload->acc[0];
                acc_y = payload->acc[1];
                acc_z = payload->acc[2];
                //角速度
                gyr_x = payload->gyr[0];
                gyr_y = payload->gyr[1];
                gyr_z = payload->gyr[2];
                //磁場速度
                mag_x = payload->mag[0];
                mag_y = payload->mag[1];
                mag_z = payload->mag[2];

                roll = payload->eul[0];
                pitch = payload->eul[1];
                yaw = payload->eul[2];
                //四元數
                quaternion_w = payload->quaternion[0];
                quaternion_x = payload->quaternion[1];
                quaternion_y = payload->quaternion[2];
                quaternion_z = payload->quaternion[3];

                //yaw校正
                if (yaw > 0)
                    yaw = yaw + 360;
                final_yaw = yaw;
                if (final_yaw >= 180)
                    final_yaw = final_yaw - 360;
                final_yaw *= -1;
            }
            rxIndex = 0;
        }
    }
}