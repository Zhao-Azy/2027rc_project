#include "radar.h"

static uint8_t Ccr_Check(uint8_t *message, uint8_t start_index, uint8_t len);

typedef union
{
    float f;
    uint8_t u8[4];
}F_U8;


void Radar_Data_Process(Radar_HandleTypeDef *radar, uint8_t *pdata)
{
    F_U8 x;
    F_U8 y;
    F_U8 z;
    F_U8 translation_distance;
    F_U8 high_err;
    F_U8 front_err;

     if(pdata[0] == 0xFF && pdata[29] == 0xFE)
    {
        if(Ccr_Check(pdata, 1, 27) == pdata[28])
        {
            x.u8[0] = pdata[1];
            x.u8[1] = pdata[2];
            x.u8[2] = pdata[3];
            x.u8[3] = pdata[4];

            y.u8[0] = pdata[5];
            y.u8[1] = pdata[6];
            y.u8[2] = pdata[7];
            y.u8[3] = pdata[8];

            z.u8[0] = pdata[9];
            z.u8[1] = pdata[10];
            z.u8[2] = pdata[11];
            z.u8[3] = pdata[12];

            translation_distance.u8[0] = pdata[13];
            translation_distance.u8[1] = pdata[14];
            translation_distance.u8[2] = pdata[15];
            translation_distance.u8[3] = pdata[16];

            high_err.u8[0] = pdata[17];
            high_err.u8[1] = pdata[18];
            high_err.u8[2] = pdata[19];
            high_err.u8[3] = pdata[20];

            front_err.u8[0] = pdata[21];
            front_err.u8[1] = pdata[22];
            front_err.u8[2] = pdata[23];
            front_err.u8[3] = pdata[24];

            radar->x = x.f;
            radar->y = y.f;
            radar->z = z.f;
            radar->high_err = high_err.f / 1000.0f;
            radar->front_err = front_err.f / 1000.0f;
			radar->translation_distance = translation_distance.f / 1000.0f;
            radar->route_flag = pdata[25];
            radar->transmit_block_flag = pdata[26];
            radar->error_flag = pdata[27];
        }

    }


}

static uint8_t Ccr_Check(uint8_t *message, uint8_t start_index, uint8_t len)
{
    uint8_t ccr = 0;
    for (int i = start_index; i < start_index + len; i++)
    {
        ccr += message[i];
    }
    return ccr;
}