#include "STP_23L.h"


//一次收195个字节
STP_DATA STP_23L[12];

float stp23l_distance_left = 0;
float stp23l_distance_right = 0;

void STP23L_Distance_Process(uint8_t *data,float *stp23l_distance)
{
	int32_t diatance_agg = 0;
	if(data[0]==0xAA && data[1]==0xAA && data[2]==0xAA && data[3]==0xAA &&
       data[5]==0x02 && data[8]==0xB8 && data[9]==0x00)
    {
		for(int i = 0; i < 12; i++)
		{
			int base = 10 + 15 * i;

			STP_23L[i].distance   = (data[base + 1] << 8) + data[base];
			STP_23L[i].noise      = (data[base + 3] << 8) + data[base + 2];
			STP_23L[i].peak       = ((uint32_t)data[base + 7] << 24) + ((uint32_t)data[base + 6] << 16)
								  + ((uint32_t)data[base + 5] << 8) + data[base + 4];
			STP_23L[i].confidence = data[base + 8];
			STP_23L[i].intg       = ((uint32_t)data[base + 12] << 24) + ((uint32_t)data[base + 11] << 16)
								  + ((uint32_t)data[base + 10] << 8) + data[base + 9];
			STP_23L[i].reftof     = (data[base + 14] << 8) + data[base + 13];
			
			diatance_agg += STP_23L[i].distance;
		}
		
		*stp23l_distance = (diatance_agg / 12.0f) / 1000.0f;
		
    }
}
