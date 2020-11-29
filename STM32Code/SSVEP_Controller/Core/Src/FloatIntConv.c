#include "FloatIntConv.h"

// Function definitions
//1. Convert from float to uint8 array (IEEE 754 rep.)
void float2int(float *floatnum, uint8_t *intnum)
{
		// Instantiating the union used for conversion
		FloatIntUnion floatint;
		floatint.f = *floatnum;
		// Saving values to contents of given address
		*(intnum+0) = floatint.raw.byte1;
		*(intnum+1) = floatint.raw.byte2;
		*(intnum+2) = floatint.raw.byte3;
		*(intnum+3) = floatint.raw.byte4;
};
//2. Convert from uint8 array (IEEE 754 rep.) to float
void int2float(float *floatnum, uint8_t *intnum)
{
		// Instantiating the union used for conversion
		FloatIntUnion floatint;
		floatint.raw.byte1 = *(intnum+0);
		floatint.raw.byte2 = *(intnum+1);
		floatint.raw.byte3 = *(intnum+2);
		floatint.raw.byte4 = *(intnum+3);
		// Saving values to contents of given address
		*floatnum = floatint.f;
};
