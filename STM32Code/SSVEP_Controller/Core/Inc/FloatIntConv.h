#include <stdio.h> 
#include <stdint.h>

// Converts between floats (IEEE 754 rep.) and uint8 arrays
typedef union { 
  
    float f; 
    struct
    { 
        // Here the members of the union data structure 
        // use the same memory (32 bits). 
        // The ordering is taken from the LSB to the MSB.
        // sign + exp + manti = byte1 + byte2 + byte3 + byte4
        uint8_t byte4 : 8;
        uint8_t byte3 : 8;
        uint8_t byte2 : 8;
        uint8_t byte1 : 8;
        
    } raw; 
    
} FloatIntUnion;

// Function prototypes
//1. Convert from float to uint8 array (IEEE 754 rep.)
void float2int(float *floatnum, uint8_t *intnum);
//2. Convert from uint8 array (IEEE 754 rep.) to float
void int2float(float *floatnum, uint8_t *intnum);
