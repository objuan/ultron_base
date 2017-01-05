

#ifndef __CRC16_H
#define __CRC16_H

#include <stdint.h>

/***----------Table-driven crc function----------***/
/*Inputs: -size of the character array, the CRC of which is being computed   */
/*        - the initial value of the register to be used in the calculation  */
/*        - a pointer to the first element of said character array           */
/*Outputs: the crc as an unsigned short int                                  */
uint16_t crc16(int size, int init_val, uint8_t *data);

#endif
