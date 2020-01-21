/*
 *  softi2c.h
 *
 *  Created on: 5 de ago de 2018
 *  Author: Helder Sales
 */

#ifndef INC_SOFTI2CMASTER_H_
#define INC_SOFTI2CMASTER_H_

#include <msp430.h>
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"

void softI2CMasterInit(void);
void pulseSCL(void);
uint8_t *pulseSCLReadData(void);
void startBit(void);
void stopBit(void);
uint8_t sendAddr(uint8_t addr, bool rw);
uint8_t sendData(uint8_t data, bool stop);
void readData(uint8_t *data, uint8_t count);
uint8_t softI2CMaster(uint8_t addr, uint8_t *data, uint8_t count, bool rw);

#define ACK		1
#define NACK	0

#define SDA     BIT4
#define SCL     BIT5

#define WRITE   false
#define READ    true

#define CLOCK   16000000

#endif /* INC_SOFTI2CMASTER_H_ */
