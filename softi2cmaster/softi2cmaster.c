/*
 *  softi2c.c
 *
 *  Created on: 5 de ago de 2018
 *  Author: Helder Sales
 */

#include "softi2cmaster.h"

void softI2CMasterInit(void)
{
    // Init I2C pins to a known state
    P1DIR &= ~(SDA | SCL);
    P1OUT &= ~(SDA | SCL);
}

// Clock pulse
void pulseSCL(void)
{
    P1DIR &= ~SCL; // SCL High

    __delay_cycles(CLOCK * 0.000005);

    P1DIR |= SCL; // SCL Low
}

// Generates a clock pulse and reads SDA (approx. in the middle of clock pulse)
// Returns the address of the pointer to the SDA data
uint8_t *pulseSCLReadData(void)
{
    uint8_t *readSDA = malloc(sizeof *readSDA);

    *readSDA = 0;

    P1DIR &= ~SCL; // SCL High

    __delay_cycles(CLOCK * 0.000003);

    *readSDA = (SDA & P1IN);

    __delay_cycles(CLOCK * 0.000002);

    P1DIR |= SCL; // SCL Low

    return readSDA;
}

// Start I2C comm
void startBit(void)
{
    P1DIR |= SDA; //SDA Low

    __delay_cycles(CLOCK * 0.000005);


    P1DIR |= SCL; //SCL Low

    __delay_cycles(CLOCK * 0.000001);
}

// Stop I2C comm
void stopBit(void)
{
    P1DIR &= ~SCL; // SCL High

    __delay_cycles(CLOCK * 0.000005);

    P1DIR &= ~SDA; // SDA High

    __delay_cycles(CLOCK * 0.000005);
}

// Sends the I2C address
uint8_t sendAddr(uint8_t addr, bool rw)
{
    bool ack = false;

    uint8_t auxAddr = addr;

    startBit();

    // Transmit address, retry 3 times if NACK
    for(uint8_t count = 0; count < 4; count++)
    {
        // Sends 7-bit address (MSB -> LSB)
        // 0x40 = 0b0100 0000 --> reads the 7th bit then shift left until all
        // bits are read
        for(uint8_t i = 0; i < 7; i++)
        {

            if(addr & 0x40)
                P1DIR &= ~SDA; // SDA High

            else
                P1DIR |= SDA; // SDA Low

            __delay_cycles(CLOCK * 0.000005);

            pulseSCL();

            __delay_cycles(CLOCK * 0.000001);

            addr <<= 1;
        }

        // Select read (low) or write (high)
        if(!rw)
            P1DIR |= SDA; // SDA Low

        else
            P1DIR &= ~SDA; // SDA High

        __delay_cycles(CLOCK * 0.000005);

        pulseSCL();

        __delay_cycles(CLOCK * 0.000001);

        // Release SDA line to receive ACK from the other device
        P1DIR &= ~SDA; // SDA High

        __delay_cycles(CLOCK * 0.000005);

        P1DIR &= ~SCL; // SCL High

        __delay_cycles(CLOCK * 0.000003);

        ack = (SDA & P1IN);

        __delay_cycles(CLOCK * 0.000002);

        P1DIR |= SCL; // SCL Low

        __delay_cycles(CLOCK * 0.000005);

        // If received ACK (SDA low), then proceed
        if(ack == 0)
            break;

        // If NACK, try send the address one more time
        addr = auxAddr;
    }

    if(ack == 1)
        return NACK;

    else
        return ACK;
}

// Transmit 8-bit data after sending the address
uint8_t sendData(uint8_t data, bool stop)
{
    bool ack = false;

    uint8_t auxData = data;

    // Transmit data, retry 3 times if NACK
    for(uint8_t count = 0; count < 4; count++)
    {
        // Send 8-bit data (MSB -> LSB)
        for(uint8_t i = 0; i < 8; i++)
        {
            if(data & 0x80)
                P1DIR &= ~SDA; // SDA High

            else
                P1DIR |= SDA; // SDA Low

            __delay_cycles(CLOCK * 0.000005);

            pulseSCL();

            __delay_cycles(CLOCK * 0.000001);

            data <<= 1;
        }

        // Release SDA line to receive ACK from the other device
        P1DIR &= ~SDA; // SDA High

        __delay_cycles(CLOCK * 0.000005);

        P1DIR &= ~SCL; // SCL High

        __delay_cycles(CLOCK * 0.000003);

        ack = (SDA & P1IN);

        __delay_cycles(CLOCK * 0.000002);

        // Turn SDA logic low to avoid stop bit problems because SCL must be
        // logic high before SDA
        P1DIR |= SDA; // SDA Low
        P1DIR |= SCL; // SCL Low

        __delay_cycles(CLOCK * 0.000005);

        if(ack == 0)
            break;

        data = auxData;
    }

    if(ack == 1)
        return NACK;

    // End transmission
    if(stop)
        stopBit();

    return ACK;
}

// Receives I2C data
// Choose how many bytes to read
void readData(uint8_t *data, uint8_t count)
{
    // Creates a pointer to read from SDA
    uint8_t *readSDAData;

    // Clear the receive data buffer
    memset(data, 0x00, count);

    // Receives n requested bytes of data
    for(uint8_t i = 0; i < count; i++)
    {
        // Receive 8-bit data (MSB -> LSB)
        for(uint8_t j = 0; j < 8; j++)
        {
            readSDAData = pulseSCLReadData();

            // Save data in buffer (MSB to LSB)
            data[i] |= *readSDAData << (7 - j);

            // If not the last byte, hold SDA low sending a ACK to inform slave
            // to continue sending data
            if(j == 7 && i < count - 1)
                P1DIR |= SDA; // SDA Low

            __delay_cycles(CLOCK * 0.000005);
        }

        // If the last byte, break from loop and send NACK to inform all data
        // was received
        if(i == count - 1)
        {
            // Sends NACK with a pulse
            pulseSCL();

            // SDA low before transmit stop bit
            P1DIR |= SDA; // SDA Low

            __delay_cycles(CLOCK * 0.000005);

            break;
        }

        pulseSCL();

        P1DIR &= ~SDA; //SDA High

        __delay_cycles(CLOCK * 0.000005);
    }

    free(readSDAData);

    // End receive
    stopBit();
}

uint8_t softI2CMaster(uint8_t addr, uint8_t *data, uint8_t count, bool rw)
{
    uint8_t ack = 0;

    if(!rw)
    {
        ack = sendAddr(addr, false);

        for(uint8_t i = 0; i < count - 1; i++)
        {
            ack = sendData(*data, false);

            data++;
        }

        ack = sendData(*data, true);
    }

    else
    {
        ack = sendAddr(addr, true);

        readData(data, count);
    }

    return ack;
}
