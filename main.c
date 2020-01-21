#include <msp430.h>
#include "softi2cmaster/softi2cmaster.h"

uint32_t g_errorCount;

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    if (CALBC1_16MHZ == 0xFF)
        while(1);

    DCOCTL = 0;
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;

    softI2CMasterInit();

    __bis_SR_register(GIE);

    uint8_t data = 0x00;
    uint8_t *pdata = &data;

    if(!softI2CMaster(0x38, pdata, sizeof *pdata, WRITE))
        g_errorCount++;

    for(;;)
    {
        *pdata = 0x0F;

        if(!softI2CMaster(0x38, pdata, sizeof *pdata, WRITE))
            g_errorCount++;

        __delay_cycles(CLOCK * 0.5);

        *pdata = 0xF0;

        if(!softI2CMaster(0x38, pdata, sizeof *pdata, WRITE))
            g_errorCount++;

        __delay_cycles(CLOCK * 0.5);
    }
}
