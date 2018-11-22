
//------------------------------------------------------------------------------

#include <sys/types.h>
#include <sys/stat.h>
       
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include "../lib/types.h"
#include "memory.h"
#include <unistd.h>

#include "../lib/radio/pi_cc_spi.h"
#include "../lib/radio/pi_cc_cc1101.h"

#define SPI_MOCK_FILE "spimock.test"
// ------------------------------------------------------------------------------------------------
// Initialize default parameters
void PI_CC_SPIParmsDefaults(spi_parms_t *spi_parms)
// ------------------------------------------------------------------------------------------------
{
    spi_parms->mode             = 0;
    spi_parms->bits             = 8;
    spi_parms->speed            = 1000000;
    spi_parms->delay            = 4;
    spi_parms->fd               = 0;
    spi_parms->ret              = 0;
    
    spi_parms->tr.tx_buf        = (unsigned long) spi_parms->tx;
    spi_parms->tr.rx_buf        = (unsigned long) spi_parms->rx;
    spi_parms->tr.len           = 0;
    spi_parms->tr.delay_usecs   = 0;
    spi_parms->tr.speed_hz      = 1000000;
    spi_parms->tr.bits_per_word = 4;
}

// ------------------------------------------------------------------------------------------------
// Delay function. # of CPU cycles delayed is similar to "cycles". Specifically,
// it's ((cycles-15) % 6) + 15.  Not exact, but gives a sense of the real-time
// delay.  Also, if MCLK ~1MHz, "cycles" is similar to # of useconds delayed.
void PI_CC_Wait(unsigned int cycles)
// ------------------------------------------------------------------------------------------------
{
    while(cycles>15)                        // 15 cycles consumed by overhead
        cycles = cycles - 6;                // 6 cycles consumed each iteration
}

// ------------------------------------------------------------------------------------------------
int PI_CC_SPISetup(spi_parms_t *spi_parms, arguments_t *arguments)
// ------------------------------------------------------------------------------------------------
{
    spi_parms->ret = 0;

        spi_parms->fd = open(SPI_MOCK_FILE, O_RDWR);
        if (spi_parms->fd < 0)
        {
            perror("SPI: can't create or open dump file ");
            return -1;
        }

        spi_parms->tr.delay_usecs = spi_parms->delay;
        spi_parms->tr.speed_hz = spi_parms->speed;
        spi_parms->tr.bits_per_word = spi_parms->bits;

        if (!spi_parms->ret)
        {
            fprintf(stderr, "-- SPI --\n");
            fprintf(stderr, "SPI mode ............: %d\n", spi_parms->mode);
            fprintf(stderr, "Bits per word .......: %d\n", spi_parms->bits);
            fprintf(stderr, "Interbyte delay .....: %d us\n", spi_parms->delay);
            fprintf(stderr, "Max speed ...........: %d Hz (%d KHz)\n", spi_parms->speed, spi_parms->speed/1000);
        }

    return spi_parms->ret;
}

int spi_write(int fd, int addr, spi_ioc_transfer* value){

    char buff[1024];
    
    int len = sprintf(buff, "%0b\n", addr);
    write(fd, buff, len);
    
    return 1;
}

// ------------------------------------------------------------------------------------------------
int PI_CC_SPIWriteReg(spi_parms_t *spi_parms, uint8_t addr, uint8_t value)
// ------------------------------------------------------------------------------------------------
{
    printf("%02X\n",value);
    spi_parms->tx[0] = addr;
    spi_parms->tx[1] = value;
    spi_parms->tr.len = 2;

    spi_parms->ret = spi_write(spi_parms->fd, SPI_IOC_MESSAGE(1), &spi_parms->tr);

    if (spi_parms->ret < 1)
    {
        fprintf(stderr, "SPI: can't send write register\n");
        return 1;
    }

    return 0;
}

// ------------------------------------------------------------------------------------------------
int PI_CC_SPIWriteBurstReg(spi_parms_t *spi_parms, uint8_t addr, const uint8_t *buffer, uint8_t count)
// ------------------------------------------------------------------------------------------------
{
    uint8_t i;

    count %= 64;
    spi_parms->tx[0] = addr | PI_CCxxx0_WRITE_BURST;   // Send address

    for (i=1; i<count+1; i++)
    {
        spi_parms->tx[i] = buffer[i-1];
    }

    spi_parms->tr.len = count+1;
    spi_parms->ret = spi_write(spi_parms->fd, SPI_IOC_MESSAGE(1), &spi_parms->tr);

    if (spi_parms->ret < 1)
    {
        fprintf(stderr, "SPI: can't send write burst register\n");
        return 1;
    }

    return spi_parms->ret; // returns length sent
}

// ------------------------------------------------------------------------------------------------
int PI_CC_SPIReadReg(spi_parms_t *spi_parms, uint8_t addr, uint8_t *data)
// ------------------------------------------------------------------------------------------------
{
    spi_parms->tx[0] = addr | PI_CCxxx0_READ_SINGLE; // Send address
    spi_parms->tx[1] = 0; // Dummy write so we can read data
    spi_parms->tr.len = 2;

    spi_parms->ret = spi_write(spi_parms->fd, SPI_IOC_MESSAGE(1), &spi_parms->tr);

    if (spi_parms->ret < 1)
    {
        fprintf(stderr, "SPI: can't send read register\n");
        return 1;
    }

    *data = (uint8_t) spi_parms->rx[1];
    return 0;
}

// ------------------------------------------------------------------------------------------------
int PI_CC_SPIReadBurstReg(spi_parms_t *spi_parms, uint8_t addr, byte* buffer, uint8_t count)
// ------------------------------------------------------------------------------------------------
{
    uint8_t i;

    count %= 64;
    spi_parms->tx[0] = addr | PI_CCxxx0_READ_BURST;   // Send address

    for (i=1; i<count+1; i++)
    {
        spi_parms->tx[i] = 0; // Dummy write so we can read data
    }

    spi_parms->tr.len = count+1;
    spi_parms->ret = spi_write(spi_parms->fd, SPI_IOC_MESSAGE(1), &spi_parms->tr);

    if (spi_parms->ret < 1)
    {
        fprintf(stderr, "SPI: can't send read burst register\n");
        return 1;
    }

    memcpy(buffer, &spi_parms->rx, count);
    return 0;
}

// ------------------------------------------------------------------------------------------------
// For status/strobe addresses, the BURST bit selects between status registers
// and command strobes.
int PI_CC_SPIReadStatus(spi_parms_t *spi_parms, uint8_t addr, uint8_t *status)
// ------------------------------------------------------------------------------------------------
{
    spi_parms->tx[0] = addr | PI_CCxxx0_READ_BURST;   // Send address
    spi_parms->tx[1] = 0; // Dummy write so we can read data
    spi_parms->tr.len = 2;

    spi_parms->ret = spi_write(spi_parms->fd, SPI_IOC_MESSAGE(1), &spi_parms->tr);

    if (spi_parms->ret < 1)
    {
        fprintf(stderr, "SPI: can't send read status register\n");
        return 1;
    }

    *status = spi_parms->rx[1];
    return 0;
}

// ------------------------------------------------------------------------------------------------
int PI_CC_SPIStrobe(spi_parms_t *spi_parms, uint8_t strobe)
// ------------------------------------------------------------------------------------------------
{
    spi_parms->tx[0] = strobe;   // Send strobe
    spi_parms->tr.len = 1;

    spi_parms->ret = spi_write(spi_parms->fd, SPI_IOC_MESSAGE(1), &spi_parms->tr);

    if (spi_parms->ret < 1)
    {
        fprintf(stderr, "SPI: can't send strobe %02x", strobe);
        return 1;
    }

    return 0;
}


// ------------------------------------------------------------------------------------------------
int PI_CC_PowerupResetCCxxxx(spi_parms_t *spi_parms)
// ------------------------------------------------------------------------------------------------
{
    return PI_CC_SPIStrobe(spi_parms, PI_CCxxx0_SRES);
}

