#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <linux/spi/spidev.h>

#include "spiComm.h"


// The SPI bus parameters
//	Variables as they need to be passed as pointers later on
#define SPI_DEV0 "/dev/spidev0.0"
#define SPI_DEV1 "/dev/spidev0.1"
#define SPI_DEVS 2

//static const char       *spiDev0  = "/dev/spidev0.0";
static const char		*devices[] = {SPI_DEV0, SPI_DEV1};
static uint8_t    spiBPW	  = 8;
static uint16_t	spiDelay = 0 ;

static uint32_t    spiSpeeds [SPI_DEVS];
static int         spiFds [SPI_DEVS] ;

static void pabort(const char *s)
{
	perror(s);
	abort();
}

/*
 * wiringPiSPIGetFd:
 *	Return the file-descriptor for the given channel
 *********************************************************************************
 */

int spiGetFd (int channel)
{
	return spiFds [channel & 1] ;
}

/*
 * spiDataRW:
 *	Write and Read a block of data over the SPI bus.
 *	Note the data ia being read into the read buffer, so rx and tx
 *	length must be same size OR len < ARRAY_SIZE(tx) && len < ARRAY_SIZE(rx)
 *	This is also a full-duplex operation.
 *********************************************************************************
 */

int spiDataRW (int channel, uint8_t *tx, uint8_t *rx, int len)
{
	struct spi_ioc_transfer spi ;

	channel &= 1 ;

	// Mentioned in spidev.h but not used in the original kernel documentation
	//	test program )-:

	memset (&spi, 0, sizeof (spi)) ;

	spi.tx_buf        = (unsigned long)tx ;
	spi.rx_buf        = (unsigned long)rx ;
	spi.len           = len ;
	spi.delay_usecs   = spiDelay ;
	spi.speed_hz      = spiSpeeds [channel] ;
	spi.bits_per_word = spiBPW ;

	int ret = ioctl (spiFds [channel], SPI_IOC_MESSAGE(1), &spi);
	if (ret < 1) pabort("can't send spi message");

	return ret;
}


/*
 * spiSetupMode:
 *	Open the SPI device, and set it up, with the mode, etc.
 *********************************************************************************
 */

int spiSetupMode (int channel, int speed, int mode)
{
	int fd ;

	//mode    &= 3 ;	// Mode is 0, 1, 2 or 3
	//channel &= 1 ;	// Channel is 0 or 1

	mode = 1;
	channel = 0;

	if ((fd = open (devices[channel], O_RDWR)) < 0) {
		pabort("Unable to open SPI device");
		//return -1;
	}

	spiSpeeds [channel] = speed ;
	spiFds    [channel] = fd ;

	// Set SPI parameters.

	if (ioctl (fd, SPI_IOC_WR_MODE, &mode) < 0) {
		pabort("Can't set spi Write mode");
	}

	if (ioctl (fd, SPI_IOC_RD_MODE, &mode) < 0) {
		pabort("Can't get spi Read mode");
	}

	/*
	 * bits per word
	 */
	if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spiBPW) < 0) {
		pabort("can't set bits per word");
	}

	if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &spiBPW) < 0) {
		pabort("can't get bits per word");
	}

	/*
	 * max speed hz
	 */
	if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
		pabort("can't set max speed hz");
	}

	if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
		pabort("can't get max speed hz");
	}

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", spiBPW);
	printf("max speed: %d KHz\n", speed, speed/1000);
	return fd ;
}


/*
 * spiSetup:
 *	Open the SPI device, and set it up, etc. in the default MODE 1
 *********************************************************************************
 */

int spiSetup (int channel, int speed)
{
	return spiSetupMode(channel, speed, 1) ;
}
