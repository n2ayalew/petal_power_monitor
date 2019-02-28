#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int spiGetFd     (int channel) ;
int spiDataRW    (int channel, uint8_t *tx, uint8_t *rx, int len) ;
int spiSetupMode (int channel, int speed, int mode) ;
int spiSetup     (int channel, int speed) ;

#ifdef __cplusplus
}
#endif
