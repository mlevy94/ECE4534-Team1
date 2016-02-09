#ifndef _SENSOR_H    /* Guard against multiple inclusion */
#define _SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"

/* Provide C++ Compatibility */
#include "peripheral/adc/plib_adc.h"
//#include "framework/driver/adc/drv_adc_static.h"
#ifdef __cplusplus
extern "C" {
#endif

//function declarations
    uint16_t read_distance();

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */