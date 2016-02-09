#include "sensor.h"


uint16_t read_distance()
{
    //ADC
   /* PLIB_ADC_Enable( ADC_MODULE_ID index );
    PLIB_ADC_SamplingStart( ADC_MODULE_ID index );
    while(!PLIB_ADC_ConversionHasCompleted( ADC_MODULE_ID index ));
    PLIB_ADC_ResultGetByIndex( ADC_MODULE_ID index, uint8_t bufferIndex );*/
    uint16_t ADCOUT;
    uint16_t CONVERSION_FACTOR;
    CONVERSION_FACTOR = 256;
    //PLIB_ADC_Enable(ADC_ID_1);
    //while(!PLIB_ADC_ConversionHasCompleted(ADC_ID_1));
    //setDebugVal('A');
    while(!PLIB_ADC_ConversionHasCompleted(ADC_ID_1));
    ADCOUT = PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0) / CONVERSION_FACTOR;
    
    return ADCOUT;
}

