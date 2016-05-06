/* 
 * File:   pixyMessage.h
 * Author: Jyssica Baehr
 *
 * Created on February 9, 2016, 4:48 AM
 */

#ifndef PIXYMESS_H    /* Guard against multiple inclusion */
#define PIXYMESS_H


#ifdef __cplusplus
extern "C" {
#endif

    #define MSG_SIZE 12 // bytes

    
///////////////////////////////////////////////////////////////////////////////
// Message Structure for Pixy Sensor
//  0-1 sync word 
//  2-3 checksum
//  4-5 signature number
//  6-7 x coordinate object center
//  8-9 y coordinate object center
//  10-11 width of object 
//  12-13 height of object
//  
///////////////////////////////////////////////////////////////////////////////
typedef struct {
    uint16_t x;
    uint16_t y;
} grid;
    
    
typedef struct {
    //char checksum_LSB;
    //char checksum_MSB;
    unsigned char signature_LSB;
    unsigned char signature_MSB;
    unsigned char Xcoor_LSB;
    unsigned char Xcoor_MSB;
    unsigned char Ycoor_LSB;
    unsigned char Ycoor_MSB;
    unsigned char width_LSB;
    unsigned char width_MSB;
    unsigned char height_LSB;
    unsigned char height_MSB;
} solidColor;

typedef struct {
    //char checksum_LSB;
    //char checksum_MSB;
    unsigned char signature_LSB;
    unsigned char signature_MSB;
    unsigned char Xcoor_LSB;
    unsigned char Xcoor_MSB;
    unsigned char Ycoor_LSB;
    unsigned char Ycoor_MSB;
    unsigned char width_LSB;
    unsigned char width_MSB;
    unsigned char height_LSB;
    unsigned char height_MSB;
    unsigned char angle_LSB;
    unsigned char angle_MSB;
} colorCode;
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
