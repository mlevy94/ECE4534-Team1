#include "pixy_rx.h"

PIXY_RX_DATA pixy_rxData;
//global variables
int originX = 82;
int originY = 5;
float inches_per_pixel = 36000/176;
float conversion_x;
float conversion_y;
int counter = 0;
int counter_max = 0;
//could make these arrays and average values

BaseType_t addToPixyQFromISR(char data)
{
    setDebugVal(ADD_TO_PIXY_Q);
    return xQueueSendFromISR(pixy_rxData.rxMessageQ, &data, 0);
}

void PIXY_RX_Initialize ( void )
{
    setDebugVal(PIXY_RX_INIT);
    pixy_rxData.state = PIXY_RX_STATE_INIT;
    pixy_rxData.rxMessageQ = xQueueCreate(100, 8);
    pixy_rxData.k = 0;
    pixy_rxData.conversion_x = 0;
    pixy_rxData.conversion_y = 0;
    pixy_rxData.calibration_cycle = 0;
    pixy_rxData.originX = 0;
    pixy_rxData.originY = 0;
    
}

void PIXY_RX_Tasks ( void )
{
    pixy_rxData.val = 0x00;
    while(1)
    {
        switch ( pixy_rxData.state )
        {
            /* Application's initial state. */
            
            case PIXY_RX_STATE_INIT:
            {
//                dataCollection();
                setDebugVal(0x99);
                calibrate();
                
                if (pixy_rxData.k == 16)
                {
                    inchesPerPixel();
                    pixy_rxData.calibration_cycle++;
                }
                pixy_rxData.k = 0;
                
                if (pixy_rxData.calibration_cycle == 100)
                    pixy_rxData.state = COLLECT_DATA;
            }
            break;
            case COLLECT_DATA:
            {
                dataCollection();
            }
            break;

            /* TODO: implement your application state machine.*/

            /* The default state should never be executed. */
            default:
            {
                /* TODO: Handle error in application's state machine. */
                break;
            }
        }
    }
}

void calibrate(void)
{
    pixy_rxData.a = 0;
    pixy_rxData.b = 0;
    //setDebugVal(PIXY_RX_TASKS);
    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
    setDebugVal(0x0000 | pixy_rxData.val);
    if (pixy_rxData.val == 0x55)
    {
        //setDebugVal(0x72);
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        setDebugVal(pixy_rxData.val);
        if (pixy_rxData.val == 0xAA)
        {
            while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
            setDebugVal(pixy_rxData.val);
            while (pixy_rxData.val == 0x55 || pixy_rxData.val == 0x56)
            {
                if (pixy_rxData.val == 0x55)
                {
                    determineEdge();
                }
                else if (pixy_rxData.val == 0x56)
                {
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                }
                //reads in to check for 55 or 56 again
                while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                //setDebugVal(pixy_rxData.val);
            }
        }
    }
}

void determineEdge(void)
{
    
    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
    //setDebugVal(0x0000 | pixy_rxData.val);
    if (pixy_rxData.val == 0xAA)
    {
        //checksum
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));

        //signature
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.signature_LSB = pixy_rxData.val;

        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.signature_MSB = pixy_rxData.val;

        //x coordinate
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.Xcoor_LSB = pixy_rxData.val;

        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.Xcoor_MSB = pixy_rxData.val;
        
        //y coordinate
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.Ycoor_LSB = pixy_rxData.val;
        //setDebugVal(0x0000 | pixy_rxData.SC_message.Ycoor_LSB);

        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.Ycoor_MSB = pixy_rxData.val;
        //setDebugVal(0x0000 | pixy_rxData.SC_message.Ycoor_MSB);

        //width
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.width_LSB = pixy_rxData.val;

        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.width_MSB = pixy_rxData.val;

        //height
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.height_LSB = pixy_rxData.val;

        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.height_MSB = pixy_rxData.val;
        
        //add these values to a queue depending on signature
        //setDebugVal(0x69);
        if (pixy_rxData.SC_message.signature_LSB == 0x05)//obstacle
        {
            
                pixy_rxData.c[pixy_rxData.k] = pixy_rxData.SC_message;
                pixy_rxData.k++;
        }
        //setDebugVal(POST_PIXY_MESS_TO_Q);
  }
}
//////////////////////////////////////////////////
void swap (grid a[16], int left, int right)
{
    grid temp;
    temp = a[left];
    a[left] = a[right];
    a[right] = temp;
}//end swap
void quicksort_y(grid a[16], int low, int high)
{
    int pivot;
    // Termination condition! 
    if ( high > low )
    {
        pivot = partition_y(a, low, high);
        quicksort_y(a, low, pivot-1);
        quicksort_y(a, pivot+1, high);
    }
} //end quicksort
int partition_y( grid a[16], int low, int high )
{
    int left, right;
    grid pivot_item;
    pivot_item = a[low];
    left = low;
    right = high;
    while (left < right)
    {
        // Move left while item < pivot
        while(a[left].y <= pivot_item.y)
            left++;
        // Move right while item > pivot
        while(a[right].y > pivot_item.y)
            right--;
        if (left < right) 
            swap(a,left,right);
    }
    // right is final position for the pivot
    a[low] = a[right];
    a[right] = pivot_item;
    return right;
}//end partition

void quicksort_x(grid a[16], int low, int high)
{
    int pivot;
    // Termination condition! 
    if ( high > low )
    {
        pivot = partition_x(a, low, high);
        quicksort_x(a, low, pivot-1);
        quicksort_x(a, pivot+1, high);
    }
} //end quicksort
int partition_x( grid a[16], int low, int high )
{
    int left, right;
    grid pivot_item;
    pivot_item = a[low];
    left = low;
    right = high;
    while (left < right)
    {
        // Move left while item < pivot
        while(a[left].x <= pivot_item.x)
            left++;
        // Move right while item > pivot
        while(a[right].x > pivot_item.x)
            right--;
        if (left < right) 
            swap(a,left,right);
    }
    // right is final position for the pivot
    a[low] = a[right];
    a[right] = pivot_item;
    return right;
}//end partition
/////////////////////////////////////////////////////////////


void inchesPerPixel(void)
{
    grid g_01, g_02, g_03, g_04;
    grid g_11, g_12, g_13, g_14;
    grid g_21, g_22, g_23, g_24;
    grid g_31, g_32, g_33, g_34;
    
    g_01.x = (pixy_rxData.c[0].Xcoor_MSB<<8 | pixy_rxData.c[0].Xcoor_LSB);
    g_01.y = (pixy_rxData.c[0].Ycoor_MSB<<8 | pixy_rxData.c[0].Ycoor_LSB);
    g_02.x = (pixy_rxData.c[1].Xcoor_MSB<<8 | pixy_rxData.c[1].Xcoor_LSB);
    g_02.y = (pixy_rxData.c[1].Ycoor_MSB<<8 | pixy_rxData.c[1].Ycoor_LSB);
    g_03.x = (pixy_rxData.c[2].Xcoor_MSB<<8 | pixy_rxData.c[2].Xcoor_LSB);
    g_03.y = (pixy_rxData.c[2].Ycoor_MSB<<8 | pixy_rxData.c[2].Ycoor_LSB);
    g_04.x = (pixy_rxData.c[3].Xcoor_MSB<<8 | pixy_rxData.c[3].Xcoor_LSB);
    g_04.y = (pixy_rxData.c[3].Ycoor_MSB<<8 | pixy_rxData.c[3].Ycoor_LSB);
    
    g_11.x = (pixy_rxData.c[4].Xcoor_MSB<<8 | pixy_rxData.c[4].Xcoor_LSB);
    g_11.y = (pixy_rxData.c[4].Ycoor_MSB<<8 | pixy_rxData.c[4].Ycoor_LSB);
    g_12.x = (pixy_rxData.c[5].Xcoor_MSB<<8 | pixy_rxData.c[5].Xcoor_LSB);
    g_12.y = (pixy_rxData.c[5].Ycoor_MSB<<8 | pixy_rxData.c[5].Ycoor_LSB);
    g_13.x = (pixy_rxData.c[6].Xcoor_MSB<<8 | pixy_rxData.c[6].Xcoor_LSB);
    g_13.y = (pixy_rxData.c[6].Ycoor_MSB<<8 | pixy_rxData.c[6].Ycoor_LSB);
    g_14.x = (pixy_rxData.c[7].Xcoor_MSB<<8 | pixy_rxData.c[7].Xcoor_LSB);
    g_14.y = (pixy_rxData.c[7].Ycoor_MSB<<8 | pixy_rxData.c[7].Ycoor_LSB);
    
    g_21.x = (pixy_rxData.c[8].Xcoor_MSB<<8 | pixy_rxData.c[8].Xcoor_LSB);
    g_21.y = (pixy_rxData.c[8].Ycoor_MSB<<8 | pixy_rxData.c[8].Ycoor_LSB);
    g_22.x = (pixy_rxData.c[9].Xcoor_MSB<<8 | pixy_rxData.c[9].Xcoor_LSB);
    g_22.y = (pixy_rxData.c[9].Ycoor_MSB<<8 | pixy_rxData.c[9].Ycoor_LSB);
    g_23.x = (pixy_rxData.c[10].Xcoor_MSB<<8 | pixy_rxData.c[10].Xcoor_LSB);
    g_23.y = (pixy_rxData.c[10].Ycoor_MSB<<8 | pixy_rxData.c[10].Ycoor_LSB);
    g_24.x = (pixy_rxData.c[11].Xcoor_MSB<<8 | pixy_rxData.c[11].Xcoor_LSB);
    g_24.y = (pixy_rxData.c[11].Ycoor_MSB<<8 | pixy_rxData.c[11].Ycoor_LSB);
    
    g_31.x = (pixy_rxData.c[12].Xcoor_MSB<<8 | pixy_rxData.c[12].Xcoor_LSB);
    g_31.y = (pixy_rxData.c[12].Ycoor_MSB<<8 | pixy_rxData.c[12].Ycoor_LSB);
    g_32.x = (pixy_rxData.c[13].Xcoor_MSB<<8 | pixy_rxData.c[13].Xcoor_LSB);
    g_32.y = (pixy_rxData.c[13].Ycoor_MSB<<8 | pixy_rxData.c[13].Ycoor_LSB);
    g_33.x = (pixy_rxData.c[14].Xcoor_MSB<<8 | pixy_rxData.c[14].Xcoor_LSB);
    g_33.y = (pixy_rxData.c[14].Ycoor_MSB<<8 | pixy_rxData.c[14].Ycoor_LSB);
    g_34.x = (pixy_rxData.c[15].Xcoor_MSB<<8 | pixy_rxData.c[15].Xcoor_LSB);
    g_34.y = (pixy_rxData.c[15].Ycoor_MSB<<8 | pixy_rxData.c[15].Ycoor_LSB);
    
    pixy_rxData.calibrationGrid[0] = g_01;
    pixy_rxData.calibrationGrid[1] = g_02;
    pixy_rxData.calibrationGrid[2] = g_03;
    pixy_rxData.calibrationGrid[3] = g_04;
    pixy_rxData.calibrationGrid[4] = g_11;
    pixy_rxData.calibrationGrid[5] = g_12;
    pixy_rxData.calibrationGrid[6] = g_13;
    pixy_rxData.calibrationGrid[7] = g_14;
    pixy_rxData.calibrationGrid[8] = g_21;
    pixy_rxData.calibrationGrid[9] = g_22;
    pixy_rxData.calibrationGrid[10] = g_23;
    pixy_rxData.calibrationGrid[11] = g_24;
    pixy_rxData.calibrationGrid[12] = g_31;
    pixy_rxData.calibrationGrid[13] = g_32;
    pixy_rxData.calibrationGrid[14] = g_33;
    pixy_rxData.calibrationGrid[15] = g_34;
    //pass into a quicksort function
    
    quicksort_y(pixy_rxData.calibrationGrid, 0, 15);
    quicksort_x(pixy_rxData.calibrationGrid, 0, 3);
    quicksort_x(pixy_rxData.calibrationGrid, 4, 7);
    quicksort_x(pixy_rxData.calibrationGrid, 8, 11);
    quicksort_x(pixy_rxData.calibrationGrid, 12, 15);
    //sort y, then x

    
    
    
}

//////////////////////////////////////////////////////////////////////////////////////////
//the following is used for obstacle, token, and rover data collection
//////////////////////////////////////////////////////////////////////////////////////////

void dataCollection(void)
{
    setDebugVal(PIXY_RX_TASKS);
    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
    setDebugVal(0x0000 | pixy_rxData.val);
    if (pixy_rxData.val == 0x55 && counter == 3)
    {
        counter = 0;
        setDebugVal(0x72);
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        setDebugVal(pixy_rxData.val);
        if (pixy_rxData.val == 0xAA)
        {
            
            while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
            setDebugVal(pixy_rxData.val);
            while (pixy_rxData.val == 0x55 || pixy_rxData.val == 0x56)
            {
                if (pixy_rxData.val == 0x55)
                {
                    setDebugVal(SC);
                    addToSolidColorQ();
                }
                else if (pixy_rxData.val == 0x56)
                {
                    setDebugVal(CC);
                    addToColorCodeQ();
                }
                //reads in to check for 55 or 56 again
                while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                setDebugVal(pixy_rxData.val);
            }
        }
    }
    else if (pixy_rxData.val == 0x55)
    {
        counter++;
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        setDebugVal(pixy_rxData.val);
        if (pixy_rxData.val == 0xAA)
        {
            while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
            setDebugVal(pixy_rxData.val);
            while (pixy_rxData.val == 0x55 || pixy_rxData.val == 0x56)
            {
                if (pixy_rxData.val == 0x55)
                {
                    setDebugVal(SC);
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                }
                else if (pixy_rxData.val == 0x56)
                {
                    setDebugVal(CC);
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                }
                //reads in to check for 55 or 56 again
                while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
                setDebugVal(pixy_rxData.val);
            }
        }
    }
}


void addToSolidColorQ(void)//QueueHandle_t pixyQueue)
{
    uint16_t x = 0x0000;
    uint16_t y = 0x0000;
    
    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
    setDebugVal(0x0000 | pixy_rxData.val);
    if (pixy_rxData.val == 0xAA)
    {
        //checksum
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));

        //signature
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.signature_LSB = pixy_rxData.val;

        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.signature_MSB = pixy_rxData.val;

        //x coordinate
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.Xcoor_LSB = pixy_rxData.val;

        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.Xcoor_MSB = pixy_rxData.val;
        
        //y coordinate
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.Ycoor_LSB = pixy_rxData.val;
        setDebugVal(0x0000 | pixy_rxData.SC_message.Ycoor_LSB);

        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.Ycoor_MSB = pixy_rxData.val;
        setDebugVal(0x0000 | pixy_rxData.SC_message.Ycoor_MSB);
        

        //width
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.width_LSB = pixy_rxData.val;

        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.width_MSB = pixy_rxData.val;

        //height
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.height_LSB = pixy_rxData.val;

        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.SC_message.height_MSB = pixy_rxData.val;
        
        //add these values to a queue depending on signature
        if (pixy_rxData.SC_message.signature_LSB == 0x01 || pixy_rxData.SC_message.signature_LSB == 0x02)
            SCMessage(pixy_rxData.SC_message);
        setDebugVal(POST_PIXY_MESS_TO_Q);
  }
}



void SCMessage(solidColor SC_received)//QueueHandle_t SCQueue)
{
    
    
    
    OBJECT_STRUCTURE obj;
    uint16_t height = 0x0000;
    uint16_t width = 0x0000;
    float height_inches;
    float width_inches;
    uint16_t x = 0x0000;
    uint16_t y = 0x0000;
    float centerX_inches;
    float centerY_inches;
    int top_left, top_right, bottom_left, bottom_right;
    float x_unitSquare, y_unitSquare;
    float block_num = 1;
    float x_start, y_start;
    BaseType_t appropriate_obj;
    float block_x[4];
    float block_y[4];
    //while(!xQueueReceive(SCQueue, &pixy_rxData.SC_received, portMAX_DELAY));
    setDebugVal(0x88);
    //////////////////////////////////////////////////////////////////////////////////////////////
    height = 0x0000 | (SC_received.height_MSB<<8) | SC_received.height_LSB;
    width = 0x0000 | (SC_received.width_MSB<<8) | SC_received.width_LSB;
    height_inches = height*inches_per_pixel/100;//pixy_rxData.conversion_y/100;//inches_per_pixel/100;
    width_inches = width*inches_per_pixel/100;//pixy_rxData.conversion_x/100;//inches_per_pixel/100;
    //////////////////////////////////////////////////////////////////////////////////////////////
    x = (SC_received.Xcoor_MSB<<8) | SC_received.Xcoor_LSB;
    y = (SC_received.Ycoor_MSB<<8) | SC_received.Ycoor_LSB;
    
//    centerX_inches = x*inches_per_pixel/100;//pixy_rxData.conversion_x/100;//inches_per_pixel/100;//divide by 10
//    centerY_inches = y*inches_per_pixel/100;//pixy_rxData.conversion_y/100;//inches_per_pixel/100;//divide by 10
    ///////////////////////////////////////////////////////////////////////////////
    
    top_left = 0;
    top_right = 1;
    bottom_left = 4;
    bottom_right = 5;
    appropriate_obj = pdFALSE;
////    bool val;
//    val = checkBlock(top_left, top_right, bottom_left, bottom_right, x, y);
    int i;
    for (i = 0; i < 9; i++)
    {
        if (checkBlock(top_left, top_right, bottom_left, bottom_right, x, y) == pdFALSE)//checkBlock(block_x, block_y, (float)x, (float)y) == pdFALSE)
        {
            if (top_left == 2 || top_left == 6 || top_left == 10 || top_left == 14)
            {
                top_left = top_left + 2;
                top_right = top_right + 2;
                bottom_left = bottom_left + 2;
                bottom_right = bottom_right + 2;
            }
            else
            {
                top_left++;
                top_right++;
                bottom_left++;
                bottom_right++;
            }
            block_num++;
        }
        else
        {
            appropriate_obj = pdTRUE;
            break;
        }
    }
    
    
//    
////    //set all variables to be used
    if (appropriate_obj == pdTRUE)
        {
        pixy_rxData.fx00 = (float)pixy_rxData.calibrationGrid[top_left].x;
        pixy_rxData.fy00 = (float)pixy_rxData.calibrationGrid[top_left].y;

        pixy_rxData.fx10 = (float)pixy_rxData.calibrationGrid[top_right].x;
        pixy_rxData.fy10 = (float)pixy_rxData.calibrationGrid[top_right].y;

        pixy_rxData.fx01 = (float)pixy_rxData.calibrationGrid[bottom_left].x;
        pixy_rxData.fy01 = (float)pixy_rxData.calibrationGrid[bottom_left].y;

        pixy_rxData.fx11 = (float)pixy_rxData.calibrationGrid[bottom_right].x;
        pixy_rxData.fy11 = (float)pixy_rxData.calibrationGrid[bottom_right].y;
        //big constant in equations
        pixy_rxData.interpolationConstant_A = pixy_rxData.fx00 - pixy_rxData.fx10 - pixy_rxData.fx01 + pixy_rxData.fx11;
        pixy_rxData.interpolationConstant_B = pixy_rxData.fy00 - pixy_rxData.fy10 - pixy_rxData.fy01 + pixy_rxData.fy11;
    ////    
    ////    //function call to calculate y, then x
        y_unitSquare = y_interpolation((float)x, (float)y);
        x_unitSquare = x_interpolation((float)x, y_unitSquare);
        if (block_num == 1 || block_num == 2 || block_num == 3)
            y_start = 0.0;
        else if (block_num == 4 || block_num == 5 || block_num == 6)
            y_start = 130.0;
        else if (block_num == 7 || block_num == 8 || block_num == 9)
            y_start = 260.0;
        if (block_num == 1 || block_num == 4 || block_num == 7)
            x_start = 0.0;
        else if (block_num == 2 || block_num == 5 || block_num == 8)
            x_start = 130.0;
        else if (block_num == 3 || block_num == 6 || block_num == 9)
            x_start = 260.0;
    //    


        centerX_inches = x_start+(13.0*x_unitSquare)*10.0-15.0;
        centerY_inches = y_start+(13.0*y_unitSquare)*10.0-15.0;
    //    
    //    
    //    
    //    
        if (SC_received.signature_LSB == 0x01)
        {
            obj.type = OBSTACLE;
            obj.xPos = (uint16_t)height_correction(centerX_inches, 40.0);
            obj.yPos = (uint16_t)height_correction(centerY_inches, 40.0);
        }
        else if (SC_received.signature_LSB == 0x02)
        {
            obj.type = TOKEN;
            obj.xPos = (uint16_t)centerX_inches;
            obj.yPos = (uint16_t)centerY_inches;
        }
        
        //obj.xPos = (uint16_t)centerX_inches;
        //obj.yPos = (uint16_t)centerY_inches;
        obj.angle = 0x0000;
        obj.width = (uint16_t)width_inches;
        obj.length = (uint16_t)height_inches;

        

        setDebugBool(pdTRUE);
        setDebugVal(obj.xPos>>8);
        setDebugVal(obj.xPos);
        setDebugVal(0x55);
        setDebugVal(obj.yPos>>8);
        setDebugVal(obj.yPos);
        setDebugVal(0x55);
        setDebugBool(pdFALSE);
        
        addToUartTXQ(makeLocationMessage(obj));
//        addToUartTXQ(debugMessage(obj));
    }
}

void addToColorCodeQ(void)//QueueHandle_t pixyQueue)
{
    uint16_t x = 0x0000;
    uint16_t y = 0x0000;
    
    while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
    setDebugVal(0x0000 | pixy_rxData.val);
    if (pixy_rxData.val == 0xAA)
    {
        //checksum
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));

        //signature
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.CC_message.signature_LSB = pixy_rxData.val;

        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.CC_message.signature_MSB = pixy_rxData.val;
        
        //x coordinate
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.CC_message.Xcoor_LSB = pixy_rxData.val;
        
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.CC_message.Xcoor_MSB = pixy_rxData.val;
        
        
        
        //y coordinate
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.CC_message.Ycoor_LSB = pixy_rxData.val;

        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.CC_message.Ycoor_MSB = pixy_rxData.val;
        
        
        //width
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.CC_message.width_LSB = pixy_rxData.val;

        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.CC_message.width_MSB = pixy_rxData.val;

        //height
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.CC_message.height_LSB = pixy_rxData.val;

        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.CC_message.height_MSB = pixy_rxData.val;
        
        //angle
        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.CC_message.angle_LSB = pixy_rxData.val;

        while(!xQueueReceive(pixy_rxData.rxMessageQ, &pixy_rxData.val, portMAX_DELAY));
        pixy_rxData.CC_message.angle_MSB = pixy_rxData.val;
        
        //add these values to a queue depending on signature
        CCMessage(pixy_rxData.CC_message);
        setDebugVal(POST_PIXY_MESS_TO_Q);
  }
}

void CCMessage(colorCode CC_received)//QueueHandle_t CCQueue)//
{
    OBJECT_STRUCTURE obj;
    
    uint16_t angle;
    int16_t theta;
    uint16_t height = 0x0000;
    uint16_t width = 0x0000;
    float height_inches;
    float width_inches;
    uint16_t x = 0x0000;
    uint16_t y = 0x0000;
    float centerX_inches;
    float centerY_inches;
    int top_left, top_right, bottom_left, bottom_right;
    float x_unitSquare, y_unitSquare;
    float block_num = 1;
    float x_start, y_start;
    BaseType_t appropriate_obj;
    float block_x[4];
    float block_y[4];
    //while(!xQueueReceive(CCQueue, &pixy_rxData.CC_received, portMAX_DELAY));
    
    height = (CC_received.height_MSB<<8) | CC_received.height_LSB;
    width = (CC_received.width_MSB<<8) | CC_received.width_LSB;
    height_inches = height*inches_per_pixel/100;//pixy_rxData.conversion_y/100;//inches_per_pixel/100;
    width_inches = width*inches_per_pixel/100;//pixy_rxData.conversion_x/100;//inches_per_pixel/100;
    
    x = (CC_received.Xcoor_MSB<<8) | CC_received.Xcoor_LSB;
    y = (CC_received.Ycoor_MSB<<8) | CC_received.Ycoor_LSB;
//    centerX_inches = 360*x_interpolation(x, y);
//    centerY_inches = y_interpolation(x, y);
//    centerX_inches = x*inches_per_pixel/100;//pixy_rxData.conversion_x/100;//inches_per_pixel/100;//divide by 10
//    centerY_inches = y*inches_per_pixel/100;//pixy_rxData.conversion_y/100;//inches_per_pixel/100;//divide by 10
    top_left = 0;
    top_right = 1;
    bottom_left = 4;
    bottom_right = 5;
    appropriate_obj = pdFALSE;
//    bool val;
//    val = checkBlock(top_left, top_right, bottom_left, bottom_right, x, y);
    int i;
    for (i = 0; i < 9; i++)
    {
        if (checkBlock(top_left, top_right, bottom_left, bottom_right, x, y) == pdFALSE)//checkBlock(block_x, block_y, (float)x, (float)y) == pdFALSE)
        {
            if (top_left == 2 || top_left == 6 || top_left == 10 || top_left == 14)
            {
                top_left = top_left + 2;
                top_right = top_right + 2;
                bottom_left = bottom_left + 2;
                bottom_right = bottom_right + 2;
            }
            else
            {
                top_left++;
                top_right++;
                bottom_left++;
                bottom_right++;
            }
            block_num++;
        }
        else
        {
            appropriate_obj = pdTRUE;
            break;
        }
    }
    
    
    
//    //set all variables to be used
    if (appropriate_obj == pdTRUE)
    {
        pixy_rxData.fx00 = (float)pixy_rxData.calibrationGrid[top_left].x;
        pixy_rxData.fy00 = (float)pixy_rxData.calibrationGrid[top_left].y;

        pixy_rxData.fx10 = (float)pixy_rxData.calibrationGrid[top_right].x;
        pixy_rxData.fy10 = (float)pixy_rxData.calibrationGrid[top_right].y;

        pixy_rxData.fx01 = (float)pixy_rxData.calibrationGrid[bottom_left].x;
        pixy_rxData.fy01 = (float)pixy_rxData.calibrationGrid[bottom_left].y;

        pixy_rxData.fx11 = (float)pixy_rxData.calibrationGrid[bottom_right].x;
        pixy_rxData.fy11 = (float)pixy_rxData.calibrationGrid[bottom_right].y;
        //big constant in equations
        pixy_rxData.interpolationConstant_A = pixy_rxData.fx00 - pixy_rxData.fx10 - pixy_rxData.fx01 + pixy_rxData.fx11;
        pixy_rxData.interpolationConstant_B = pixy_rxData.fy00 - pixy_rxData.fy10 - pixy_rxData.fy01 + pixy_rxData.fy11;

    //    //function call to calculate y, then x
        y_unitSquare = y_interpolation((float)x, (float)y);
        x_unitSquare = x_interpolation((float)x, y_unitSquare);
        //determine inches before the current block
        if (block_num == 1 || block_num == 2 || block_num == 3)
            y_start = 0.0;
        else if (block_num == 4 || block_num == 5 || block_num == 6)
            y_start = 130.0;
        else if (block_num == 7 || block_num == 8 || block_num == 9)
            y_start = 260.0;
        if (block_num == 1 || block_num == 4 || block_num == 7)
            x_start = 0.0;
        else if (block_num == 2 || block_num == 5 || block_num == 8)
            x_start = 130.0;
        else if (block_num == 3 || block_num == 6 || block_num == 9)
            x_start = 260.0;
    //    


        centerX_inches = x_start+(13.0*x_unitSquare)*10.0-12.5;
        centerY_inches = y_start+(13.0*y_unitSquare)*10.0-12.5;

        angle = (CC_received.angle_MSB<<8) | CC_received.angle_LSB;
        theta = (int16_t)angle;
        if (theta < 0)
            theta = -theta + 90;
        else if (theta > 0)
        {
            theta = 360 - theta + 90;
        }
        if (theta > 359)
                theta = theta - 360;

        
        if (CC_received.signature_LSB == 0x1C)
        {
            obj.type = LEAD;
            obj.xPos = (uint16_t)height_correction(centerX_inches, 57.0);
            obj.yPos = (uint16_t)height_correction(centerY_inches, 57.0);
        }
        else if (CC_received.signature_LSB == 0x26)
        {
            obj.type = FOLLOW;
            obj.xPos = (uint16_t)centerX_inches;
            obj.yPos = (uint16_t)centerY_inches;
        }
            
        obj.angle = (uint16_t)theta;
        obj.width = (uint16_t)width_inches;
        obj.length = (uint16_t)height_inches;
        
        setDebugBool(pdTRUE);
        setDebugVal(obj.xPos>>8);
        setDebugVal(obj.xPos);
        setDebugVal(0x55);
        setDebugVal(obj.yPos>>8);
        setDebugVal(obj.yPos);
        setDebugVal(0x55);
        setDebugBool(pdFALSE);
        
        addToUartTXQ(makeLocationMessage(obj));
//        addToUartTXQ(debugMessage(obj));
    }
}

BaseType_t checkBlock(int TL, int TR, int BL, int BR, uint16_t x, uint16_t y)
{
    if (x > pixy_rxData.calibrationGrid[TL].x && x < pixy_rxData.calibrationGrid[TR].x
            && x > pixy_rxData.calibrationGrid[BL].x && x < pixy_rxData.calibrationGrid[BR].x
            && y > pixy_rxData.calibrationGrid[TL].y && y < pixy_rxData.calibrationGrid[BL].y
            && y > pixy_rxData.calibrationGrid[TR].y && y < pixy_rxData.calibrationGrid[BR].y)
        return pdTRUE;
    else
        return pdFALSE;
}




float y_interpolation(float x, float y)
{
    float a, b, c;
    float quad_add_top, quad_add;
    float quad_neg_top, quad_neg;
    float y_interp, quad_bottom;
    a = pixy_rxData.interpolationConstant_A*(pixy_rxData.fy00-pixy_rxData.fy01) +
            pixy_rxData.interpolationConstant_B*(pixy_rxData.fx00-pixy_rxData.fx01);
    b = pixy_rxData.interpolationConstant_A*pixy_rxData.fy00 +
            (pixy_rxData.fx00-pixy_rxData.fx01)*(pixy_rxData.fy00-pixy_rxData.fy10) -
            (pixy_rxData.fx00-pixy_rxData.fx10)*(pixy_rxData.fy00-pixy_rxData.fy01) +
            pixy_rxData.interpolationConstant_B*(pixy_rxData.fx00-x) - pixy_rxData.interpolationConstant_A*y;
    c = y*(pixy_rxData.fx00-pixy_rxData.fx10) - pixy_rxData.fy00*(pixy_rxData.fx00-pixy_rxData.fx10) +
            (pixy_rxData.fy00-pixy_rxData.fy10)*(pixy_rxData.fx00-x);
    if (a == 0)
        y_interp = fabs(-c/b);
    else
    {
        quad_bottom = 2*a;
        quad_add_top = -b+sqrt(pow(b,2.0)-4*a*c);
        quad_neg_top = -b-sqrt(pow(b,2.0)-4*a*c);
        quad_add = quad_add_top/quad_bottom;
        quad_neg = quad_neg_top/quad_bottom;
        //determine factor from 2 values from quadratic equation
        if(fabs(quad_add) > 0.0 && fabs(quad_add) < 1.0)
        {
            y_interp = fabs(quad_add);
        }
        else if (fabs(quad_neg) > 0.0 && fabs(quad_neg) < 1.0)
        {
            y_interp = fabs(quad_neg);
        }
    }
    return y_interp;//(-c/b);
}

float x_interpolation(float x, float y_interpolated)
{
//store all values initially
    float top;
    float bottom;
    top = pixy_rxData.fx00 - x - y_interpolated*(pixy_rxData.fx00-pixy_rxData.fx01);
    bottom = pixy_rxData.fx00-pixy_rxData.fx10+y_interpolated*pixy_rxData.interpolationConstant_A;
    return (top/bottom);
}
//height adjustment equation
float height_correction(float j_a, float object_height)
{
    float z0, j_r;
    
    z0 = 487.5;
    if (j_a > 180.0)
        j_r = j_a - (object_height*fabs(j_a-180.0))/(z0-object_height);
    else
        j_r = j_a + (object_height*fabs(j_a-180.0))/(z0-object_height);
    return j_r;
}

