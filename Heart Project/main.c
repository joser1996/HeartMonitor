// Standard includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "spi.h"
#include "utils.h"
#include "timer.h"
#include "gpio.h"
// Common interface includes

#include "uart_if.h"
#include "i2c_if.h"
#include "algorithm.h"
#include "max30102.h"
#include "pinmux.h"
#include "gpio_if.h"

//OLED Lib
#include "test.h"
#include "Adafruit_OLED.h"
#include "glcdfont.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"


//OUR headers
//#include "heartOLED.h"

//GLOBAL VARIABLES -- Start
int warning = 0;
int maxHeartRate = 10;
int maxWarning = 1;
unsigned int n_ir_buffer_length; //data length
unsigned int aun_ir_buffer[100]; //IR LED sensor data
unsigned int aun_red_buffer[100]; //RED LED sensor data
unsigned int n_spo2; //SPO2 value
signed char ch_spo2_valid; //indicator to show if valid
int n_heart_rate;   //heart rate value
signed char ch_hr_valid; //show if heart rate value is valid
unsigned char uch_dummy;
#define MAX_BRIGHTNESS 235


//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define APPLICATION_VERSION     "1.1.1"
#define APP_NAME                "I2C Demo"
#define UART_PRINT              Report
#define FOREVER                 1
#define CONSOLE                 UARTA0_BASE
#define FAILURE                 -1
#define SUCCESS                 0
#define UartPutChar(c)       MAP_UARTCharPut(CONSOLE,c)
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}


#define SPI_IF_BIT_RATE  500000


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif


int avgBufferSize = 200;
int last = 0;
int avgBuff[200];
int itr = 0;
int filling = 1;
char* numBuf;
// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

#define MILLISECONDS_TO_TICKS(ms) ((80000000 / 1000) * (ms))

enum state{Standing, Sitting};
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************


static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}
void drawHeaders()
{
    int xc = 20;
    int k = 0;

    char* hr = "HR";
    char* Ox = "O2";
    SPIEnable(GSPI_BASE);
    for(k = 0; k < 2; ++k){

        drawChar(xc, 20, hr[k], WHITE, BLACK, 2);
        xc += 12;
    }
//    xc = 90;
//    for(k = 0; k < 2; ++k){
//        drawChar(xc, 20, Ox[k], WHITE, BLACK, 2);
//        xc += 12;
//    }
    SPIDisable(GSPI_BASE);
}




void displayHeartInfo(int hr, int ox){
    int sb = 0;
    int xcur = 20, ycur = 40;
    char* numBuf = malloc(30 * sizeof(char));
    SPIEnable(GSPI_BASE);
    fillRect(20, 40, 110, 17, BLACK);
    sprintf(numBuf, "%d", hr);
    while(numBuf[sb] != '\0'){
        drawChar(xcur, ycur, numBuf[sb], WHITE, BLACK, 2);
        xcur += 12;
        sb++;
    }
//    sb = 0;
//    xcur = 90;
//
//    sprintf(numBuf, "%d", ox);
//    while(numBuf[sb] != '\0'){
//        drawChar(xcur, ycur, numBuf[sb], WHITE, BLACK, 2);
//        xcur += 12;
//        sb++;
//    }
    sb = 0;
    xcur = 20;
    delay(5);
    SPIDisable(GSPI_BASE);
    free(numBuf);
}


int sendHR2UART() {
   // Message("Here\r\n");
    int value;
    char value_bit[sizeof(int)];
    if (n_heart_rate == -999)
        value = 0;
    else if (n_heart_rate <= 130 && n_heart_rate >= 50){
        value = n_heart_rate;
        last = n_heart_rate;
    }
    else
        value = last;

    avgBuff[itr] = value;

    if ((itr == avgBufferSize - 1)) {
        itr = 0;
        if (filling == 1)
            filling = 0;
    }
    itr++;

    if (filling == 0) {
        int i;
        int sum = 0;
        int send;
        //Message("Here for\r\n");
        for (i = 0; i < avgBufferSize; i++) {
            sum += avgBuff[i];
        }
        send =  sum / avgBufferSize;
        memcpy(value_bit,&value,sizeof(int));
        //Message("Here after for\r\n");
        for (i = 0; i < sizeof(int); i++) {
            UartPutChar(value_bit[i]);
        }
        //Report("%d \n\r", send); //uncomment to test on TeraTerm
        displayHeartInfo(send, 0);

        if (warning == maxWarning) {
            GPIOPinWrite(GPIOA3_BASE, 0x1, 0x1);
            warning = 0;
        }

        if (send >= maxHeartRate) {
            warning++;
        }
        else {
            warning = 0;
        }

//        if(GPIOPinRead(GPIOA2_BASE,0x40)) {
//            GPIOPinWrite(GPIOA3_BASE, 0x1, 0x0);
//        }

    }
    return last;
}

unsigned char * readCord(int cord) {
    unsigned char ucDevAddr, ucRegOffset;
    unsigned char aucRdDataBuf[256];
    //char *pcErrPtr;

    ucDevAddr = 0x18;
    if (cord == 0) ucRegOffset = 0x03; //x
    else if (cord == 1) ucRegOffset = 0x05; //y
    else if (cord == 2) ucRegOffset = 0x07; //z

    // Write the register address to be read from.
    // Stop bit implicitly assumed to be 0.
    RET_IF_ERR(I2C_IF_Write(ucDevAddr,&ucRegOffset,1,0));
    // Read the specified length of data
    RET_IF_ERR(I2C_IF_Read(ucDevAddr, &aucRdDataBuf, 1));
    //DisplayBuffer(aucRdDataBuf, 1);

    return aucRdDataBuf;
}

void collect_raw(short int * raw_x, short int * raw_y) {
    unsigned char * x;
    unsigned char * y;
    short int zero = 0;
    short int holder;

    // read x coordinate
    x = readCord(0);
    holder = x[0] | zero;
    holder = holder << 8;
    holder /= 1000;
    *raw_x = holder;

    // read y coordinate
    y = readCord(1);
    holder = y[0] | zero;
    holder = holder << 8;
    holder /= 1000;
    *raw_y = holder;
}

void oledPrintState(enum state st){
    char* sit = "sitting";
    char* stand = "standing";
    int i = 0;
    int xc = 20;
    SPIEnable(GSPI_BASE);
    //Erasing previous values
    fillRect(20, 110, 110, 17, BLACK);
    //Message("Drawing State \r\n");
    if(st == Standing){
        for(i = 0; i < 8; i++){
            drawChar(xc,110, stand[i], WHITE, BLACK, 2);
            xc += 12;
        }
    }
    else{
        for(i = 0; i < 7; i++){
            drawChar(xc,110, sit[i], WHITE, BLACK, 2);
            xc += 12;
        }
    }
    SPIDisable(GSPI_BASE);
}

void userPosition(){
//    TimerIntDisable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
    //Message("In Handler");

    short int raw_x, raw_y;
    int conf= 0;
    enum state st; //
    enum state prevState;
    int done = 0;
    while(done == 0){
        //get gyro. value
        collect_raw(&raw_x, &raw_y);
        //Report("X: %hd, Y: %hd \n\r", raw_x, raw_y);

        if((raw_y >= 14) || (raw_y <= -14)){
            //Message("Standing \n\r");
            st = Standing;
        }
        else if((raw_y >= -3) && (raw_y <= 3)){
            //Message("Sitting \n\r");
            st = Sitting;
        }
        else{
            //Message("Moving \n\r");
            ;
        }

        if(conf == 0){
            prevState = st;
            conf++;
        }
        else if(conf == 10){
            conf = 0;
            oledPrintState(prevState);
            done = 1;
        }
        else if(prevState == st){
            conf++;
            prevState = st;
        }
        else{
            conf = 0;
        }
    }

}

void timerInit(){
    PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
    PRCMPeripheralReset(PRCM_TIMERA0);
    //Timer used to check wether standing or sitting
    TimerConfigure(TIMERA0_BASE,TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMERA0_BASE,TIMER_A, MILLISECONDS_TO_TICKS(5000)); //Want to sample at 16 kHz
    TimerEnable(TIMERA0_BASE, TIMER_A);

    //Enabling Interruts for our timer
    TimerIntRegister(TIMERA0_BASE, TIMER_A, userPosition);
    TimerIntEnable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
}

void initMaster(void)
{
    //Some Init Stuffs
    BoardInit();
    PinMuxConfig();
    InitTerm();
    ClearTerm();

    //Configure UART clock
    UARTConfigSetExpClk(UARTA1_BASE,PRCMPeripheralClockGet(PRCM_UARTA1),9600, (UART_CONFIG_WLEN_8
            | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));


    UARTEnable(UARTA1_BASE);
    UARTFIFODisable(UARTA1_BASE);


    //Enable SPI to communicate with the OLED
    PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
    PRCMPeripheralReset(PRCM_GSPI);
    SPIReset(GSPI_BASE);
    SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));
    SPIEnable(GSPI_BASE);
    Adafruit_Init();
    fillScreen(BLACK);
    SPIDisable(GSPI_BASE);

    //Initialize the timer
    //timerInit();

    I2C_IF_Open(I2C_MASTER_MODE_FST);


    //reset device
    if(maxim_max30102_reset() != 0)
        return -1;
        //Message("Error Reseting the device \n\r");

    //Clears INT status, but we don't use INT so  maybe we don't need it
    if(maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_dummy) != 0)
        return -1;
        //Message("Error clearing INT status reg\n\r");

    //Initialize Device
    if(maxim_max30102_init() != 0)
        return -1;
        //Message("Error Initializing the device \n\r");

    GPIOPinWrite(GPIOA3_BASE, 0x1, 0x0);
}
//*****************************************************************************
//
//! Main function handling the I2C example
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
void main()
{
//    int iRetVal;
//    char acCmdStore[512];
//    unsigned char uchdummy;
    initMaster();

    unsigned int un_min, un_max, un_prev_data, un_brightness; //used to calculate LED brightness
    unsigned int i;
    int f_temp;

    int avgHr;
    int avgOx;
    int avgCount = 0;
    long int Ox = 0;
    numBuf = malloc(30 * sizeof(char));
    int counterState = 0;
    //draw OLED headers
    SPIEnable(GSPI_BASE);
    drawHeaders();
    SPIDisable(GSPI_BASE);

    while (!GPIOPinRead(GPIOA2_BASE,0x40)) {
        continue;
    }

    while(1){

        un_brightness = 0;
        un_min = 0x3FFFF;
        un_max = 0;

        n_ir_buffer_length = 100; //buffer len. of 100 that stores 4 seconds of samples running at 25sps

        for(i = 0; i < n_ir_buffer_length; i++){

            while(GPIOPinRead(GPIOA3_BASE, 0x40) == 0x40);

            maxim_max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i)); //read from fifo

            //update signal max. and min.
            if(un_min > aun_red_buffer[i])
                un_min = aun_red_buffer[i];
            if(un_max < aun_red_buffer[i])
                un_max = aun_red_buffer[i];

            //Report("Red = %d \t , IR= %d\r\n",aun_red_buffer[i], aun_ir_buffer[i]);

        }//End for loop

        un_prev_data = aun_red_buffer[i];

        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer,
                                     &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

//        Report("Heart Rate: %d\n\r", n_heart_rate);
//        Report("SPO2: %d\n\r", n_spo2);


        //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
        while(1)
        {
          i=0;
          un_min=0x3FFFF;
          un_max=0;

          //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
          for(i=25;i<100;i++)
          {
            aun_red_buffer[i-25]=aun_red_buffer[i];
            aun_ir_buffer[i-25]=aun_ir_buffer[i];

            //update the signal min and max
            if(un_min>aun_red_buffer[i])
              un_min=aun_red_buffer[i];
            if(un_max<aun_red_buffer[i])
              un_max=aun_red_buffer[i];
          }

          //take 25 sets of samples before calculating the heart rate.
          for(i=75;i<100;i++)
          {
            un_prev_data=aun_red_buffer[i-1];
            //Message("Enter\r\n");
            while(GPIOPinRead(GPIOA3_BASE, 0x40) ==  0x40);
            //Message("Reading FIFO\r\n");
            maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));

            //calculate the brightness of the LED
            if(aun_red_buffer[i]>un_prev_data)
            {
              f_temp=aun_red_buffer[i]-un_prev_data;
              f_temp/=(un_max-un_min);
              f_temp*=MAX_BRIGHTNESS;
              f_temp=un_brightness-f_temp;
              if(f_temp<0)
                un_brightness=0;
              else
                un_brightness=(int)f_temp;
            }
            else
            {
              f_temp=un_prev_data-aun_red_buffer[i];
              f_temp/=(un_max-un_min);
              f_temp*=MAX_BRIGHTNESS;
              un_brightness+=(int)f_temp;
              if(un_brightness>MAX_BRIGHTNESS)
                un_brightness=MAX_BRIGHTNESS;
            }

            //Report("Red = %d \t , IR= %d\t",aun_red_buffer[i], aun_ir_buffer[i]); //format data ???

            //Report(", HR= %d\t", n_heart_rate);
            last = sendHR2UART(n_heart_rate);
            ++counterState;
            //get user state every 30 iterations
            if(counterState == 30){
                counterState = 0;
                userPosition();
            }
            //Report(", HRvalid= %d\t", (int)ch_hr_valid);

            //Report(", SPO2= %ld\t", n_spo2);

            //Report(", SPO2Valid = %d\r\n", (int)ch_spo2_valid);
          }
          maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
          //display heart info on OLED

        }//End of nested while loop

    }//End while loop



}
