#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_timer.h"
#include "hw_ints.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "spi.h"
#include "timer.h"
#include "pin.h"
#include "Adafruit_SSD1351.h"
#include "gpio.h"
#include "Adafruit_OLED.h"
#include "test.h"
#include "glcdfont.h"
#include "Adafruit_GFX.h"
#include "timer_if.h"

#include <stdbool.h>
//Common interface includes
#include "pinmux.h"
#include "gpio_if.h"
#include "uart_if.h"
#include "uart.h"
#include "Ourcommon.h"
#include "systick_if.h"

#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1


#define APPLICATION_NAME        "SSL"
#define APPLICATION_VERSION     "1.1.1.EEC.Spring2018"
#define SERVER_NAME             "apsxfivnxf0hp-ats.iot.us-west-2.amazonaws.com"
#define GOOGLE_DST_PORT         8443

#define SL_SSL_CA_CERT "/cert/rootCA.der" //starfield class2 rootca (from firefox) // <-- this one works
#define SL_SSL_PRIVATE "/cert/private.der"
#define SL_SSL_CLIENT  "/cert/client.der"


//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                12    /* Current Date */
#define MONTH               3    /* Month 1-12 */
#define YEAR                2019  /* Current year */
#define HOUR                10    /* Time - hours */
#define MINUTE              39    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define POSTHEADER "POST /things/cc3200-thing/shadow HTTP/1.1\n\r"
#define GETHEADER "GET /things/cc3200-thing/shadow HTTP/1.1\n\r"
#define HOSTHEADER "Host: apsxfivnxf0hp-ats.iot.us-west-2.amazonaws.com\r\n"
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"


#define DATA1 "{\"state\": {\r\n\"desired\" : {\r\n\"var\" : \"Hello phone, message from CC3200 via AWS IoT!\"\r\n}}}\r\n\r\n"

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

typedef struct
{
   /* time */
   unsigned long tm_sec;
   unsigned long tm_min;
   unsigned long tm_hour;
   /* date */
   unsigned long tm_day;
   unsigned long tm_mon;
   unsigned long tm_year;
   unsigned long tm_week_day; //not required
   unsigned long tm_year_day; //not required
   unsigned long reserved[3];
}SlDateTime;


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile int textTimeOut = 0;
volatile int sampleBuffer[411];
volatile int readyF = 0;
volatile int sampleCount = 0;
volatile int power_all[8];
volatile bool new_dig;
volatile long id = -1;
// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

int coeffecients[8] = {31548, 31281, 30950, 30556, 29143, 28360, 27408, 26258};
int i,row,col,max_power;

 char row_col[4][4] =
    {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
    };


volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulPingPacketsRecv = 0; //Number of Ping Packets received
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
signed char    *g_Host = SERVER_NAME;
SlDateTime g_time;
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End: df
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static long WlanConnect();
static int set_time();
static void BoardInit(void);
static long InitializeAppVariables();
static int tls_connect();
static int connectToAccessPoint();
static int http_post(int, char*);

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
    if(!pWlanEvent) {
        return;
    }

    switch(pWlanEvent->Event) {
        case SL_WLAN_CONNECT_EVENT: {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'.
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

//            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
//                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
//                       g_ucConnectionSSID,g_ucConnectionBSSID[0],
//                       g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
//                       g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
//                       g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT: {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code) {
                //UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
//                    "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
//                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
//                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
//                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
//                           g_ucConnectionBSSID[5]);
            }
            else {
//                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s, "
//                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
//                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
//                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
//                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
//                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default: {
//            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
//                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    if(!pNetAppEvent) {
        return;
    }

    switch(pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT: {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

//            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
//                       "Gateway=%d.%d.%d.%d\n\r",
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
        }
        break;

        default: {
//            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
//                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse) {
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {
    if(!pDevEvent) {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
//    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
//               pDevEvent->EventData.deviceEvent.status,
//               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
    if(!pSock) {
        return;
    }

    switch( pSock->Event ) {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status) {
                case SL_ECLOSE: 
//                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
//                                "failed to transmit all queued packets\n\n",
//                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default: 
//                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
//                                "(%d) \n\n",
//                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
//            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End breadcrumb: s18_df
//*****************************************************************************


//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    0 on success else error code
//!
//! \return None
//!
//*****************************************************************************
static long InitializeAppVariables() {
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    g_Host = SERVER_NAME;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    return SUCCESS;
}


//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState() {
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode 
    if (ROLE_STA != lMode) {
        if (ROLE_AP == lMode) {
            // If the device is in AP mode, we need to wait for this event 
            // before doing anything 
            while(!IS_IP_ACQUIRED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
            }
        }

        // Switch to STA role and restart 
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again 
        if (ROLE_STA != lRetVal) {
            // We don't want to proceed if the device is not coming up in STA-mode 
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }
    
    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, 
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);
    
//    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
//    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
//    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
//    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
//    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
//    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
//    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig 
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, 
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore 
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal) {
        // Wait
        while(IS_CONNECTED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, 
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();
    
    return lRetVal; // Success
}


//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void BoardInit(void) {
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


//****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  None
//!
//! \return  0 on success else error code
//!
//! \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect() {
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

//    UART_PRINT("Attempting connection to access point: ");
//    UART_PRINT(SSID_NAME);
//    UART_PRINT("... ...");
    lRetVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

//    UART_PRINT(" Connected!!!\n\r");


    // Wait for WLAN Event
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
        // Toggle LEDs to Indicate Connection Progress
        _SlNonOsMainLoopTask();
        GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
        _SlNonOsMainLoopTask();
        GPIO_IF_LedOn(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
    }

    return SUCCESS;

}




long printErrConvenience(char * msg, long retVal) {
   // UART_PRINT(msg);
    GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    return retVal;
}


//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! This function demonstrates how certificate can be used with SSL.
//! The procedure includes the following steps:
//! 1) connect to an open AP
//! 2) get the server name via a DNS request
//! 3) define all socket options and point to the CA certificate
//! 4) connect to the server via TCP
//!
//! \param None
//!
//! \return  0 on success else error code
//! \return  LED1 is turned solid in case of success
//!    LED2 is turned solid in case of failure
//!
//*****************************************************************************
static int tls_connect() {
    SlSockAddrIn_t    Addr;
    int    iAddrSize;
    unsigned char    ucMethod = SL_SO_SEC_METHOD_TLSV1_2;
    unsigned int uiIP;
//    unsigned int uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA;
    unsigned int uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256;
// SL_SEC_MASK_SSL_RSA_WITH_RC4_128_SHA
// SL_SEC_MASK_SSL_RSA_WITH_RC4_128_MD5
// SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_DHE_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_RC4_128_SHA
// SL_SEC_MASK_TLS_RSA_WITH_AES_128_CBC_SHA256
// SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA256
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256
// SL_SEC_MASK_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256 // does not work (-340, handshake fails)
    long lRetVal = -1;
    int iSockID;

    lRetVal = sl_NetAppDnsGetHostByName(g_Host, strlen((const char *)g_Host),
                                    (unsigned long*)&uiIP, SL_AF_INET);

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't retrieve the host name \n\r", lRetVal);
    }

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons(GOOGLE_DST_PORT);
    Addr.sin_addr.s_addr = sl_Htonl(uiIP);
    iAddrSize = sizeof(SlSockAddrIn_t);
    //
    // opens a secure socket 
    //
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
    if( iSockID < 0 ) {
        return printErrConvenience("Device unable to create secure socket \n\r", lRetVal);
    }

    //
    // configure the socket as TLS1.2
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECMETHOD, &ucMethod,\
                               sizeof(ucMethod));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
    //
    //configure the socket as ECDHE RSA WITH AES256 CBC SHA
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &uiCipher,\
                           sizeof(uiCipher));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }



/////////////////////////////////
// START: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION
    //
    //configure the socket with CA certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                           SL_SO_SECURE_FILES_CA_FILE_NAME, \
                           SL_SSL_CA_CERT, \
                           strlen(SL_SSL_CA_CERT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
// END: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION
/////////////////////////////////


    //configure the socket with Client Certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                SL_SO_SECURE_FILES_CERTIFICATE_FILE_NAME, \
                                    SL_SSL_CLIENT, \
                           strlen(SL_SSL_CLIENT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //configure the socket with Private Key - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
            SL_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME, \
            SL_SSL_PRIVATE, \
                           strlen(SL_SSL_PRIVATE));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }


    /* connect to the peer device - Google server */
    lRetVal = sl_Connect(iSockID, ( SlSockAddr_t *)&Addr, iAddrSize);

    if(lRetVal >= 0) {
//        UART_PRINT("Device has connected to the website:");
//        UART_PRINT(SERVER_NAME);
//        UART_PRINT("\n\r");
    }
    else if(lRetVal == SL_ESECSNOVERIFY) {
//        UART_PRINT("Device has connected to the website (UNVERIFIED):");
//        UART_PRINT(SERVER_NAME);
//        UART_PRINT("\n\r");
    }
    else if(lRetVal < 0) {
//        UART_PRINT("Device couldn't connect to server:");
//        UART_PRINT(SERVER_NAME);
//        UART_PRINT("\n\r");
        return printErrConvenience("Device couldn't connect to server \n\r", lRetVal);
    }

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    return iSockID;
}



int connectToAccessPoint() {
    long lRetVal = -1;
    GPIO_IF_LedConfigure(LED1|LED3);

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    lRetVal = InitializeAppVariables();
    ASSERT_ON_ERROR(lRetVal);

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0) {
      if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
          //UART_PRINT("Failed to configure the device in its default state \n\r");

      return lRetVal;
    }

    //UART_PRINT("Device is configured in default state \n\r");

    CLR_STATUS_BIT_ALL(g_ulStatus);

    ///
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    //UART_PRINT("Opening sl_start\n\r");
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal) {
        //UART_PRINT("Failed to start the device \n\r");
        return lRetVal;
    }

    //UART_PRINT("Device started as STATION \n\r");

    //
    //Connecting to WLAN AP
    //
    lRetVal = WlanConnect();
    if(lRetVal < 0) {
        //UART_PRINT("Failed to establish connection w/ an AP \n\r");
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }

   // UART_PRINT("Connection established w/ AP and IP is aquired \n\r");
    return 0;
}

/*****************************************************************************/
//adding new stuff
#define MAXSIZE 100
char globBuffer[MAXSIZE];
char currChar;
char prevChar;
char prevButton = 'I';
int lastIdx = 0;
int newButtonFlag = 0;
unsigned char enterFlag = 0;
unsigned char muteFlag = 0;
int curX = 0;
int curY = 0;
void stateMachine(char state) {
    unsigned char size = 1;
    switch(state) {
    case '0':
        if (prevChar == ' ') {currChar = '0';}
        else {currChar = ' ';}

        if(prevButton != '0' && prevButton != 'I')
        {
            prevButton = '0';
            newButtonFlag = 1;
        }
        else if(prevButton != '0' && prevButton == 'I'){
            prevButton = '0';
        }

        prevChar = currChar;
        break;
    case '1':
        if (prevChar == '!') {currChar = '?';}
        else if (prevChar == '?') {currChar = '.';}
        else if (prevChar == '.') {currChar = ',';}
        else if (prevChar == ',') {currChar = '1';}
        else {currChar = '!';}

        if(prevButton != '1' && prevButton != 'I')
        {
            prevButton = '1';
            newButtonFlag = 1;
        }
        else if(prevButton != '1' && prevButton == 'I'){
            prevButton = '1';
        }

        prevChar = currChar;
        break;
    case '2':
        if (prevChar == 'a') {currChar = 'b';}
        else if (prevChar == 'b') {currChar = 'c';}
        else if (prevChar == 'c') {currChar = '2';}
        else {currChar = 'a';}


        if(prevButton != '2' && prevButton != 'I')
        {
            prevButton = '2';
            newButtonFlag = 1;
        }
        else if(prevButton != '2' && prevButton == 'I'){
            prevButton = '2';
        }

        prevChar = currChar;
        break;
    case '3':
        if (prevChar == 'd') {currChar = 'e';}
        else if (prevChar == 'e') {currChar = 'f';}
        else if (currChar == 'f'){currChar = '3';}
        else {currChar = 'd';}

        if(prevButton != '3' && prevButton != 'I')
        {
            prevButton = '3';
            newButtonFlag = 1;
        }
        else if(prevButton != '3' && prevButton == 'I'){
            prevButton = '3';
        }

        prevChar = currChar;
        break;
    case '4':
        if (prevChar == 'g') {currChar = 'h';}
        else if (prevChar == 'h') {currChar = 'i';}
        else if (currChar == 'i') {currChar = '4';}
        else {currChar = 'g';}

        if(prevButton != '4' && prevButton != 'I')
        {
            prevButton = '4';
            newButtonFlag = 1;
        }
        else if(prevButton != '4' && prevButton == 'I'){
            prevButton = '4';
        }

        prevChar = currChar;
        break;
    case '5':
        if (prevChar == 'j') {currChar = 'k';}
        else if (prevChar == 'k') {currChar = 'l';}
        else if (currChar == 'l') {currChar = '5';}
        else {currChar = 'j';}

        if(prevButton != '5' && prevButton != 'I')
        {
            prevButton = '5';
            newButtonFlag = 1;
        }
        else if(prevButton != '5' && prevButton == 'I'){
            prevButton = '5';
        }

        prevChar = currChar;
        break;
    case '6':
        if (prevChar == 'm') {currChar = 'n';}
        else if (prevChar == 'n') {currChar = 'o';}
        else if (currChar == 'o') {currChar = '6';}
        else {currChar = 'm';}


        if(prevButton != '6' && prevButton != 'I')
        {
            prevButton = '6';
            newButtonFlag = 1;
        }
        else if(prevButton != '6' && prevButton == 'I'){
            prevButton = '6';
        }

        prevChar = currChar;
        break;
    case '7':
        if (prevChar == 'p') {currChar = 'q';}
        else if (prevChar == 'q') {currChar = 'r';}
        else if (currChar == 'r') {currChar = 's';}
        else if (currChar == 's') {currChar = '7';}
        else {currChar = 'p';}

        if(prevButton != '7' && prevButton != 'I')
        {
            prevButton = '7';
            newButtonFlag = 1;
        }
        else if(prevButton != '7' && prevButton == 'I'){
            prevButton = '7';
        }

        prevChar = currChar;
        break;
    case '8':
        if (prevChar == 't') {currChar = 'u';}
        else if (prevChar == 'u') {currChar = 'v';}
        else if (currChar == 'v') {currChar = '8';}
        else {currChar = 't';}

        if(prevButton != '8' && prevButton != 'I')
        {
            prevButton = '8';
            newButtonFlag = 1;
        }
        else if(prevButton != '8' && prevButton == 'I'){
            prevButton = '8';
        }

        prevChar = currChar;
        break;
    case '9':
        if (prevChar == 'w') {currChar = 'x';}
        else if (prevChar == 'x') {currChar = 'y';}
        else if (currChar == 'y') {currChar = 'z';}
        else if (currChar == 'z') {currChar = '9';}
        else {currChar = 'w';}

        if(prevButton != '9' && prevButton != 'I')
        {
            prevButton = '9';
            newButtonFlag = 1;
        }
        else if(prevButton != '9' && prevButton == 'I'){
            prevButton = '9';
        }

        prevChar = currChar;
        break;
    case '#':
        enterFlag = 1;
        if(prevButton != '#' && prevButton != 'I')
        {
            prevButton = '#';
            newButtonFlag = 1;
        }
        else if(prevButton != '#' && prevButton == 'I'){
            prevButton = '#';
        }

        prevChar = '#';
        break;
    case '*':
        muteFlag = 1;


        if(prevButton != '*' && prevButton != 'I')
        {
            prevButton = '*';
            newButtonFlag = 1;
        }
        else if(prevButton != '*' && prevButton == 'I'){
            prevButton = '*';
        }

        prevChar = '*';
        break;
    default: return;
    }


    if(newButtonFlag == 1 || textTimeOut == 1){
        //Text timeout timer
        TimerIntDisable(TIMERA1_BASE, TIMER_TIMA_TIMEOUT);

        if(muteFlag == 1)
        {
            drawChar(curX,curY, '\b', RED, WHITE, size);
            curX = curX - 8;
            muteFlag = 0;
            lastIdx--;
        }
        else if(enterFlag == 1)
        {
            enterFlag = 0;
//            int i = 0;
//            for(i = 0; i <= lastIdx; ++i){
//                Report("Buffer: %c \n \r", globBuffer[i]);
//                UARTCharPut(UARTA1_BASE, globBuffer[i]);
//            }
            //modify http_post() to send buffer

            http_post(id,globBuffer);
            memset(globBuffer, '\0', MAXSIZE);
            lastIdx = 0;
            curX = 0;
            curY = 0;
            newButtonFlag = 0;
            prevButton = 'I';
            fillScreen(BLACK);
            //clearTopHalf();
        }
        else{
            curX = curX + 8;
            if(curX >= 120)
            {
                curX = 0;
                curY = curY + 10;
            }
            drawChar(curX,curY, currChar, RED, WHITE, size);
            //Report("Char: %c \r \n", currChar);
            lastIdx++;
            globBuffer[lastIdx] = currChar;
        }

        newButtonFlag = 0;
        textTimeOut = 0;
    }
    else{
        if(muteFlag == 1)
        {
            drawChar(curX,curY, '\b', RED, WHITE, size);
            curX = curX - 8;
            muteFlag = 0;
            lastIdx--;
        }
        else{
            drawChar(curX,curY, currChar, RED, WHITE, size);
            //Report("Char: %c \r \n", currChar);
            globBuffer[lastIdx] = currChar;
        }
        //If same button is pressed start timeout timer
        TimerLoadSet(TIMERA1_BASE,TIMER_A,64000000);
        TimerIntEnable(TIMERA1_BASE, TIMER_TIMA_TIMEOUT);
        TimerEnable(TIMERA1_BASE, TIMER_A);
    }
}


long int goertzel(int sample[], long int coeff, int N)
//---------------------------------------------------------------//
{
//initialize variables to be used in the function
int Q, Q_prev, Q_prev2,i;
long prod1,prod2,prod3,power;

    Q_prev = 0;         //set delay element1 Q_prev as zero
    Q_prev2 = 0;        //set delay element2 Q_prev2 as zero
    power=0;            //set power as zero

    for (i=0; i<N; i++) // loop N times and calculate Q, Q_prev, Q_prev2 at each iteration
        {
            Q = (sample[i]) + ((coeff* Q_prev)>>14) - (Q_prev2); // >>14 used as the coeff was used in Q15 format
            Q_prev2 = Q_prev;                                    // shuffle delay elements
            Q_prev = Q;
        }

        //calculate the three products used to calculate power
        prod1=( (long) Q_prev*Q_prev);
        prod2=( (long) Q_prev2*Q_prev2);
        prod3=( (long) Q_prev *coeff)>>14;
        prod3=( prod3 * Q_prev2);

        power = ((prod1+prod2-prod3))>>8; //calculate power using the three products and scale the result down

        return power;
}

//-------Post-test function---------------------------------------//
unsigned char post_test(void)
//---------------------------------------------------------------//
{

    row = -1;
    col = -1;
// find the maximum power in the row frequencies and the row number

    max_power=0;            //initialize max_power=0

    for(i=0;i<4;i++)        //loop 4 times from 0>3 (the indecies of the rows)
        {
        if (power_all[i] > max_power)   //if power of the current row frequency > max_power
            {
            max_power=power_all[i];     //set max_power as the current row frequency
            row=i;                      //update row number
            }
        }


// find the maximum power in the column frequencies and the column number

    max_power=0;            //initialize max_power=0

    for(i=4;i<8;i++)        //loop 4 times from 4>7 (the indecies of the columns)
        {
        if (power_all[i] > max_power)   //if power of the current column frequency > max_power
            {
            max_power=power_all[i];     //set max_power as the current column frequency
            col=i;                      //update column number
            }
        }



    if(power_all[col]< 10000 && power_all[row] < 10000)
    new_dig=1;                             //set new_dig to 1 to display the next decoded digit


    if((power_all[col]>100000 && power_all[row]>100000) && (new_dig==1)) // check if maximum powers of row & column exceed certain threshold AND new_dig flag is set to 1
        {
            //write_lcd(1,row_col[row][col-4]);                       // display the digit on the LCD
            //dis_7seg(8,row_col[row][col-4]);                        // display the digit on 7-seg
            new_dig=0;                                              // set new_dig to 0 to avoid displaying the same digit again.
            return row_col[row][col-4];
        }

    return 0;
}

static void TextTimeout()
{
    TimerIntClear(TIMERA1_BASE, TIMER_TIMA_TIMEOUT);
    textTimeOut = 1;
    TimerDisable(TIMERA1_BASE, TIMER_A);
}

void sampleHandler(void)
{
//    Message("In Handler");
    unsigned char c0;    //temp char that holds bytes obtained
    unsigned char c1;

    if(sampleCount >= 410 && readyF == 0 )
    {
        readyF = 1;
        TimerIntDisable(TIMERA0_BASE, TIMER_A);
        sampleCount = 0;
    }
    else
    {
        //Converter CS Active = LOW
        GPIOPinWrite(GPIOA3_BASE, 0x40, 0x00);
        SPICSEnable(GSPI_BASE);

        SPITransfer(GSPI_BASE, 0, &c0, 1, 0);
        sampleBuffer[sampleCount] = ((unsigned int)c0 & 31) << 5;
        //c0= (c0 & (0x1f)) << 5;

        SPITransfer(GSPI_BASE, 0, &c0, 1, 0);
        sampleBuffer[sampleCount] = sampleBuffer[sampleCount] | (c0 >> 3);
        //c1 = (c1 >> 3) | c0;
        //Report("Buffer: %d \r \n", sampleBuffer[sampleCount]);
        sampleBuffer[sampleCount] -= 300;
        sampleCount++;
        //Disabling CS's
        GPIOPinWrite(GPIOA3_BASE, 0x40, 0x40);
        SPICSDisable(GSPI_BASE);
    }
    TimerIntClear(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);//maybe every time

}


static void UartHandler()
{

    unsigned long status = UARTIntStatus(UARTA1_BASE, true);
    char temp;

    if((status & UART_INT_RX) && UARTCharsAvail(UARTA1_BASE))
    {
        temp = UARTCharGet(UARTA1_BASE);
        //Report("%c",temp);
        //recvBuffer[charCount++] = temp;
        http_post(id, "WUT");
    }

    UARTIntClear(UARTA1_BASE, UART_INT_RX);
}


void interruptInit() {
    //Initializeing Timer
    UARTConfigSetExpClk(UARTA1_BASE,PRCMPeripheralClockGet(PRCM_UARTA1),9600, (UART_CONFIG_WLEN_8
            | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));


    UARTEnable(UARTA1_BASE);
    UARTFIFODisable(UARTA1_BASE);

    UARTIntRegister(UARTA1_BASE, UartHandler);
    UARTIntClear(UARTA1_BASE, UART_INT_RX);
    UARTIntEnable(UARTA1_BASE, UART_INT_RX);


}





//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void main() {

    char letter;
    unsigned long ulStatus;
    int confidenceLevel = 0;
    char prevC;
    long lRetVal = -1;

    BoardInit();
    PinMuxConfig();


    //Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();
    //Set time so that encryption can be used
    lRetVal = set_time();
    if(lRetVal < 0) {
        //UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }
    //Connect to the website with TLS encryption
    lRetVal = tls_connect();
    if(lRetVal < 0) {
        ERR_PRINT(lRetVal);
    }
    id = lRetVal;
    //Initializing stuff for interrupts
    //interruptInit();
//    http_post(lRetVal, "Heart rate monitor initiated");

    int polling = 0;
    while(1){
        //Leds used to help debug when testing the device after program has been flashed
        //onto the cc3200 to know if if() statement was entered
        GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
        delay(100);
        //waiting for signal from other launchpad
        long predicate = GPIOPinRead(GPIOA3_BASE, 0x40);
        if(predicate){
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            delay(100);
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            delay(100);
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            delay(100);
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            delay(100);
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            delay(100);
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);

            //send warning to phone
            http_post(lRetVal, "Warning!");


            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            delay(100);
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            delay(100);
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            delay(100);
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            delay(100);
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            delay(100);
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            delay(100);

            polling = 1;
        }

        GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
        delay(100);


//        else if((GPIOPinRead(GPIOA3_BASE, 0x2) == 0x0)) {
//            polling = 0;
//        }
    }

}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

static int http_post(int iTLSSockID, char* message){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char jsonBuff[500];
    char* json;
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    json = jsonBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    strcpy(json, "{\"state\": {\r\n\"desired\" : {\r\n\"var\" : \"");
    json += strlen("{\"state\": {\r\n\"desired\" : {\r\n\"var\" : \"");

    strcpy(json, message);
    json += strlen(message);

    strcpy(json,"\"\r\n}}}\r\n\r\n");
    json += strlen("\"\r\n}}}\r\n\r\n");

//    strcpy(json, "{\n\r\"default\":\"");
//    json += strlen("{\n\r\"default\":\"");
//
//    strcpy(json, message);
//    json += strlen(message);
//
//    strcpy(json, "\",\n\r\"sms\":\"");
//    json += strlen("\",\n\r\"sms\":\"");
//
//    strcpy(json, message);
//    json += strlen(message);
//
//    strcpy(json, "\",\n\r\"APNS_SANDBOX\":\"{\\\"aps\\\":{\\\"alert\\\":\\\"");
//    json += strlen("\",\n\r\"APNS_SANDBOX\":\"{\\\"aps\\\":{\\\"alert\\\":\\\"");
//
//    strcpy(json, message);
//    json += strlen(message);
//
//    strcpy(json, "\\\"}}\"\n\r}");
//    json += strlen("\\\"}}\"\n\r}");

    int dataLength = strlen(jsonBuff);
//    int dataLength = strlen(DATA1);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

//    strcpy(pcBufHeaders, DATA1);
//    pcBufHeaders += strlen(DATA1);

    strcpy(pcBufHeaders, jsonBuff);
    pcBufHeaders += strlen(jsonBuff);

    int testDataLength = strlen(pcBufHeaders);

    //UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        //UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        //UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        //UART_PRINT(acRecvbuff);
        //UART_PRINT("\n\r\n\r");
    }

    return 0;
}




static int http_get(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, GETHEADER);
    pcBufHeaders += strlen(GETHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");
//
//    int dataLength = strlen(DATA1);
//
//    strcpy(pcBufHeaders, CTHEADER);
//    pcBufHeaders += strlen(CTHEADER);
//    strcpy(pcBufHeaders, CLHEADER1);
//
//    pcBufHeaders += strlen(CLHEADER1);
//    sprintf(cCLLength, "%d", dataLength);
//
//    strcpy(pcBufHeaders, cCLLength);
//    pcBufHeaders += strlen(cCLLength);
//    strcpy(pcBufHeaders, CLHEADER2);
//    pcBufHeaders += strlen(CLHEADER2);
//
//    strcpy(pcBufHeaders, DATA1);
//    pcBufHeaders += strlen(DATA1);
//
//    int testDataLength = strlen(pcBufHeaders);
//
    //UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        //UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        //UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
//        UART_PRINT(acRecvbuff);
//        UART_PRINT("\n\r\n\r");
    }

//    UART_PRINT(acRecvbuff);

    return 0;
}
