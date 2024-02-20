/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "config/default/system/debug/sys_debug.h"
#include "config/default/usb/usb_device.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
int sof;

/* TODO:  Add any necessary callback functions.
*/
USB_DEVICE_EVENT_RESPONSE APP_USBDeviceEventHandler
(
    USB_DEVICE_EVENT event,
    void * pData,
    uintptr_t context
)
{
    uint8_t     activeConfiguration;

    // Handling of each event
    switch(event)
    {
        case USB_DEVICE_EVENT_POWER_DETECTED:

            // This means the device detected a valid VBUS voltage
            // and is attached to the USB if the device is bus powered.
            SYS_DEBUG(0, "USB_DEVICE_Attach!\r\n");
            USB_DEVICE_Attach(appData.usbDeviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            // This means the device is not attached to the USB.
            SYS_DEBUG(0, "USB_DEVICE_Detach!\r\n");
            USB_DEVICE_Detach(appData.usbDeviceHandle);
            appData.deviceIsConfigured = false;
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            // The bus is idle. There was no activity detected.
            // The application can switch to a low power mode after
            // exiting the event handler.
            break;

        case USB_DEVICE_EVENT_SOF:

            // A start of frame was received. This is a periodic
            // event and can be used the application for time
            // related activities. pData will point to a USB_DEVICE_EVENT_DATA_SOF type data
            // containing the frame number.

            // Example code for retriveing the frame number.
            // frameNumber = ((USB_DEVICE_EVENT_DATA_SOF *)(pData))->frameNumber;
            if (sof++%1000 == 0) {
                SYS_DEBUG(0, "SOF 1000!\r\n");
            }

            break;

        case USB_DEVICE_EVENT_RESET :

            // Reset signalling was detected on the bus. The
            // application can find out the attach speed.

            // Example code for retriveing the speed
            // attachSpeed = USB_DEVICE_ActiveSpeedGet(appData.usbDeviceHandle);

            break;

        case USB_DEVICE_EVENT_DECONFIGURED :

            // This indicates that host has deconfigured the device i.e., it
            // has set the configuration as 0. All function driver instances
            // would have been deinitialized.

            appData.deviceIsConfigured = false;
            
            break;

        case USB_DEVICE_EVENT_ERROR :

            // This means an unknown error has occurred on the bus.
            // The application can try detaching and attaching the
            // device again.
            break;

        case USB_DEVICE_EVENT_CONFIGURED :

            // This means that device is configured and the application can
            // start using the device functionality. The application must
            // register function driver event handlers within this event.
            // The pData parameter will be a pointer to a USB_DEVICE_EVENT_DATA_CONFIGURED data type
            // that contains the active configuration number.

            activeConfiguration = ((USB_DEVICE_EVENT_DATA_CONFIGURED *)(pData))->configurationValue;
            if (activeConfiguration == 1)
            {
                // Device is enumerated. Register here the USB Function Driver Event Handler function.
                appData.deviceIsConfigured = true;
            }
            break;

        case USB_DEVICE_EVENT_RESUMED:

            // This means that the resume signalling was detected on the
            // bus. The application can bring the device out of power
            // saving mode.

            break;

        case USB_DEVICE_EVENT_CONTROL_TRANSFER_SETUP_REQUEST:

            // This means that the setup stage of the control transfer is in
            // progress and a setup packet has been received. The pData
            // parameter will point to a USB_SETUP_PACKET data type The
            // application can process the command and update its control
            // transfer state machine. The application for example could call
            // the USB_DEVICE_ControlReceive() function (as shown here) to
            // submit the buffer that would receive data in case of a
            // control read transfer.
            // Example:
            // setupEventData = (USB_SETUP_PACKET *)pData;

            // Application can now respond to the Setup packet by submitting a buffer
            // to receive 32 bytes in the  control write transfer */
            // USB_DEVICE_ControlReceive(appData.usbDevHandle, data, 32);
            break;

        case USB_DEVICE_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            // This means that data in the data stage of the control write
            // transfer has been received. The application can either accept
            // the received data by calling the USB_DEVICE_ControlStatus()
            // function with USB_DEVICE_CONTROL_STATUS_OK flag (as shown in
            // this example) or it can reject it by calling the
            // USB_DEVICE_ControlStatus() function with
            // USB_DEVICE_CONTROL_STATUS_ERROR flag.

            USB_DEVICE_ControlStatus(appData.usbDeviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_EVENT_CONTROL_TRANSFER_DATA_SENT:

            // This means that data in the data stage of the control
            // read transfer has been sent.
            break;

        case USB_DEVICE_EVENT_CONTROL_TRANSFER_ABORTED:

            // This means the host has aborted the control transfer. The
            // application can reset it's control transfer state machine.

            break;

        default:
            break;
     }

     return USB_DEVICE_EVENT_RESPONSE_NONE;
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    SYS_DEBUG(0, "Hello world!\r\n");

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    // set clock testpoints
    RPD5R = 0x12;  // REFCLKO4/RD5
    REFO4CON = 0x2009000;  // SYSCLK/2/512
    RPC8R = 0x12;  // REFCLKO3/RC8
    REFO3CON = 0x2009006;  // UPLL/2/512    
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;

            SYS_DEBUG(0, "USB_DEVICE_Open!\r\n");
            appData.usbDeviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, 0);
            appInitialized &= (appData.usbDeviceHandle != USB_DEVICE_HANDLE_INVALID);

            if (appInitialized)
            {
                SYS_DEBUG(0, "USB_DEVICE_EventHandlerSet!\r\n");
                USB_DEVICE_EventHandlerSet(appData.usbDeviceHandle, APP_USBDeviceEventHandler, 0);
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {

            break;
        }

        /* TODO: implement your application state machine.*/


        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}


/*******************************************************************************
 End of File
 */
