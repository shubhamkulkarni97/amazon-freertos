/*
 * FreeRTOS V1.4.7
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

#include "iot_config.h"

/* FreeRTOS includes. */

#include "FreeRTOS.h"
#include "task.h"
#include "iot_i2c.h"

/* Demo includes */
#include "aws_demo.h"
#include "aws_dev_mode_key_provisioning.h"

/* AWS System includes. */
#include "bt_hal_manager.h"
#include "iot_system_init.h"
#include "iot_logging_task.h"

#include "nvs_flash.h"
#if !AFR_ESP_LWIP
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#endif

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_interface.h"
#include "esp_bt.h"
#if CONFIG_NIMBLE_ENABLED == 1
    #include "esp_nimble_hci.h"
#else
    #include "esp_gap_ble_api.h"
    #include "esp_bt_main.h"
#endif

#include "driver/uart.h"
#include "aws_application_version.h"
#include "tcpip_adapter.h"

#include "iot_network_manager_private.h"

#if BLE_ENABLED
    #include "bt_hal_manager_adapter_ble.h"
    #include "bt_hal_manager.h"
    #include "bt_hal_gatt_server.h"

    #include "iot_ble.h"
    #include "iot_ble_config.h"
    #include "iot_ble_wifi_provisioning.h"
    #include "iot_ble_numericComparison.h"
#endif

/* Logging Task Defines. */
#define mainLOGGING_MESSAGE_QUEUE_LENGTH    ( 32 )
#define mainLOGGING_TASK_STACK_SIZE         ( configMINIMAL_STACK_SIZE * 4 )
#define mainDEVICE_NICK_NAME                "Espressif_Demo"

QueueHandle_t spp_uart_queue = NULL;

/* Static arrays for FreeRTOS+TCP stack initialization for Ethernet network connections
 * are use are below. If you are using an Ethernet connection on your MCU device it is
 * recommended to use the FreeRTOS+TCP stack. The default values are defined in
 * FreeRTOSConfig.h. */

/**
 * @brief Initializes the board.
 */
static void prvMiscInitialization( void );

#if BLE_ENABLED
/* Initializes bluetooth */
    static esp_err_t prvBLEStackInit( void );
    /** Helper function to teardown BLE stack. **/
    esp_err_t xBLEStackTeardown( void );
    static void spp_uart_init( void );
#endif

/*-----------------------------------------------------------*/

int8_t i2c_read()
{
    IotI2CHandle_t xI2CHandle;
    int32_t lRetVal;
    uint8_t singleByte;

    uint16_t readBytes;
    uint16_t writeBytes;

    uint8_t testIotI2C_INSTANCE = 0;
    uint8_t uctestIotI2CSlaveAddr = 0x68;   // Set I2C slave address
    uint8_t uctestIotI2CDeviceRegister = 0x2;

    IotI2CConfig_t xI2CConfig =
    {
        .ulBusFreq       = 400000,
        .ulMasterTimeout = 400,
    };

    /* Open i2c to initialize hardware */
    xI2CHandle = iot_i2c_open( testIotI2C_INSTANCE );

    /* Set i2c configuration */
    lRetVal = iot_i2c_ioctl( xI2CHandle, eI2CSetMasterConfig, &xI2CConfig );
    if (lRetVal != IOT_I2C_SUCCESS) {
        printf("Failed to set master config\n");
        return -1;
    }

    /* Set i2c slave address */
    lRetVal = iot_i2c_ioctl( xI2CHandle, eI2CSetSlaveAddr, &uctestIotI2CSlaveAddr );
    if (lRetVal != IOT_I2C_SUCCESS) {
        printf("Failed to set slave address\n");
        return -1;
    }

    /* Set i2c configuration */
    lRetVal = iot_i2c_ioctl( xI2CHandle, eI2CSendNoStopFlag, NULL );
    if (lRetVal != IOT_I2C_SUCCESS) {
        printf("Failed to ser no stop flag\n");
        return -1;
    }

    /* write the device register address. */
    lRetVal = iot_i2c_write_sync( xI2CHandle, &uctestIotI2CDeviceRegister, sizeof( uctestIotI2CDeviceRegister ) );
    if (lRetVal != IOT_I2C_SUCCESS) {
        printf("Failed to write\n");
        return -1;
    }

    /* repeated start to read */
    lRetVal = iot_i2c_ioctl( xI2CHandle, eI2CSetSlaveAddr, &uctestIotI2CSlaveAddr );
    if (lRetVal != IOT_I2C_SUCCESS) {
        printf("Failed to set slave address\n");
        return -1;
    }

    /* read from i2c device for single byte */
    lRetVal = iot_i2c_read_sync( xI2CHandle, &singleByte, sizeof( singleByte ) );
    if (lRetVal != IOT_I2C_SUCCESS) {
        printf("Failed to read\n");
        return -1;
    }

    lRetVal = iot_i2c_ioctl( xI2CHandle, eI2CGetTxNoOfbytes, &writeBytes );
    /* Assert the number of bytes being written is 1. */
    if (lRetVal != IOT_I2C_SUCCESS || writeBytes != sizeof( singleByte )) {
        printf("Failed to get number of TxBytes\n");
        return -1;
    }

    lRetVal = iot_i2c_ioctl( xI2CHandle, eI2CGetRxNoOfbytes, &readBytes );
    /* Assert the number of bytes being read is 1. */
    if (lRetVal != IOT_I2C_SUCCESS || readBytes != sizeof( singleByte )) {
        printf("Failed to get number of RxBytes\n");
        return -1;
    }

    lRetVal = iot_i2c_close( xI2CHandle );
    if (lRetVal != IOT_I2C_SUCCESS) {
        printf("Failed to close\n");
        return -1;
    }
    return singleByte;
}

int8_t i2c_write(uint8_t uctestIotI2CWriteVal)
{
    IotI2CHandle_t xI2CHandle;
    int32_t lRetVal;
    uint8_t testIotI2C_INSTANCE = 0;
    uint8_t uctestIotI2CSlaveAddr = 0x68;   // Set I2C slave address
    uint8_t uctestIotI2CDeviceRegister = 0x2;
    uint8_t writeVal1[] = { uctestIotI2CDeviceRegister, uctestIotI2CWriteVal };

    uint16_t writeBytes;

    IotI2CConfig_t xI2CConfig =
    {
        .ulBusFreq       = 400000,
        .ulMasterTimeout = 400
    };

    /* Open i2c to initialize hardware */
    xI2CHandle = iot_i2c_open( testIotI2C_INSTANCE );

    /* Set i2c configuration */
    lRetVal = iot_i2c_ioctl( xI2CHandle, eI2CSetMasterConfig, &xI2CConfig );
    if (lRetVal != IOT_I2C_SUCCESS) {
        printf("Failed to set master config\n");
        return -1;
    }

    /* Set i2c slave address */
    lRetVal = iot_i2c_ioctl( xI2CHandle, eI2CSetSlaveAddr, &uctestIotI2CSlaveAddr );
    if (lRetVal != IOT_I2C_SUCCESS) {
        printf("Failed to set slave address\n");
        return -1;
    }

    /* write the value to the device */
    lRetVal = iot_i2c_write_sync( xI2CHandle, writeVal1, sizeof( writeVal1 ) );
    if (lRetVal != IOT_I2C_SUCCESS) {
        printf("Failed to write\n");
        return -1;
    }

    lRetVal = iot_i2c_ioctl( xI2CHandle, eI2CGetTxNoOfbytes, &writeBytes );
    if (lRetVal != IOT_I2C_SUCCESS) {
        printf("Failed to get number of TxBytes\n");
        return -1;
    }

    lRetVal = iot_i2c_close( xI2CHandle );
    if (lRetVal != IOT_I2C_SUCCESS) {
        printf("Failed to close\n");
        return -1;
    }
    return 0;
}

/**
 * @brief Application runtime entry point.
 */
int app_main( void )
{
    int8_t data = 7;
    if (i2c_write(data) < 0) {
        printf("Failed to write\n");
    }
    int8_t data_read = i2c_read();
    if (data_read < 0) {
        printf("Failed to read\n");
    }
    printf("%d is data read\n", data_read);
    /* Perform any hardware initialization that does not require the RTOS to be
     * running.  */

    return 0;
}

/*-----------------------------------------------------------*/
extern void vApplicationIPInit( void );
static void prvMiscInitialization( void )
{
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();

    if( ( ret == ESP_ERR_NVS_NO_FREE_PAGES ) || ( ret == ESP_ERR_NVS_NEW_VERSION_FOUND ) )
    {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK( ret );

    #if BLE_ENABLED
        NumericComparisonInit();
        spp_uart_init();
    #endif

    /* Create tasks that are not dependent on the WiFi being initialized. */
    xLoggingTaskInitialize( mainLOGGING_TASK_STACK_SIZE,
                            tskIDLE_PRIORITY + 5,
                            mainLOGGING_MESSAGE_QUEUE_LENGTH );

#if AFR_ESP_LWIP
    configPRINTF( ("Initializing lwIP TCP stack\r\n") );
    tcpip_adapter_init();
#else
    configPRINTF( ("Initializing FreeRTOS TCP stack\r\n") );
    vApplicationIPInit();
#endif
}

/*-----------------------------------------------------------*/

#if BLE_ENABLED

    #if CONFIG_NIMBLE_ENABLED == 1
        esp_err_t prvBLEStackInit( void )
        {
            return ESP_OK;
        }


        esp_err_t xBLEStackTeardown( void )
        {
            esp_err_t xRet;

            xRet = esp_bt_controller_mem_release( ESP_BT_MODE_BLE );

            return xRet;
        }

    #else /* if CONFIG_NIMBLE_ENABLED == 1 */

        static esp_err_t prvBLEStackInit( void )
        {
            return ESP_OK;
        }

        esp_err_t xBLEStackTeardown( void )
        {
            esp_err_t xRet = ESP_OK;

            if( esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED )
            {
                xRet = esp_bluedroid_disable();
            }

            if( xRet == ESP_OK )
            {
                xRet = esp_bluedroid_deinit();
            }

            if( xRet == ESP_OK )
            {
                if( esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED )
                {
                    xRet = esp_bt_controller_disable();
                }
            }

            if( xRet == ESP_OK )
            {
                xRet = esp_bt_controller_deinit();
            }

            if( xRet == ESP_OK )
            {
                xRet = esp_bt_controller_mem_release( ESP_BT_MODE_BLE );
            }

            if( xRet == ESP_OK )
            {
                xRet = esp_bt_controller_mem_release( ESP_BT_MODE_BTDM );
            }

            return xRet;
        }
    #endif /* if CONFIG_NIMBLE_ENABLED == 1 */
#endif /* if BLE_ENABLED */

/*-----------------------------------------------------------*/

#if BLE_ENABLED
    static void spp_uart_init( void )
    {
        uart_config_t uart_config =
        {
            .baud_rate           = 115200,
            .data_bits           = UART_DATA_8_BITS,
            .parity              = UART_PARITY_DISABLE,
            .stop_bits           = UART_STOP_BITS_1,
            .flow_ctrl           = UART_HW_FLOWCTRL_RTS,
            .rx_flow_ctrl_thresh = 122,
        };

        /* Set UART parameters */
        uart_param_config( UART_NUM_0, &uart_config );
        /*Set UART pins */
        uart_set_pin( UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE );
        /*Install UART driver, and get the queue. */
        uart_driver_install( UART_NUM_0, 4096, 8192, 10, &spp_uart_queue, 0 );
    }

/*-----------------------------------------------------------*/

    BaseType_t getUserMessage( INPUTMessage_t * pxINPUTmessage,
                               TickType_t xAuthTimeout )
    {
        uart_event_t xEvent;
        BaseType_t xReturnMessage = pdFALSE;

        if( xQueueReceive( spp_uart_queue, ( void * ) &xEvent, ( portTickType ) xAuthTimeout ) )
        {
            switch( xEvent.type )
            {
                /*Event of UART receiving data */
                case UART_DATA:

                    if( xEvent.size )
                    {
                        pxINPUTmessage->pcData = ( uint8_t * ) malloc( sizeof( uint8_t ) * xEvent.size );

                        if( pxINPUTmessage->pcData != NULL )
                        {
                            memset( pxINPUTmessage->pcData, 0x0, xEvent.size );
                            uart_read_bytes( UART_NUM_0, ( uint8_t * ) pxINPUTmessage->pcData, xEvent.size, portMAX_DELAY );
                            xReturnMessage = pdTRUE;
                        }
                        else
                        {
                            configPRINTF( ( "Malloc failed in main.c\n" ) );
                        }
                    }

                    break;

                default:
                    break;
            }
        }

        return xReturnMessage;
    }
#endif /* if BLE_ENABLED */

/*-----------------------------------------------------------*/

extern void esp_vApplicationTickHook();
void IRAM_ATTR vApplicationTickHook()
{
    esp_vApplicationTickHook();
}

/*-----------------------------------------------------------*/
extern void esp_vApplicationIdleHook();
void vApplicationIdleHook()
{
    esp_vApplicationIdleHook();
}

/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook( void )
{
}

#if !AFR_ESP_LWIP
/*-----------------------------------------------------------*/
void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent )
{
    uint32_t ulIPAddress, ulNetMask, ulGatewayAddress, ulDNSServerAddress;
    system_event_t evt;

    if( eNetworkEvent == eNetworkUp )
    {
        /* Print out the network configuration, which may have come from a DHCP
         * server. */
        FreeRTOS_GetAddressConfiguration(
            &ulIPAddress,
            &ulNetMask,
            &ulGatewayAddress,
            &ulDNSServerAddress );

        evt.event_id = SYSTEM_EVENT_STA_GOT_IP;
        evt.event_info.got_ip.ip_changed = true;
        evt.event_info.got_ip.ip_info.ip.addr = ulIPAddress;
        evt.event_info.got_ip.ip_info.netmask.addr = ulNetMask;
        evt.event_info.got_ip.ip_info.gw.addr = ulGatewayAddress;
        esp_event_send( &evt );
    }
}
#endif
