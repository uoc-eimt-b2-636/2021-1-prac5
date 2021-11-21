/*
 * Copyright (C) 2017 Universitat Oberta de Catalunya - http://www.uoc.edu/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Universitat Oberta de Catalunya nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*----------------------------------------------------------------------------*/

/* Standard includes */
#include <stdlib.h>
#include <stdio.h>

/* Free-RTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portmacro.h"

/* Launchpad, Wi-Fi and Sensors includes */
#include "msp432_launchpad_board.h"
#include "edu_boosterpack_microphone.h"
#include "edu_boosterpack_sensors.h"
#include "edu_boosterpack_lcd.h"
#include "edu_boosterpack_rgb.h"
#include "edu_boosterpack_buzzer.h"
#include "cc3100_boosterpack.h"

/*----------------------------------------------------------------------------*/

#define SPAWN_TASK_PRIORITY     ( tskIDLE_PRIORITY + 7 )
#define TX_TASK_PRIORITY        ( tskIDLE_PRIORITY + 2 )
#define SENSORS_TASK_PRIORITY   ( tskIDLE_PRIORITY + 1 )

#define MAIN_STACK_SIZE         ( 2048 )
#define BLINK_STACK_SIZE        ( 128 )

#define SERVER_ADDRESS          ( SL_IPV4_VAL(0,0,0,0) )
#define TCP_PORT_NUMBER         ( 5001 )
#define UDP_PORT_NUMBER         ( 12345 )

#define TX_BUFFER_SIZE          ( 512 )
#define RX_BUFFER_SIZE          ( 512 )

#define NODE_ID                 ( 1 )

/*----------------------------------------------------------------------------*/

QueueHandle_t xQueue;

typedef struct
{
    float ambient_temp;
    float ambient_light;
    int16_t acceleration_x;
    int16_t acceleration_y;
    int16_t acceleration_z;
} measurements_t;

/*----------------------------------------------------------------------------*/

static void SensorsTask(void *pvParameters);
static void ParserTask(void *pvParameters);

static void SensorsTask(void *pvParameters)
{
    measurements_t measurements = { 0.0f, 0.0f, 0, 0, 0 };
    BaseType_t xStatus;

    float ambient_temp;
    float ambient_light;
    int16_t acceleration_x;
    int16_t acceleration_y;
    int16_t acceleration_z;

    /* Initialize sensors */
    edu_boosterpack_sensors_init();

    while (true)
    {
        /* Turn red LED on */
        led_red_on();

        ambient_temp = 0.0f;
        ambient_light = 0.0f;
        acceleration_x = 0;
        acceleration_y = 0;
        acceleration_z = 0;

        /* Read temperature */
        edu_boosterpack_sensors_temperature_read(&ambient_temp);

        /* Read light */
        edu_boosterpack_sensors_light_read(&ambient_light);

        /* Read acceleration */
        edu_boosterpack_sensors_acceleration_read(ambient_temp, &acceleration_x,
                                                  &acceleration_y,
                                                  &acceleration_z);

        measurements.ambient_temp = ambient_temp;
        measurements.ambient_light = ambient_light;
        measurements.acceleration_x = acceleration_x;
        measurements.acceleration_y = acceleration_y;
        measurements.acceleration_z = acceleration_z;

        xStatus = xQueueSend(xQueue, &measurements, 0);

        /* Turn red LED on */
        led_red_off();

        /* Sleep for 5 seconds */
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void printLCD(char* buffer)
{
//    edu_bootsterpack_lcd_clear();
    edu_bootsterpack_lcd_print(buffer, 50, 5);
}

static void ParserTask(void *pvParameters)
{
    tune_t note = { E6, _M};

    SlSockAddrIn_t socket_addr;

    int32_t retVal = 0;
    int16_t tcp_socket = 0;

    uint8_t recv_buffer[16];
    uint8_t send_buffer[6] = {0x00,0x01,0x02,0x03,0x04,0x05};

    /* Restore Wi-Fi */
    retVal = wifi_restore();
    if (retVal < 0)
    {
        led_red_on();
        while (1)
            ;
    }

    /* Initialize Wi-Fi */
    retVal = wifi_init();
    if (retVal < 0)
    {
        led_red_on();
        while (1)
            ;
    }

    /* Create a UDP socket */
    wifi_set_socket_address(&socket_addr, SERVER_ADDRESS, TCP_PORT_NUMBER, true);

    tcp_socket = wifi_tcp_client_open(&socket_addr);

    if (tcp_socket < 0)
    {
        led_red_on();
        while (1)
            ;
    }

    while (true)
    {
        retVal = wifi_tcp_client_receive(tcp_socket, recv_buffer, 16);
        if (retVal > 0)
        {
            switch (recv_buffer[0])
            {
            case 'G':
                edu_boosterpack_rgb_green_on();
                break;
            case 'g':
                edu_boosterpack_rgb_green_off();
                break;
            case 'R':
                edu_boosterpack_rgb_red_on();
                break;
            case 'r':
                edu_boosterpack_rgb_red_off();
                break;
            case 'z':
                edu_boosterpack_buzzer_play_tune(note, 1);
                break;
            case 'e': /* echo */
                wifi_tcp_client_send(tcp_socket, send_buffer, 6);
                break;
            }
        }

    }

}

/*----------------------------------------------------------------------------*/

void buzzer_callback(void) {
    MAP_Timer_A_stopTimer(TIMER_A1_BASE);
    MAP_Timer_A_stopTimer(TIMER_A0_BASE);
}

int main(int argc, char** argv)
{
    int32_t retVal;

    xQueue = xQueueCreate(5, sizeof(measurements_t));

    if (xQueue != NULL)
    {
        /* Initialize the board */
        board_init();

        // Tests
        edu_boosterpack_rgb_init();

        // Init LCD
        edu_boosterpack_lcd_init();

        // Init buzzer
        edu_boosterpack_buzzer_init();
        edu_boosterpack_set_bpm(78);
        edu_boosterpack_buzzer_callback_set(buzzer_callback);
//        edu_boosterpack_buzzer_pwm(5, 1);

        /*
         * Start the SimpleLink task
         */
        retVal = VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
        if (retVal < 0)
        {
            led_red_on();
            while (1)
                ;
        }

        /* Create sensors task */
        retVal = xTaskCreate(SensorsTask, "SensorsTask",
        BLINK_STACK_SIZE,
                             NULL,
                             SENSORS_TASK_PRIORITY,
                             NULL);
        if (retVal < 0)
        {
            led_red_on();
            while (1)
                ;
        }

        /* Create communication task */
        retVal = xTaskCreate(ParserTask, "ParserTask",
        MAIN_STACK_SIZE,
                             NULL,
                             TX_TASK_PRIORITY,
                             NULL);
        if (retVal < 0)
        {
            led_red_on();
            while (1)
                ;
        }

        /* Start the task scheduler */
        vTaskStartScheduler();
    }
    return 0;
}

void PORT1_IRQHandler(void)
{
    uint32_t status;

    // Lee el estado de la interrupcion generada por GPIO_PORT_P1
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);

    // Reset del flag de interrupcion del pin que la genera
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);
}
