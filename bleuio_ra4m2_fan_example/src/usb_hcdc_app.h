/***********************************************************************************************************************
 * File Name    : usb_hcdc_app.h
 * Description  : Contains data structures and functions used in usb_hcdc_app.h.
 **********************************************************************************************************************/
/***********************************************************************************************************************
 *
 * Copyright (C) 2025 Smart Sensor Devices AB. All rights reserved.
 ***********************************************************************************************************************/


#ifndef USB_HCDC_APP_H_
#define USB_HCDC_APP_H_

#include "bsp_api.h"
#include "hal_data.h"
#include "common_data.h"

/** Macros definitions **/

#define EP_INFO        "\r\nThis example project demonstrates how to use the Renesas RA MCUs together\r\n"\
                       "with a BleuIO dongle to turn a fan on or off based on the CO2 values of a nearby\r\n"\
                       "HibouAir sensor. The BleuIO will scan the HibouAir sensor and based on the\r\n"\
                       "CO2 value the fan will be turned on or off.\r\n"\
                       "If CO2 value is 600ppm or above, the fan will turn on.\r\n"\
                       "If CO2 value is 550ppm or below, the fan will turn off.\r\n"\
                       "The on board LEDs will also light up based on the CO2 value.\r\n"\
                       "Blue for good (<600ppm), green for average (<1000ppm) and red for poor (>1000ppm).\r\n"\
                       "The board will print the CO2 values as the change on the RTTViewer.\r\n\n\n"

#define SET_LINE_CODING             (USB_CDC_SET_LINE_CODING | USB_HOST_TO_DEV | USB_CLASS | USB_INTERFACE)
#define GET_LINE_CODING             (USB_CDC_GET_LINE_CODING | USB_DEV_TO_HOST | USB_CLASS | USB_INTERFACE)
#define SET_CONTROL_LINE_STATE      (USB_CDC_SET_CONTROL_LINE_STATE | USB_HOST_TO_DEV | USB_CLASS | USB_INTERFACE)
#define LINE_CODING_LENGTH          (0x07U)
#define CONTROL_LINE_STATE_LENGTH   (0x02U)
#define VALUE_ZERO                  (0x0000U)
#define NO_WAIT_TIME                0
#define CDC_READ_DATA_LEN           512
#define CDC_WRITE_DATA_LEN          512
#define ZERO_INDEX                  0

#define OFF                                       (0U)
#define ON                                        (1U)

#define BLUE                                      (BSP_LED_LED1)
#define GREEN                                     (BSP_LED_LED2)
#define RED                                       (BSP_LED_LED3)

#define TURN_RED_ON                               R_IOPORT_PinWrite(&g_ioport_ctrl, g_bsp_leds.p_leds[RED], ON);
#define TURN_RED_OFF                              R_IOPORT_PinWrite(&g_ioport_ctrl, g_bsp_leds.p_leds[RED], OFF);
#define TURN_GREEN_ON                             R_IOPORT_PinWrite(&g_ioport_ctrl, g_bsp_leds.p_leds[GREEN], ON);
#define TURN_GREEN_OFF                            R_IOPORT_PinWrite(&g_ioport_ctrl, g_bsp_leds.p_leds[GREEN], OFF);
#define TURN_BLUE_ON                              R_IOPORT_PinWrite(&g_ioport_ctrl, g_bsp_leds.p_leds[BLUE], ON);
#define TURN_BLUE_OFF                             R_IOPORT_PinWrite(&g_ioport_ctrl, g_bsp_leds.p_leds[BLUE], OFF);

extern bsp_leds_t g_bsp_leds;

/** Function declarations **/
void usb_hcdc_task(void);


#endif /* USB_HCDC_APP_H_ */
