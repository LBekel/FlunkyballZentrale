/***************************************************************************//**
 * @file
 * @brief Application interface provided to main().
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#ifndef APP_H
#define APP_H

#include "em_common.h"
typedef struct
{
    uint8_t connection_handle;
    int8_t rssi;
    bool power_control_active;
    int8_t tx_power;
    int8_t remote_tx_power;
    uint16_t server_address;
    uint32_t weight_service_handle;
    uint16_t weight_characteristic_handle;
    uint32_t game_service_handle;
    uint16_t team_characteristic_handle;
    uint8_t team;
    float weight;
    char weightunit;
} conn_properties_t;

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void);

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void);
void get_player_data(conn_properties_t *conn_properties_ext, uint8_t player);
bool getbuttonstate(void);

#endif // APP_H
