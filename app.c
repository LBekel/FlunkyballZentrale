/***************************************************************************//**
 * @file
 * @brief Core application logic.
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
#include <stdbool.h>
#include <math.h>
#include "em_common.h"
#include "sl_status.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_timer.h"
#include "app_log.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif // SL_COMPONENT_CATALOG_PRESENT
#ifdef SL_CATALOG_CLI_PRESENT
#include "sl_cli.h"
#endif // SL_CATALOG_CLI_PRESENT
#include "sl_sensor_rht.h"
#include "sl_health_thermometer.h"
#include "app.h"

// connection parameters
#define CONN_INTERVAL_MIN             80   //100ms
#define CONN_INTERVAL_MAX             80   //100ms
#define CONN_RESPONDER_LATENCY        0    //no latency
#define CONN_TIMEOUT                  100  //1000ms
#define CONN_MIN_CE_LENGTH            0
#define CONN_MAX_CE_LENGTH            0xffff

#define SCAN_INTERVAL                 16   //10ms
#define SCAN_WINDOW                   16   //10ms
#define SCAN_PASSIVE                  0

#define TEMP_INVALID                  NAN
#define UNIT_INVALID                  ('?')
#define UNIT_CELSIUS                  ('C')
#define UNIT_FAHRENHEIT               ('F')
#define UNIT_KILOGRAM                 ('k')
#define RSSI_INVALID                  ((int8_t)0x7F)
#define CONNECTION_HANDLE_INVALID     ((uint8_t)0xFFu)
#define SERVICE_HANDLE_INVALID        ((uint32_t)0xFFFFFFFFu)
#define CHARACTERISTIC_HANDLE_INVALID ((uint16_t)0xFFFFu)
#define TABLE_INDEX_INVALID           ((uint8_t)0xFFu)
#define TX_POWER_INVALID              ((uint8_t)0x7C)
#define TX_POWER_CONTROL_ACTIVE       ((uint8_t)0x00)
#define TX_POWER_CONTROL_INACTIVE     ((uint8_t)0x01)
#define PRINT_TX_POWER_DEFAULT        (false)
#define TEAM_INVALID              ((uint8_t)0xFF)

// Macro to translate the Flags to Celsius (C) or Fahrenheit (F). Flags is the first byte of the
// Temperature Measurement characteristic value according to the Bluetooth SIG
#define translate_flags_to_temperature_unit(flags) (((flags) & 1) ? UNIT_FAHRENHEIT : UNIT_CELSIUS)

typedef enum
{
    scanning,
    opening,
    discover_thermometer_service,
    discover_weight_service,
    discover_game_service,
    discover_thermometer_characteristics,
    discover_weight_characteristics,
    discover_team_characteristics,
    enable_indication,
    running
} conn_state_t;

typedef struct
{
    uint8_t connection_handle;
    int8_t rssi;
    bool power_control_active;
    int8_t tx_power;
    int8_t remote_tx_power;
    uint16_t server_address;
    uint32_t thermometer_service_handle;
    uint16_t thermometer_characteristic_handle;
    uint32_t weight_service_handle;
    uint16_t weight_characteristic_handle;
    uint32_t game_service_handle;
    uint16_t team_characteristic_handle;
    uint8_t team;
    float temperature;
    char unit;
    float weight;
    char weightunit;
} conn_properties_t;

typedef struct
{
    uint8_t mantissa_l;
    uint8_t mantissa_m;
    int8_t mantissa_h;
    int8_t exponent;
} IEEE_11073_float;

// Array for holding properties of multiple (parallel) connections
static conn_properties_t conn_properties[SL_BT_CONFIG_MAX_CONNECTIONS];
// Counter of active connections
static uint8_t active_connections_num;

// Connection handle.
static uint8_t app_connection = 0;

// State of the connection under establishment
static conn_state_t conn_state;

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

// Button state.
static volatile bool app_btn0_pressed = false;

// Periodic timer handle.
static sl_simple_timer_t app_periodic_timer;

// Periodic timer callback.
static void app_periodic_timer_cb(sl_simple_timer_t *timer, void *data);

static void add_connection(uint8_t connection, uint16_t address);
static void remove_connection(uint8_t connection);
static float translate_IEEE_11073_temperature_to_float(IEEE_11073_float const *IEEE_11073_value);
bool compare_uuid(uint8array *firstuuid, const uint8array *seconduuid);
void team_counter(uint8_t team, bool add);

// Find the index of a given connection in the connection_properties array
static uint8_t find_index_by_connection_handle(uint8_t connection);
static uint8_t find_service_in_advertisement(uint8_t *data, uint8_t len);

// Health Thermometer service UUID defined by Bluetooth SIG
//1809
static const uint8array thermo_service = {.len = 2 , .data = {0x09, 0x18}};

//181D
static const uint8array weight_service = {.len = 2 , .data = {0x1D, 0x18}};

//10f2fce7-1ff8-4bad-8960-092fe5f7ae8f
static const uint8array game_service = {.len = 16 , .data = {0x8f,0xae,0xf7,0xe5,
                                                0x2f,0x09,0x60,0x89,
                                                0xad,0x4b,0xf8,0x1f,
                                                0xe7,0xfc,0xf2,0x10}};
// Temperature Measurement characteristic UUID defined by Bluetooth SIG
static const uint8array thermo_char = {.len = 2 , .data = {0x1c, 0x2a}};
static const uint8array weight_char = {.len = 2 , .data = {0x9D, 0x2a}};
// 42dfdd8e-58bc-4560-9e23-000000000003
static const uint8array team_char = {.len = 16 , .data = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                          0x23, 0x9e, 0x60, 0x45, 0xbc, 0x58,
                                                          0x8e, 0xdd, 0xdf, 0x42}};
// Print out tx power value
static bool print_tx_power = PRINT_TX_POWER_DEFAULT;

// Print RSSI and temperature values
static void print_values(void);

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
    sl_status_t sc;
    uint8_t *char_value;
    uint16_t addr_value;
    uint8_t table_index;
    bd_addr address;
    uint8_t address_type;
    uint8_t system_id[8];

    // Handle stack events
    switch(SL_BT_MSG_ID(evt->header))
    {
        // -------------------------------
        // This event indicates the device has started and the radio is ready.
        // Do not call any stack command before receiving this boot event!
        case sl_bt_evt_system_boot_id:
            // Print boot message.
            app_log_info("Bluetooth stack booted: v%d.%d.%d-b%d\n\r",
                    evt->data.evt_system_boot.major,
                    evt->data.evt_system_boot.minor,
                    evt->data.evt_system_boot.patch,
                    evt->data.evt_system_boot.build);

            // Set passive scanning on 1Mb PHY
            sc = sl_bt_scanner_set_mode(sl_bt_gap_1m_phy, SCAN_PASSIVE);
            app_assert_status(sc);
            // Set scan interval and scan window
            sc = sl_bt_scanner_set_timing(sl_bt_gap_1m_phy, SCAN_INTERVAL, SCAN_WINDOW);
            app_assert_status(sc);
            // Set the default connection parameters for subsequent connections
            sc = sl_bt_connection_set_default_parameters(CONN_INTERVAL_MIN, CONN_INTERVAL_MAX, CONN_RESPONDER_LATENCY,
            CONN_TIMEOUT, CONN_MIN_CE_LENGTH, CONN_MAX_CE_LENGTH);
            app_assert_status(sc);
            // Start scanning - looking for thermometer devices
            sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
            app_assert_status_f(sc, "Failed to start discovery #1\n\r");

            // Extract unique ID from BT Address.
            sc = sl_bt_system_get_identity_address(&address, &address_type);
            app_assert_status(sc);

            // Pad and reverse unique ID to get System ID.
            system_id[0] = address.addr[5];
            system_id[1] = address.addr[4];
            system_id[2] = address.addr[3];
            system_id[3] = 0xFF;
            system_id[4] = 0xFE;
            system_id[5] = address.addr[2];
            system_id[6] = address.addr[1];
            system_id[7] = address.addr[0];

            sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id, 0, sizeof(system_id), system_id);
            app_assert_status(sc);

            app_log_info("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n\r",
                    address_type ? "static random" : "public device", address.addr[5], address.addr[4], address.addr[3],
                    address.addr[2], address.addr[1], address.addr[0]);

            // Create an advertising set.
            sc = sl_bt_advertiser_create_set(&advertising_set_handle);
            app_assert_status(sc);

            // Set advertising interval to 100ms.
            sc = sl_bt_advertiser_set_timing(advertising_set_handle, // advertising set handle
                    160, // min. adv. interval (milliseconds * 1.6)
                    160, // max. adv. interval (milliseconds * 1.6)
                    0,   // adv. duration
                    0);  // max. num. adv. events
            app_assert_status(sc);
            // Start general advertising and enable connections.
            sc = sl_bt_advertiser_start(advertising_set_handle, sl_bt_advertiser_general_discoverable,
                    sl_bt_advertiser_connectable_scannable);
            app_assert_status(sc);
            app_log_info("Started advertising\n\r");

            conn_state = scanning;

            break;
            // -------------------------------
            // This event is generated when an advertisement packet or a scan response
            // is received from a responder
        case sl_bt_evt_scanner_scan_report_id:
            // Parse advertisement packets
            if(evt->data.evt_scanner_scan_report.packet_type == 0)
            {
                // If a thermometer advertisement is found...
                if(find_service_in_advertisement(&(evt->data.evt_scanner_scan_report.data.data[0]),
                        evt->data.evt_scanner_scan_report.data.len) != 0)
                {
                    // then stop scanning for a while
                    sc = sl_bt_scanner_stop();
                    app_assert_status(sc);
                    // and connect to that device
                    if(active_connections_num < SL_BT_CONFIG_MAX_CONNECTIONS)
                    {
                        sc = sl_bt_connection_open(evt->data.evt_scanner_scan_report.address,
                                evt->data.evt_scanner_scan_report.address_type, sl_bt_gap_1m_phy,
                                NULL);
                        app_assert_status(sc);
                        conn_state = opening;
                    }
                }
            }
            break;

            // -------------------------------
            // This event is generated when a new connection is established
        case sl_bt_evt_connection_opened_id:
            // Get last two bytes of sender address
            addr_value = (uint16_t) (evt->data.evt_connection_opened.address.addr[1] << 8)
                    + evt->data.evt_connection_opened.address.addr[0];
            // Add connection to the connection_properties array
            add_connection(evt->data.evt_connection_opened.connection, addr_value);

            // Discover service on the responder device
            sc = sl_bt_gatt_discover_primary_services(evt->data.evt_connection_opened.connection);
            app_assert_status(sc);

            // Set remote connection power reporting - needed for Power Control
            sc = sl_bt_connection_set_remote_power_reporting(evt->data.evt_connection_opened.connection,
                    sl_bt_connection_power_reporting_enable);
            app_assert_status(sc);

            conn_state = discover_thermometer_service;
            break;

            // -------------------------------
            // This event is generated when a new service is discovered
        case sl_bt_evt_gatt_service_id:
            table_index = find_index_by_connection_handle(evt->data.evt_gatt_service.connection);
            if(table_index != TABLE_INDEX_INVALID)
            {
                // Save service handle for future reference
                if(compare_uuid(&(evt->data.evt_gatt_service.uuid), &thermo_service))
                {
                    conn_properties[table_index].thermometer_service_handle = evt->data.evt_gatt_service.service;
                }
                else if(compare_uuid(&(evt->data.evt_gatt_service.uuid), &weight_service))
                {
                    conn_properties[table_index].weight_service_handle = evt->data.evt_gatt_service.service;
                }
                else if(compare_uuid(&(evt->data.evt_gatt_service.uuid), &game_service))
                {
                    conn_properties[table_index].game_service_handle = evt->data.evt_gatt_service.service;
                }
            }
            break;


            // -------------------------------
            // This event is generated when a new characteristic is discovered
        case sl_bt_evt_gatt_characteristic_id:
            table_index = find_index_by_connection_handle(evt->data.evt_gatt_characteristic.connection);
            if(table_index != TABLE_INDEX_INVALID)
            {
                // Save characteristic handle for future reference
                if(compare_uuid(&(evt->data.evt_gatt_characteristic.uuid), &thermo_char))
                {
                    conn_properties[table_index].thermometer_characteristic_handle =
                        evt->data.evt_gatt_characteristic.characteristic;
                }
                else if(compare_uuid(&(evt->data.evt_gatt_characteristic.uuid), &weight_char))
                {
                    conn_properties[table_index].weight_characteristic_handle =
                        evt->data.evt_gatt_characteristic.characteristic;
                }
                else if(compare_uuid(&(evt->data.evt_gatt_characteristic.uuid), &team_char))
                {
                    conn_properties[table_index].team_characteristic_handle =
                        evt->data.evt_gatt_characteristic.characteristic;
                }
            }
            break;

            // -------------------------------
            // This event is generated when a connection is dropped
        case sl_bt_evt_connection_closed_id:
            // remove connection from active connections
            remove_connection(evt->data.evt_connection_closed.connection);
            if(conn_state != scanning)
            {
                // start scanning again to find new devices
                sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
                app_assert_status_f(sc, "Failed to start discovery #3\n\r");
                // Start general advertising and enable connections.
                sc = sl_bt_advertiser_start(advertising_set_handle, sl_bt_advertiser_general_discoverable,
                        sl_bt_advertiser_connectable_scannable);
                app_assert_status_f(sc, "Failed to start advertising\n\r");
                conn_state = scanning;
            }
            break;

            // -------------------------------
            // This event is generated for various procedure completions, e.g. when a
            // write procedure is completed, or service discovery is completed
        case sl_bt_evt_gatt_procedure_completed_id:
            table_index = find_index_by_connection_handle(evt->data.evt_gatt_procedure_completed.connection);
            if(table_index == TABLE_INDEX_INVALID)
            {
                break;

            }
            // If service discovery finished
            if(conn_state == discover_thermometer_service)
            {
                if(conn_properties[table_index].thermometer_service_handle != SERVICE_HANDLE_INVALID)
                {
                    // Discover thermometer characteristic on the responder device
                    sc = sl_bt_gatt_discover_characteristics_by_uuid(evt->data.evt_gatt_procedure_completed.connection,
                            conn_properties[table_index].thermometer_service_handle, thermo_char.len,
                            (const uint8_t*) thermo_char.data);
                    app_assert_status(sc);
                    conn_state = discover_thermometer_characteristics;
                    break;
                }
            }

            if(conn_state == discover_weight_service)
            {
                if(conn_properties[table_index].weight_service_handle != SERVICE_HANDLE_INVALID)
                {
                    // Discover weight characteristic on the responder device
                    sc = sl_bt_gatt_discover_characteristics_by_uuid(evt->data.evt_gatt_procedure_completed.connection,
                            conn_properties[table_index].weight_service_handle, weight_char.len,
                            (const uint8_t*) weight_char.data);
                    app_assert_status(sc);
                    conn_state = discover_weight_characteristics;
                    break;
                }
            }

            if(conn_state == discover_game_service)
            {
                if(conn_properties[table_index].game_service_handle != SERVICE_HANDLE_INVALID)
                {
                    // Discover weight characteristic on the responder device
                    sc = sl_bt_gatt_discover_characteristics_by_uuid(evt->data.evt_gatt_procedure_completed.connection,
                            conn_properties[table_index].game_service_handle,
                            team_char.len,
                            (const uint8_t*) team_char.data);
                    app_assert_status(sc);
                    conn_state = discover_team_characteristics;
                    break;
                }
            }

            // If characteristic discovery finished
            if(conn_state == discover_thermometer_characteristics)
            {
                if(conn_properties[table_index].thermometer_characteristic_handle != CHARACTERISTIC_HANDLE_INVALID)
                {
                    // stop discovering
                    sl_bt_scanner_stop();
                    // enable indications
                    sc = sl_bt_gatt_set_characteristic_notification(evt->data.evt_gatt_procedure_completed.connection,
                            conn_properties[table_index].thermometer_characteristic_handle, sl_bt_gatt_indication);
                    app_assert_status(sc);
                    conn_state = discover_weight_service;
                    break;
                }
            }
            if(conn_state == discover_weight_characteristics)
            {
                // If weight characteristic discovery finished
                if(conn_properties[table_index].weight_characteristic_handle != CHARACTERISTIC_HANDLE_INVALID)
                {
                    // stop discovering
                    sl_bt_scanner_stop();
                    // enable indications
                    sc = sl_bt_gatt_set_characteristic_notification(evt->data.evt_gatt_procedure_completed.connection,
                            conn_properties[table_index].weight_characteristic_handle, sl_bt_gatt_indication);
                    app_assert_status(sc);
                    conn_state = discover_game_service;
                    break;
                }
            }
            if(conn_state == discover_team_characteristics)
            {
                // If team characteristic discovery finished
                if(conn_properties[table_index].team_characteristic_handle != CHARACTERISTIC_HANDLE_INVALID)
                {
                    sc = sl_bt_gatt_read_characteristic_value(evt->data.evt_gatt_procedure_completed.connection
                            ,conn_properties[table_index].team_characteristic_handle);
                    app_assert_status(sc);
                    conn_state = enable_indication;
                    break;
                }
            }
            // If indication enable process finished
            if(conn_state == enable_indication)
            {
                // and we can connect to more devices
                if(active_connections_num < SL_BT_CONFIG_MAX_CONNECTIONS)
                {
                    // start scanning again to find new devices
                    sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
                    app_assert_status_f(sc, "Failed to start discovery #2\n\r");
                    // Start general advertising and enable connections.
                    sc = sl_bt_advertiser_start(advertising_set_handle, sl_bt_advertiser_general_discoverable,
                            sl_bt_advertiser_connectable_scannable);
                    app_assert_status_f(sc, "Failed to start advertising\n\r");
                    conn_state = scanning;
                }
                else
                {
                    conn_state = running;
                }
                break;
            }
            break;

            // -------------------------------
            // This event is generated when a characteristic value was received e.g. an indication
        case sl_bt_evt_gatt_characteristic_value_id:
            table_index = find_index_by_connection_handle(evt->data.evt_gatt_characteristic_value.connection);
            if(evt->data.evt_gatt_characteristic_value.characteristic ==
                    conn_properties[table_index].thermometer_characteristic_handle)
            {
                if(evt->data.evt_gatt_characteristic_value.value.len >= 5)
                {
                    char_value = &(evt->data.evt_gatt_characteristic_value.value.data[0]);
                    if(table_index != TABLE_INDEX_INVALID)
                    {
                        conn_properties[table_index].temperature = translate_IEEE_11073_temperature_to_float(
                                (IEEE_11073_float*) (char_value + 1));
                        conn_properties[table_index].unit = translate_flags_to_temperature_unit(char_value[0]);
                    }
                }
                else
                {
                    app_log_warning("Characteristic value too short: %d\n\r",
                            evt->data.evt_gatt_characteristic_value.value.len);
                }
                // Send confirmation for the indication
                sc = sl_bt_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);
                app_assert_status(sc);
            }
            else if(evt->data.evt_gatt_characteristic_value.characteristic ==
                    conn_properties[table_index].weight_characteristic_handle)
            {
                if(evt->data.evt_gatt_characteristic_value.value.len >= 5)
                {
                    char_value = &(evt->data.evt_gatt_characteristic_value.value.data[0]);
                    if(table_index != TABLE_INDEX_INVALID)
                    {
                        conn_properties[table_index].weight = translate_IEEE_11073_temperature_to_float(
                                (IEEE_11073_float*) (char_value + 1));
                        conn_properties[table_index].weightunit = UNIT_KILOGRAM;
                        if(table_index == 0)
                        {
                            sc = sl_bt_gatt_server_write_attribute_value(
                                      gattdb_weight_measurement_1,
                                      0,
                                      sizeof(char_value),
                                      char_value);
                            app_assert_status(sc);
                        }
                        if(table_index == 1)
                        {
                            sc = sl_bt_gatt_server_write_attribute_value(
                                      gattdb_weight_measurement_2,
                                      0,
                                      sizeof(char_value),
                                      char_value);
                            app_assert_status(sc);
                        }
                    }
                }
                else
                {
                    app_log_warning("Characteristic value too short: %d\n\r",
                            evt->data.evt_gatt_characteristic_value.value.len);
                }
                // Send confirmation for the indication
                sc = sl_bt_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);
                app_assert_status(sc);
            }
            else if(evt->data.evt_gatt_characteristic_value.characteristic ==
                    conn_properties[table_index].team_characteristic_handle)
            {
                conn_properties[table_index].team = evt->data.evt_gatt_characteristic_value.value.data[0];
                team_counter(conn_properties[table_index].team, true);
            }

            // Trigger RSSI measurement on the connection
            sc = sl_bt_connection_get_rssi(evt->data.evt_gatt_characteristic_value.connection);
            app_assert_status(sc);

            break;
            // -------------------------------
            // This event is generated when RSSI value was measured
        case sl_bt_evt_connection_rssi_id:
            table_index = find_index_by_connection_handle(evt->data.evt_connection_rssi.connection);
            if(table_index != TABLE_INDEX_INVALID)
            {
                conn_properties[table_index].rssi = evt->data.evt_connection_rssi.rssi;
            }

            print_values();
            break;

            // -------------------------------
            // TX Power is updated
        case sl_bt_evt_connection_tx_power_id:

            table_index = find_index_by_connection_handle(evt->data.evt_connection_tx_power.connection);

            if(table_index != TABLE_INDEX_INVALID)
            {
                conn_properties[table_index].tx_power = evt->data.evt_connection_tx_power.power_level;
            }

            // TX Power reporting is enabled on the other side.
            conn_properties[table_index].power_control_active =  TX_POWER_CONTROL_ACTIVE;
            break;

            // -------------------------------
            // Remote TX Power is updated
        case sl_bt_evt_connection_remote_tx_power_id:
            table_index = find_index_by_connection_handle(evt->data.evt_connection_remote_tx_power.connection);

            if(table_index != TABLE_INDEX_INVALID)
            {
                conn_properties[table_index].remote_tx_power = evt->data.evt_connection_remote_tx_power.power_level;
            }
            break;
            // -------------------------------
            // Default event handler.
        default:
            break;
    }
}

/**************************************************************************//**
 * Callback function of connection close event.
 *
 * @param[in] reason Unused parameter required by the health_thermometer component
 * @param[in] connection Unused parameter required by the health_thermometer component
 *****************************************************************************/
void sl_bt_connection_closed_cb(uint16_t reason, uint8_t connection)
{
    (void) reason;
    (void) connection;
    sl_status_t sc;

    // Stop timer.
    sc = sl_simple_timer_stop(&app_periodic_timer);
    app_assert_status(sc);
}

// Parse advertisements looking for advertised Health Thermometer service
static uint8_t find_service_in_advertisement(uint8_t *data, uint8_t len)
{
    uint8_t ad_field_length;
    uint8_t ad_field_type;
    uint8_t i = 0;
    // Parse advertisement packet
    while(i < len)
    {
        ad_field_length = data[i];
        ad_field_type = data[i + 1];
        // Partial ($02) or complete ($03) list of 16-bit UUIDs
        if(ad_field_type == 0x02 || ad_field_type == 0x03)
        {
            // compare UUID to Health Thermometer service UUID
            if(memcmp(&data[i + 2], thermo_service.data, 2) == 0)
            {
                return 1;
            }
        }
        // advance to the next AD struct
        i = i + ad_field_length + 1;
    }
    return 0;
}

// Find the index of a given connection in the connection_properties array
static uint8_t find_index_by_connection_handle(uint8_t connection)
{
    for(uint8_t i = 0; i < active_connections_num; i++)
    {
        if(conn_properties[i].connection_handle == connection)
        {
            return i;
        }
    }
    return TABLE_INDEX_INVALID;
}

// Add a new connection to the connection_properties array
static void add_connection(uint8_t connection, uint16_t address)
{
    conn_properties[active_connections_num].connection_handle = connection;
    conn_properties[active_connections_num].server_address = address;
    active_connections_num++;
}

// Remove a connection from the connection_properties array
static void remove_connection(uint8_t connection)
{
    uint8_t i;
    uint8_t table_index = find_index_by_connection_handle(connection);

    // inc team counter
    team_counter(conn_properties[table_index].team, false);

    if(active_connections_num > 0)
    {
        active_connections_num--;
    }
    // Shift entries after the removed connection toward 0 index
    for(i = table_index; i < active_connections_num; i++)
    {
        conn_properties[i] = conn_properties[i + 1];
    }
    // Clear the slots we've just removed so no junk values appear
    for(i = active_connections_num; i < SL_BT_CONFIG_MAX_CONNECTIONS; i++)
    {
        conn_properties[i].connection_handle = CONNECTION_HANDLE_INVALID;
        conn_properties[i].thermometer_service_handle = SERVICE_HANDLE_INVALID;
        conn_properties[i].thermometer_characteristic_handle = CHARACTERISTIC_HANDLE_INVALID;
        conn_properties[i].weight_service_handle= SERVICE_HANDLE_INVALID;
        conn_properties[i].weight_characteristic_handle = CHARACTERISTIC_HANDLE_INVALID;
        conn_properties[i].game_service_handle= SERVICE_HANDLE_INVALID;
        conn_properties[i].team_characteristic_handle = CHARACTERISTIC_HANDLE_INVALID;
        conn_properties[i].temperature = TEMP_INVALID;
        conn_properties[i].unit = UNIT_INVALID;
        conn_properties[i].rssi = RSSI_INVALID;
        conn_properties[i].power_control_active = TX_POWER_CONTROL_INACTIVE;
        conn_properties[i].tx_power = TX_POWER_INVALID;
        conn_properties[i].remote_tx_power = TX_POWER_INVALID;
        conn_properties[i].team = TEAM_INVALID;
    }
}


// Translate a IEEE-11073 Temperature Value to a float Value
static float translate_IEEE_11073_temperature_to_float(IEEE_11073_float const *IEEE_11073_value)
{
    int32_t mantissa = 0;
    uint8_t mantissa_l;
    uint8_t mantissa_m;
    int8_t mantissa_h;
    int8_t exponent;

    // Wrong Argument: NULL pointer is passed
    if(!IEEE_11073_value)
    {
        return NAN;
    }

    // Caching Fields
    mantissa_l = IEEE_11073_value->mantissa_l;
    mantissa_m = IEEE_11073_value->mantissa_m;
    mantissa_h = IEEE_11073_value->mantissa_h;
    exponent = IEEE_11073_value->exponent;

    // IEEE-11073 Standard NaN Value Passed
    if((mantissa_l == 0xFF) && (mantissa_m == 0xFF) && (mantissa_h == 0x7F) && (exponent == 0x00))
    {
        return NAN;
    }

    // Converting a 24bit Signed Value to a 32bit Signed Value
    mantissa |= mantissa_h;
    mantissa <<= 8;
    mantissa |= mantissa_m;
    mantissa <<= 8;
    mantissa |= mantissa_l;
    mantissa <<= 8;
    mantissa >>= 8;

    return ((float) mantissa) * pow(10.0f, (float) exponent);
}

/**************************************************************************//**
 * Health Thermometer - Temperature Measurement
 * Indication changed callback
 *
 * Called when indication of temperature measurement is enabled/disabled by
 * the client.
 *****************************************************************************/
void sl_bt_ht_temperature_measurement_indication_changed_cb(uint8_t connection,
        sl_bt_gatt_client_config_flag_t client_config)
{
    sl_status_t sc;
    app_connection = connection;
    // Indication or notification enabled.
    if(sl_bt_gatt_disable != client_config)
    {
        // Start timer used for periodic indications.
        sc = sl_simple_timer_start(&app_periodic_timer,
        SL_BT_HT_MEASUREMENT_INTERVAL_SEC * 1000, app_periodic_timer_cb,
        NULL,
        true);
        app_assert_status(sc);
        // Send first indication.
        app_periodic_timer_cb(&app_periodic_timer, NULL);
    }
    // Indications disabled.
    else
    {
        // Stop timer used for periodic indications.
        (void) sl_simple_timer_stop(&app_periodic_timer);
    }
}

/**************************************************************************//**
 * Simple Button
 * Button state changed callback
 * @param[in] handle Button event handle
 *****************************************************************************/
void sl_button_on_change(const sl_button_t *handle)
{
    // Button pressed.
    if(sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED)
    {
        if(&sl_button_btn0 == handle)
        {
            app_btn0_pressed = true;
        }
    }
    // Button released.
    else if(sl_button_get_state(handle) == SL_SIMPLE_BUTTON_RELEASED)
    {
        if(&sl_button_btn0 == handle)
        {
            app_btn0_pressed = false;
        }
    }
}

/**************************************************************************//**
 * Timer callback
 * Called periodically to time periodic temperature measurements and indications.
 *****************************************************************************/
static void app_periodic_timer_cb(sl_simple_timer_t *timer, void *data)
{
    (void) data;
    (void) timer;
    sl_status_t sc;
    int32_t temperature = 0;
    uint32_t humidity = 0;
    float tmp_c = 0.0;
    // float tmp_f = 0.0;
    uint8_t buf[17] = { 0 };
    size_t readlen = 0;

    // Measure temperature; units are % and milli-Celsius.
    sc = sl_sensor_rht_get(&humidity, &temperature);
    if(sc != SL_STATUS_OK)
    {
        app_log_warning("Invalid RHT reading: %lu %ld\n\r", humidity, temperature);
    }

    // button 0 pressed: overwrite temperature with -20C.
    if(app_btn0_pressed)
    {
        temperature = -20 * 1000;
    }

    tmp_c = (float) temperature / 1000;
    //app_log_info("Temperature: %5.2f C\n\r", tmp_c);
    // Send temperature measurement indication to connected client.


    sc = sl_bt_ht_temperature_measurement_indicate(app_connection, temperature, false);
    if(sc)
    {
        app_log_warning("Failed to send temperature measurement indication\n\r");
    }

    sc = sl_bt_gatt_server_read_attribute_value(gattdb_temperature_measurement, 0, sizeof(buf), &readlen, buf);
    app_assert_status(sc);

}

// Print parameters to STDOUT. CR used to display results.
void print_values(void)
{
    static bool print_header = true;
    static bool previous_print_tx_power = PRINT_TX_POWER_DEFAULT;
    uint8_t i;

    // If TX power print request changes - header should be updated.
    if(previous_print_tx_power != print_tx_power)
    {
        previous_print_tx_power = print_tx_power;
        print_header = true;
    }

    // Print header
    if(true == print_header)
    {
        app_log_info("");
        for(i = 0u; i < SL_BT_CONFIG_MAX_CONNECTIONS; i++)
        {
            if(false == print_tx_power)
            {
                app_log_append("ADDR   TEMP   WEIGHT  RSSI |");
            }
            else
            {
                app_log_append("ADDR   TEMP   WEIGHT  RSSI    TXPW |");
            }
        }
        app_log_append("\n\r");

        print_header = false;
    }

    app_log_info("");
    // Print parameters
    for(i = 0u; i < SL_BT_CONFIG_MAX_CONNECTIONS; i++)
    {
        if((TEMP_INVALID != conn_properties[i].temperature) && (RSSI_INVALID != conn_properties[i].rssi))
        {
            app_log_append("%04x ", conn_properties[i].server_address);
            app_log_append("%6.2f", conn_properties[i].temperature);
            app_log_append("%c ", conn_properties[i].unit);
            app_log_append("%6.2f", conn_properties[i].weight);
            app_log_append("%c ", conn_properties[i].weightunit);
            app_log_append("% 3d", conn_properties[i].rssi);
            app_log_append("dBm");
            if(true == print_tx_power)
            {
                app_log_append(" %4d", conn_properties[i].tx_power);
                app_log_append("dBm");
            }
            app_log_append("|");
        }
        else if(false == print_tx_power)
        {
            app_log_append("---- ------- ------- ------|");
        }
        else
        {
            app_log_append("----  ------ ------ ------  ------|");
        }
    }
    app_log_append("\r");
}

bool compare_uuid(uint8array *firstuuid, const uint8array *seconduuid)
{
    if(firstuuid->len == seconduuid->len)
    {
        if(memcmp(firstuuid->data, seconduuid->data, firstuuid->len) == 0)
        {
            return true;
        }
    }
    return false;
}

void team_counter(uint8_t team, bool add)
{
    uint8_t attribute = 0;
    uint8_t team_count = 0;
    size_t readlen = 0;
    sl_status_t sc;
    if(team != TEAM_INVALID)
    {
        if(team == 0)
        {
            attribute = gattdb_team1_count;
        }
        else
        {
            attribute = gattdb_team2_count;
        }

        sc = sl_bt_gatt_server_read_attribute_value(attribute, 0, sizeof(team_count), &readlen, &team_count);
        app_assert_status(sc);
        if(add)
        {
            team_count++;
        }
        else
        {
            team_count--;
        }
        sc = sl_bt_gatt_server_write_attribute_value(attribute, 0, sizeof(team_count), &team_count);
        app_assert_status(sc);
    }
}

#ifdef SL_CATALOG_CLI_PRESENT
void hello(sl_cli_command_arg_t *arguments)
{
    (void) arguments;
    bd_addr address;
    uint8_t address_type;
    sl_status_t sc = sl_bt_system_get_identity_address(&address, &address_type);
    app_assert_status(sc);
    app_log_info("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n\r",
            address_type ? "static random" : "public device", address.addr[5], address.addr[4], address.addr[3],
            address.addr[2], address.addr[1], address.addr[0]);
}
#endif // SL_CATALOG_CLI_PRESENT
