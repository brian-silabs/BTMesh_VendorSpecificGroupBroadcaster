/***************************************************************************//**
 * @file
 * @brief Empty BTmesh NCP-host Example Project.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
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

/* standard library headers */
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include "system.h"
#include "sl_bt_api.h"
#include "sl_btmesh_api.h"
#include "sl_btmesh_ncp_host.h"
#include "sl_bt_ncp_host.h"
#include "app_log.h"
#include "app_assert.h"
#include "uart.h"
#include "app.h"
#include "tcp.h"
#include <unistd.h>
#ifdef POSIX
#include "named_socket.h"

#define CLIENT_ENCRYPTED_PATH "client_encrypted"
#define CLIENT_UNENCRYPTED_PATH "client_unencrypted"


#include "vendor_model.h"

/** Usage string for Posix systems */
#define USAGE "\n%s\n"                                                                         \
              "Usage: -u <serial port> <baud rate> [flow control: 1(on, default) or 0(off)]\n" \
              "       -t <IPv4 address (e.g. 192.168.0.0)>\n"                                  \
              "       -n <server_domain_socket> -s (is_domain_socket_encrypted? 0/1)\n\n"
#else
/** Usage string for non-Posix systems */
#define USAGE "\n%s\n"                                                                         \
              "Usage: -u <serial port> <baud rate> [flow control: 1(on, default) or 0(off)]\n" \
              "       -t <IPv4 address (e.g. 192.168.0.0)>\n\n"
#endif
#define DEFAULT_UART_PORT             NULL
#define DEFAULT_UART_BAUD_RATE        115200
#define DEFAULT_UART_FLOW_CONTROL     1
#define DEFAULT_UART_TIMEOUT          100
#define DEFAULT_TCP_PORT              "4901"
#define MAX_OPT_LEN                   255

// This characteristic handle value has to match the value in gatt_db.h of
// NCP empty example running on the connected WSTK.
#define GATTDB_SYSTEM_ID 18

static bool enable_security = false;

// Serail port name of the NCP target
static char uart_target_port[MAX_OPT_LEN];
// IP address or host name of the NCP target using TCP connection
static char tcp_target_address[MAX_OPT_LEN];

#ifdef POSIX
static char named_socket_target_address[MAX_OPT_LEN];
#endif


////////////////// Example specific Data //////////////////////
my_model_t my_model = {
  .elem_index = PRIMARY_ELEMENT,
  .vendor_id = MY_VENDOR_ID,
  .model_id = MY_MODEL_SERVER_ID,
  .publish = 1,
  .opcodes_len = NUMBER_OF_OPCODES,
  .opcodes_data[0] = custom_get,
  .opcodes_data[1] = custom_set};

uint16_t my_address = 0xFFFF;


static int serial_port_init(char* uart_port, uint32_t uart_baud_rate,
                            uint32_t uart_flow_control, int32_t timeout);
static void app_deinit(int cause);
static void uart_tx_wrapper(uint32_t len, uint8_t *data);
static void tcp_tx_wrapper(uint32_t len, uint8_t *data);

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(int argc, char *argv[])
{
  int opt;
  uint32_t target_baud_rate = DEFAULT_UART_BAUD_RATE;
  uint32_t target_flow_control = DEFAULT_UART_FLOW_CONTROL;

  uart_target_port[0] = '\0';
  tcp_target_address[0] = '\0';

  //Parse command line arguments
  while ((opt = getopt(argc, argv, "t:T:u:U:b:B:f:F:n:N:sShH")) != -1) {
    switch (opt) {
      case 'U':
      case 'u': //Target Uart port or address.
        strncpy(uart_target_port, optarg, MAX_OPT_LEN);
        break;
      case 'T':
      case 't': //Target TCP address
        strncpy(tcp_target_address, optarg, MAX_OPT_LEN);
        break;
      case 'F':
      case 'f': //Target flow control
        target_flow_control = atol(optarg);
        break;
      case 'B':
      case 'b': //Target baud rate
        target_baud_rate = atol(optarg);
        break;
      case 'S':
      case 's':
        enable_security = true;
        break;
#ifdef POSIX
      case 'N':
      case 'n':
        strncpy(named_socket_target_address, optarg, MAX_OPT_LEN);
        break;
#endif
      case 'H':
      case 'h': //Help!
        app_log(USAGE, argv[0]);
        exit(EXIT_SUCCESS);
      default: /* '?' */
        app_log(USAGE, argv[0]);
        exit(EXIT_FAILURE);
    }
  }
  if (uart_target_port[0] != '\0') {
    // Initialise serial communication as non-blocking.
    SL_BT_API_INITIALIZE_NONBLOCK(uart_tx_wrapper, uartRx, uartRxPeek);
    if (serial_port_init(uart_target_port, target_baud_rate,
                         target_flow_control, DEFAULT_UART_TIMEOUT) < 0) {
      app_log("Non-blocking serial port init failure\n");
      exit(EXIT_FAILURE);
    }
  } else if (tcp_target_address[0] != '\0') {
    // Initialise socket communication
    SL_BT_API_INITIALIZE_NONBLOCK(tcp_tx_wrapper, tcp_rx, tcp_rx_peek);
    if (tcp_open(tcp_target_address, DEFAULT_TCP_PORT) < 0) {
      app_log("Non-blocking TCP connection init failure\n");
      exit(EXIT_FAILURE);
    }
#ifdef POSIX
  } else if (named_socket_target_address[0] != '\0') {
    // Initialise serial communication as non-blocking.
    SL_BT_API_INITIALIZE_NONBLOCK(af_socket_tx, af_socket_rx,
                                  af_socket_rx_peek);
    if (enable_security) {
      if (connect_domain_socket_server(named_socket_target_address,
                                       CLIENT_ENCRYPTED_PATH, 1)) {
        app_log("Connection to encrypted domain socket unsuccessful. Exiting..\n");
        exit(EXIT_FAILURE);
      }
      app_log("Turning on Encryption. All subsequent BGAPI commands and events will be encrypted..\n");
      turn_encryption_on();
    } else {
      if (connect_domain_socket_server(named_socket_target_address,
                                       CLIENT_UNENCRYPTED_PATH, 0)) {
        app_log("Connection to unencrypted domain socket unsuccessful. Exiting..\n");
        exit(EXIT_FAILURE);
      }
    }
#endif
  } else {
    app_log("Either uart port or TCP address shall be given.\n");
    app_log(USAGE, argv[0]);
    exit(EXIT_FAILURE);
  }

  SL_BTMESH_API_REGISTER();

  app_log("Empty NCP-host initialised\n");
  app_log("Resetting NCP...\n");
  // Reset NCP to ensure it gets into a defined state.
  // Once the chip successfully boots, boot event should be received.

  sl_bt_system_reset(0);

  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:

      // Print boot message.
      app_log("Bluetooth stack booted: v%d.%d.%d-b%d\n",
              evt->data.evt_system_boot.major,
              evt->data.evt_system_boot.minor,
              evt->data.evt_system_boot.patch,
              evt->data.evt_system_boot.build);

      // Initialize Mesh stack in Node operation mode,
      // wait for initialized event
      app_log("Node init\r\n");
      sc = sl_btmesh_node_init();
      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to init node\n",
                 (int)sc);
      break;

    case sl_bt_evt_system_soft_timer_id:
      if(evt->data.evt_system_soft_timer.handle == TIMER_ID_PERIODIC_SEND)
      {
        uint8_t opcode = custom_set;
        uint8_t length = CUSTOM_DATA_LENGTH;
        uint8_t payloadData[4] = {0xDE, 0xAD, 0xCA, 0xFE};

        sc = sl_btmesh_vendor_model_set_publication(my_model.elem_index, my_model.vendor_id, my_model.model_id, opcode, 1, length, payloadData);//1 here means final packet
        if (sc == SL_STATUS_OK) 
        {
          sc = sl_btmesh_vendor_model_publish(my_model.elem_index, my_model.vendor_id, my_model.model_id);
          if (sc == SL_STATUS_OK)
          {
            app_log("Periodic Publication Done\r\n");
            sl_bt_system_set_soft_timer(TIMER_MS_2_TIMERTICK(CUSTOM_PAYLOAD_TX_PERIOD_MS), TIMER_ID_PERIODIC_SEND, SOFT_TIMER_ONE_SHOT);
          } else {
            app_log("Periodic Publication Failed\r\n");
          }
        } else {
          app_log("Periodic Set Publication Failed\r\n");
        }
      }
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

/**************************************************************************//**
 * Bluetooth Mesh stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth Mesh stack.
 *****************************************************************************/
void sl_btmesh_on_event(sl_btmesh_msg_t *evt)
{
  sl_status_t sc;
  uint8_t i = 0;

  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_btmesh_evt_node_initialized_id:
      app_log("Initialized\r\n");

      sc = sl_btmesh_vendor_model_init(my_model.elem_index, my_model.vendor_id, my_model.model_id, my_model.publish, my_model.opcodes_len, my_model.opcodes_data);
      app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to init Vendor Model\n",
                (int)sc);

      if (!evt->data.evt_node_initialized.provisioned) {
        // The Node is now initialized,
        // start unprovisioned Beaconing using PB-ADV and PB-GATT Bearers
        app_log("Unprovisioned\r\n");
        sc = sl_btmesh_node_start_unprov_beaconing(0x3);
        app_assert(sc == SL_STATUS_OK,
                   "[E: 0x%04x] Failed to start unprovisioned beaconing\n",
                   (int)sc);
      } else {
        app_log("Device was previously provisioned IV_Index: %d, Address : 0x%4X\r\n", evt->data.evt_node_initialized.iv_index, evt->data.evt_node_initialized.address);
        my_address = evt->data.evt_node_initialized.address;
        sl_bt_system_set_soft_timer(TIMER_MS_2_TIMERTICK(CUSTOM_PAYLOAD_TX_PERIOD_MS), TIMER_ID_PERIODIC_SEND, SOFT_TIMER_ONE_SHOT);
      }
      break;

    case sl_btmesh_evt_node_provisioned_id:
      app_log("Device provisioned IV_Index: %d, Address : 0x%4X\r\n", evt->data.evt_node_provisioned.iv_index, evt->data.evt_node_provisioned.address);
      my_address = evt->data.evt_node_provisioned.address;
      break;

    case sl_btmesh_evt_node_provisioning_failed_id:
      app_log("Provisioning failed. Result = 0x%04x\r\n", evt->data.evt_node_provisioning_failed.result);
      break;

    case sl_btmesh_evt_node_provisioning_started_id:
      app_log("Provisioning started.\r\n");
      break;

    case sl_btmesh_evt_node_key_added_id:
      app_log("got new %s key with index %x\r\n", evt->data.evt_node_key_added.type == 0 ? "network" : "application", evt->data.evt_node_key_added.index);
      break;

    case sl_btmesh_evt_node_model_config_changed_id:
      app_log("Model config changed\r\n\r\n");

      if(evt->data.evt_node_model_config_changed.node_config_state == 0x00)
      {
        app_log("Model Application Key Binding\r\n");
      } else if(evt->data.evt_node_model_config_changed.node_config_state == 0x01)
      {
        app_log("Model Publication Parameters\r\n");
      } else if(evt->data.evt_node_model_config_changed.node_config_state == 0x02)
      {
        app_log("Model Subscription List\r\n");
        sl_bt_system_set_soft_timer(TIMER_MS_2_TIMERTICK(CUSTOM_PAYLOAD_TX_PERIOD_MS), TIMER_ID_PERIODIC_SEND, SOFT_TIMER_ONE_SHOT);
      } else {
        app_log("Unknown change\r\n");
      }

      app_log("Element Address: 0x%4X\r\n\r\n", evt->data.evt_node_model_config_changed.element_address);
      app_log("Vendor ID: 0x%4X\r\n", evt->data.evt_node_model_config_changed.vendor_id);
      app_log("Model ID: 0x%4X\r\n\r\n", evt->data.evt_node_model_config_changed.model_id);

      break;

    case sl_btmesh_evt_vendor_model_receive_id:
      app_log("vendor_model_received\r\n");
      if (my_address == evt->data.evt_vendor_model_receive.source_address)
      {
        app_log("From self, skipping\r\n");
      } else {
        app_log("Destination Address: 0x%4X ", evt->data.evt_vendor_model_receive.destination_address);
        app_log("Element Index: 0x%4X ", evt->data.evt_vendor_model_receive.elem_index);
        app_log("Vendor ID: 0x%4X", evt->data.evt_vendor_model_receive.vendor_id);
        app_log("Model ID: 0x%4X", evt->data.evt_vendor_model_receive.model_id);
        app_log("Source Address: 0x%4X ", evt->data.evt_vendor_model_receive.source_address);
        app_log("VA Index: 0x%4X ", evt->data.evt_vendor_model_receive.va_index);
        app_log("App Key Index: 0x%4X ", evt->data.evt_vendor_model_receive.appkey_index);
        app_log("Non Relayed: 0x%4X ", evt->data.evt_vendor_model_receive.nonrelayed);
        app_log("Op Code: 0x%4X ", evt->data.evt_vendor_model_receive.opcode);
        app_log("Final ? 0x%4X ", evt->data.evt_vendor_model_receive.final);
        app_log("\r\nPayload Len %d bytes \r\n", evt->data.evt_vendor_model_receive.payload.len);
        for ( i = 0; i < evt->data.evt_vendor_model_receive.payload.len; i++)
        {
          app_log("0x%2X ", evt->data.evt_vendor_model_receive.payload.data[i]);
        }
        app_log("\r\n");
      }

      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

/**************************************************************************//**
 * UART TX Wrapper.
 *****************************************************************************/
static void uart_tx_wrapper(uint32_t len, uint8_t *data)
{
  if (0 > uartTx(len, data)) {
    app_log("Failed to write to serial port\n");
    app_deinit(0);
    exit(EXIT_FAILURE);
  }
}

/**************************************************************************//**
 * TCP TX Wrapper.
 *****************************************************************************/
static void tcp_tx_wrapper(uint32_t len, uint8_t *data)
{
  if (0 > tcp_tx(len, data)) {
    app_log("Failed to write to TCP port\n");
    app_deinit(0);
    exit(EXIT_FAILURE);
  }
}

/**************************************************************************//**
 * Initialise serial port.
 *****************************************************************************/
static int serial_port_init(char* uart_port, uint32_t uart_baud_rate,
                            uint32_t uart_flow_control, int32_t timeout)
{
  int ret;

  // Sanity check of arguments.
  if (!uart_port || !uart_baud_rate || (uart_flow_control > 1)) {
    app_log("Serial port setting error.\n");
    ret = -1;
  } else {
    // Initialise the serial port.
    ret = uartOpen((int8_t*)uart_port, uart_baud_rate, uart_flow_control, timeout);
  }

  return ret;
}

void app_deinit(int cause)
{
  (void)cause;
  app_log("Shutting down.\n");
  if (uart_target_port[0] != '\0') {
    uartClose();
  } else if (tcp_target_address[0] != '\0') {
    tcp_close();
  }
}
