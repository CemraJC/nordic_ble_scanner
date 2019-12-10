/*
 * copyright (c) 2012 - 2018, nordic semiconductor asa
 * all rights reserved.
 *
 * redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. redistributions in binary form, except as embedded into a nordic
 *    semiconductor asa integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 /**@brief BLE Scanner application
  *
  * This file contains the source code for a simple application that scans the local environment
  * for BLE devices and reports their addresses and complete local names
  *
  * Structure of this file
  * - Includes
  * - Definitions
  * - Global variables
  * - Global functions
  * - Event functions
  * - Event dispatcher
  * - Main
  */

  /** Includes */
#include "ble.h"
#include "sd_rpc.h"
#include "sockpp/tcp_acceptor.h"
#include "ScannerClient.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <regex>
#include <mutex>
#include <cstdarg>
#include <cxxopts.hpp>


/** Prototypes */

static void on_client(sockpp::tcp_socket socket);

/** Definitions */
#define DEFAULT_BAUD_RATE 1000000 /**< The baud rate to be used for serial communication with nRF5 device. */
#define VERSION_MAJOR 1
#define VERSION_MINOR 0

#ifdef _WIN32
#define DEFAULT_UART_PORT_NAME "COM1"
#endif
#ifdef __APPLE__
#define DEFAULT_UART_PORT_NAME "/dev/tty.usbmodem00000"
#endif
#ifdef __linux__
#define DEFAULT_UART_PORT_NAME "/dev/ttyACM0"
#endif

enum
{
	UNIT_0_625_MS = 625,  /**< Number of microseconds in 0.625 milliseconds. */
	UNIT_1_25_MS = 1250, /**< Number of microseconds in 1.25 milliseconds. */
	UNIT_10_MS = 10000 /**< Number of microseconds in 10 milliseconds. */
};

#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))
#define CHECK_ERROR(CODE) if ((CODE) != NRF_SUCCESS) { return (CODE); }

#define SCAN_INTERVAL 0x00A0 /**< Determines scan interval in units of 0.625 milliseconds. */
#define SCAN_WINDOW   0x0050 /**< Determines scan window in units of 0.625 milliseconds. */
#define SCAN_TIMEOUT  0x0    /**< Scan timeout between 0x01 and 0xFFFF in seconds, 0x0 disables timeout. */

#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY                   0                                /**< Slave Latency in number of connection events. */
#define CONNECTION_SUPERVISION_TIMEOUT  MSEC_TO_UNITS(4000, UNIT_10_MS)  /**< Determines supervision time-out in units of 10 milliseconds. */

#define TARGET_DEV_NAME "Nordic_HRM" /**< Connect to a peripheral using a given advertising name here. */
#define MAX_PEER_COUNT 1            /**< Maximum number of peer's application intends to manage. */

#define BLE_UUID_HEART_RATE_SERVICE          0x180D /**< Heart Rate service UUID. */
#define BLE_UUID_HEART_RATE_MEASUREMENT_CHAR 0x2A37 /**< Heart Rate Measurement characteristic UUID. */
#define BLE_UUID_CCCD                        0x2902
#define BLE_CCCD_NOTIFY                      0x01

#define STRING_BUFFER_SIZE 256
#define CLIENT_CMD_BUFFER 512
#define LOG_BUFFER_SIZE 512

typedef struct
{
	uint8_t *     p_data;   /**< Pointer to data. */
	uint16_t      data_len; /**< Length of data. */
} data_t;


/** Global variables */
static bool		   verbose_mode = false;
static uint32_t	   server_port = 0;
static sockpp::tcp_acceptor server; // Global server object, close to exit server
static uint8_t     m_connected_devices = 0;
static uint16_t    m_connection_handle = 0;
static uint16_t    m_service_start_handle = 0;
static uint16_t    m_service_end_handle = 0;
static uint16_t    m_hrm_char_handle = 0;
static uint16_t    m_hrm_cccd_handle = 0;
static bool        m_connection_is_in_progress = false;
static adapter_t * m_adapter = NULL;

static std::mutex log_lock;
static std::mutex server_exit;
static std::mutex client_access_lock;
static std::vector<ScannerClient*> clients;

/* Server Global Primitives */

#if NRF_SD_BLE_API >= 5
static uint32_t    m_config_id = 1;
#endif

#if NRF_SD_BLE_API >= 6
static uint8_t     mp_data[100] = { 0 };
static ble_data_t  m_adv_report_buffer;
#endif

static const ble_gap_scan_params_t m_scan_param =
{
#if NRF_SD_BLE_API >= 6
	0,                       // Set if accept extended advertising packetets.
	0,                       // Set if report inomplete reports.
#endif
	0,                       // Set if active scanning.
#if NRF_SD_BLE_API < 6
	0,                       // Set if selective scanning.
#endif
#if NRF_SD_BLE_API >= 6
	BLE_GAP_SCAN_FP_ACCEPT_ALL,
	BLE_GAP_PHY_1MBPS,
#endif
#if NRF_SD_BLE_API == 2
	NULL,                    // Set white-list.
#endif
#if NRF_SD_BLE_API == 3 || NRF_SD_BLE_API == 5
	0,                       // Set adv_dir_report.
#endif
	(uint16_t)SCAN_INTERVAL,
	(uint16_t)SCAN_WINDOW,
	(uint16_t)SCAN_TIMEOUT
#if NRF_SD_BLE_API >= 6
	, { 0 }                  // Set chennel mask.
#endif
};

static const ble_gap_conn_params_t m_connection_param =
{
	(uint16_t)MIN_CONNECTION_INTERVAL,
	(uint16_t)MAX_CONNECTION_INTERVAL,
	(uint16_t)SLAVE_LATENCY,
	(uint16_t)CONNECTION_SUPERVISION_TIMEOUT
};


/** Global functions */

/**@brief Function for handling error message events from sd_rpc.
 *
 * @param[in] adapter The transport adapter.
 * @param[in] code Error code that the error message is associated with.
 * @param[in] message The error message that the callback is associated with.
 */
static void status_handler(adapter_t * adapter, sd_rpc_app_status_t code, const char * message)
{
	if (verbose_mode) 
	{
		printf("Status: %d, message: %s\n", (uint32_t)code, message);
		fflush(stdout);
	}
}

/**@brief Function for handling the log messages in the application
 *
 * @param[in] severity Level of severity that the log message is associated with.
 * @param[in] message The log message that the callback is associated with.
 */
static void log(sd_rpc_log_severity_t severity, const char* aFormat, ...) {

	log_lock.lock();

	char message[LOG_BUFFER_SIZE] = { 0 };

	va_list argptr;
	va_start(argptr, aFormat);

	vsprintf(message, aFormat, argptr);

	va_end(argptr);

	switch (severity)
	{
	case SD_RPC_LOG_ERROR:
		printf("Error: %s\n", message);
		fflush(stdout);
		break;

	case SD_RPC_LOG_WARNING:
		printf("Warning: %s\n", message);
		fflush(stdout);
		break;

	case SD_RPC_LOG_INFO:
		if (verbose_mode) {
			printf("Info: %s\n", message);
			fflush(stdout);
		}
		break;

	default:
		if (verbose_mode) {
			printf("Log: %s\n", message);
			fflush(stdout);
		}
		break;
	}

	log_lock.unlock();
}

/**@brief Function for handling the log message events from sd_rpc.
 *
 * @param[in] adapter The transport adapter.
 * @param[in] severity Level of severity that the log message is associated with.
 * @param[in] message The log message that the callback is associated with.
 */
static void log_handler(adapter_t * adapter, sd_rpc_log_severity_t severity, const char * message)
{
	log(severity, message);
}

/**@brief Function for initializing serial communication with the target nRF5 Bluetooth slave.
 *
 * @param[in] serial_port The serial port the target nRF5 device is connected to.
 *
 * @return The new transport adapter.
 */
static adapter_t * adapter_init(char * serial_port, uint32_t baud_rate)
{
	physical_layer_t  * phy;
	data_link_layer_t * data_link_layer;
	transport_layer_t * transport_layer;

	phy = sd_rpc_physical_layer_create_uart(serial_port,
		baud_rate,
		SD_RPC_FLOW_CONTROL_NONE,
		SD_RPC_PARITY_NONE);
	data_link_layer = sd_rpc_data_link_layer_create_bt_three_wire(phy, 250);
	transport_layer = sd_rpc_transport_layer_create(data_link_layer, 1500);
	return sd_rpc_adapter_create(transport_layer);
}

/**@brief Function for converting a BLE address to a string.
 *
 * @param[in] address       Bluetooth Low Energy address.
 * @param[out] string_buffer The serial port the target nRF5 device is connected to.
 */
static void ble_address_to_string_convert(ble_gap_addr_t address, uint8_t * string_buffer)
{
	const int address_length = 6;
	char      temp_str[3];

	for (int i = address_length - 1; i >= 0; --i)
	{
		sprintf(temp_str, "%02X", address.addr[i]);
		strcat((char *)string_buffer, temp_str);

		if (i > 0) 
		{
			strcat((char *)string_buffer, ":");
		}
	}
}

/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
	uint32_t  index = 0;
	uint8_t * p_data;

	p_data = p_advdata->p_data;

	while (index < p_advdata->data_len)
	{
		uint8_t field_length = p_data[index];
		uint8_t field_type = p_data[index + 1];

		if (field_type == type)
		{
			p_typedata->p_data = &p_data[index + 2];
			p_typedata->data_len = field_length - 1;
			return NRF_SUCCESS;
		}
		index += field_length + 1;
	}
	return NRF_ERROR_NOT_FOUND;
}

/**@brief Function for searching a given name in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a given
 * name in them either as 'complete_local_name' or as 'short_local_name'.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   name_to_find   name to search.
 * @return   true if the given name was found, false otherwise.
 */
static bool find_adv_name(const ble_gap_evt_adv_report_t *p_adv_report, const char * name_to_find)
{
	uint32_t err_code;
	data_t   adv_data;
	data_t   dev_name;

	// Initialize advertisement report for parsing
#if NRF_SD_BLE_API >= 6
	adv_data.p_data = (uint8_t *)p_adv_report->data.p_data;
	adv_data.data_len = p_adv_report->data.len;
#else
	adv_data.p_data = (uint8_t *)p_adv_report->data;
	adv_data.data_len = p_adv_report->dlen;
#endif

	//search for advertising names
	err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
		&adv_data,
		&dev_name);
	if (err_code == NRF_SUCCESS)
	{
		if (memcmp(name_to_find, dev_name.p_data, dev_name.data_len) == 0)
		{
			return true;
		}
	}
	else
	{
		// Look for the short local name if it was not found as complete
		err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
			&adv_data,
			&dev_name);
		if (err_code != NRF_SUCCESS)
		{
			return false;
		}
		if (memcmp(name_to_find, dev_name.p_data, dev_name.data_len) == 0)
		{
			return true;
		}
	}
	return false;
}

/**@brief Function for initializing the BLE stack.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t ble_stack_init()
{
	uint32_t            err_code;
	uint32_t *          app_ram_base = NULL;

#if NRF_SD_BLE_API <= 3
	ble_enable_params_t ble_enable_params;
	memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#endif

#if NRF_SD_BLE_API == 3
	ble_enable_params.gatt_enable_params.att_mtu = GATT_MTU_SIZE_DEFAULT;
#elif NRF_SD_BLE_API < 3
	ble_enable_params.gatts_enable_params.attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
	ble_enable_params.gatts_enable_params.service_changed = false;
	ble_enable_params.common_enable_params.p_conn_bw_counts = NULL;
	ble_enable_params.common_enable_params.vs_uuid_count = 1;
#endif

#if NRF_SD_BLE_API <= 3
	ble_enable_params.gap_enable_params.periph_conn_count = 1;
	ble_enable_params.gap_enable_params.central_conn_count = 1;
	ble_enable_params.gap_enable_params.central_sec_count = 1;

	err_code = sd_ble_enable(m_adapter, &ble_enable_params, app_ram_base);
#else
	err_code = sd_ble_enable(m_adapter, app_ram_base);
#endif

	switch (err_code) {
	case NRF_SUCCESS:
		break;
	case NRF_ERROR_INVALID_STATE:
		printf("BLE stack already enabled\n");
		fflush(stdout);
		break;
	default:
		printf("Failed to enable BLE stack. Error code: %d\n", err_code);
		fflush(stdout);
		break;
	}

	return err_code;
}

#if NRF_SD_BLE_API < 5
/**@brief Set BLE option for the BLE role and connection bandwidth.
 *
 * @return NRF_SUCCESS on option set successfully, otherwise an error code.
 */
static uint32_t ble_options_set()
{
#if NRF_SD_BLE_API <= 3
	ble_opt_t        opt;
	ble_common_opt_t common_opt;

	common_opt.conn_bw.role = BLE_GAP_ROLE_CENTRAL;
	common_opt.conn_bw.conn_bw.conn_bw_rx = BLE_CONN_BW_HIGH;
	common_opt.conn_bw.conn_bw.conn_bw_tx = BLE_CONN_BW_HIGH;
	opt.common_opt = common_opt;

	return sd_ble_opt_set(m_adapter, BLE_COMMON_OPT_CONN_BW, &opt);
#else
	return NRF_ERROR_NOT_SUPPORTED;
#endif
}
#endif

/**@brief Start scanning (GAP Discovery procedure, Observer Procedure).
 * *
 * @return NRF_SUCCESS on successfully initiating scanning procedure, otherwise an error code.
 */
static uint32_t scan_start()
{
#if NRF_SD_BLE_API >= 6
	m_adv_report_buffer.p_data = mp_data;
	m_adv_report_buffer.len = sizeof(mp_data);
#endif

	uint32_t error_code = sd_ble_gap_scan_start(m_adapter, &m_scan_param
#if NRF_SD_BLE_API >= 6
		, &m_adv_report_buffer
#endif
	);

	if (error_code != NRF_SUCCESS)
	{
		printf("Scan start failed with error code: %d\n", error_code);
		fflush(stdout);
	}
	else
	{
#ifdef DEBUG
		printf("Scan started\n");
		fflush(stdout);
#endif // DEBUG
	}

	return error_code;
}

/**@brief Function called upon connecting to BLE peripheral.
 *
 * @details Initiates primary service discovery.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t service_discovery_start()
{
	uint32_t   err_code;
	uint16_t   start_handle = 0x01;
	ble_uuid_t srvc_uuid;

	printf("Discovering primary services\n");
	fflush(stdout);

	srvc_uuid.type = BLE_UUID_TYPE_BLE;
	srvc_uuid.uuid = BLE_UUID_HEART_RATE_SERVICE;

	// Initiate procedure to find the primary BLE_UUID_HEART_RATE_SERVICE.
	err_code = sd_ble_gattc_primary_services_discover(m_adapter,
		m_connection_handle, start_handle,
		&srvc_uuid);
	if (err_code != NRF_SUCCESS)
	{
		printf("Failed to initiate or continue a GATT Primary Service Discovery procedure\n");
		fflush(stdout);
	}

	return err_code;
}

/**@brief Function called upon discovering a BLE peripheral's primary service(s).
 *
 * @details Initiates service's (m_service) characteristic discovery.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t char_discovery_start()
{
	ble_gattc_handle_range_t handle_range;

	printf("Discovering characteristics\n");
	fflush(stdout);

	handle_range.start_handle = m_service_start_handle;
	handle_range.end_handle = m_service_end_handle;

	return sd_ble_gattc_characteristics_discover(m_adapter, m_connection_handle, &handle_range);
}

/**@brief Function called upon discovering service's characteristics.
 *
 * @details Initiates heart rate monitor (m_hrm_char_handle) characteristic's descriptor discovery.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t descr_discovery_start()
{
	ble_gattc_handle_range_t handle_range;

	printf("Discovering characteristic's descriptors\n");
	fflush(stdout);

	if (m_hrm_char_handle == 0)
	{
		printf("No heart rate measurement characteristic handle found\n");
		fflush(stdout);
		return NRF_ERROR_INVALID_STATE;
	}

	handle_range.start_handle = m_hrm_char_handle;
	handle_range.end_handle = m_service_end_handle;

	return sd_ble_gattc_descriptors_discover(m_adapter, m_connection_handle, &handle_range);
}

/**@brief Function that write's the HRM characteristic's CCCD.
 * *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t hrm_cccd_set(uint8_t value)
{
	ble_gattc_write_params_t write_params;
	uint8_t                  cccd_value[2] = { value, 0 };

	printf("Setting HRM CCCD\n");
	fflush(stdout);

	if (m_hrm_cccd_handle == 0)
	{
		printf("Error. No CCCD handle has been found\n");
		fflush(stdout);
		return NRF_ERROR_INVALID_STATE;
	}

	write_params.handle = m_hrm_cccd_handle;
	write_params.len = 2;
	write_params.p_value = cccd_value;
	write_params.write_op = BLE_GATT_OP_WRITE_REQ;
	write_params.offset = 0;

	return sd_ble_gattc_write(m_adapter, m_connection_handle, &write_params);
}

/*@brief	Indicates if we're in server mode or not
 *@return	true if in server mode, false otherwise
 */
static bool in_server_mode()
{
	return server_port > 0;
}

/*@brief	Cyclic executive for program local mode
 *@return	Program return code
 */
static uint32_t local_mode_executive() {
	for (;;)
	{
		uint32_t error_code;
		char c = (char)getchar();
		log(SD_RPC_LOG_DEBUG, "Got input: '%c'", c);

		if (c == 'q' || c == 'Q')
		{
			error_code = sd_rpc_close(m_adapter);

			if (error_code != NRF_SUCCESS)
			{
				log(SD_RPC_LOG_ERROR, "Failed to close nRF BLE Driver. Error code: 0x%02X", error_code);
				return error_code;
			}

			log(SD_RPC_LOG_DEBUG, "Closed");

			return NRF_SUCCESS;
		}
	}
}

/*@brief	Cyclic executive for program server mode
 *@return	Program return code
 */
static uint32_t server_mode_executive() {
	sockpp::socket_initializer sockInit;
	server = sockpp::tcp_acceptor(server_port);

	if (!server) {
		std::cout << "Error creating the server: " << server.last_error_str() << std::endl;
		return NRF_ERROR_INTERNAL;
	}

	std::cout << "Listening for connections on port " << server_port << "." << std::endl;

	server_exit.lock(); // Open the server - will exit when a client unlocks this

	while (!server_exit.try_lock()) {
		sockpp::inet_address peer;

		// Accept a new client connection
		sockpp::tcp_socket listening_socket = server.accept(&peer);

		if (!listening_socket)
		{
			log(SD_RPC_LOG_ERROR, "%s", server.last_error_str());
		}
		else 
		{
			// Handle client thread
			std::cout << "Connection request from " << peer << std::endl;
			std::thread thr(on_client, std::move(listening_socket));
			thr.detach();
		}
	}

	return NRF_SUCCESS;
}

/** Event functions */

static void on_client(sockpp::tcp_socket socket) {

	// Create a client, and transfer it into the queue.
	ScannerClient client(&socket);

	client_access_lock.lock();
	clients.push_back(&client);
	client_access_lock.unlock();

	while (socket.is_open())
	{
		ssize_t n;
		char buff[CLIENT_CMD_BUFFER] = { 0 };

		if ((n = socket.read(buff, sizeof(buff))) > 0) 
		{
			// Set final newline to a null (if exists) to allow netcat debugging
			for (int i = 0; i < CLIENT_CMD_BUFFER && buff[i] != '\0'; ++i) {
				if (buff[i] == '\n' || buff[i] == '\r') {
					buff[i] = '\0';
				}
			}

			log(SD_RPC_LOG_DEBUG, "CLIENT: %s", buff);

			// Parse commands
			if (strcmp(buff, "QUIT") == 0) 
			{
				log(SD_RPC_LOG_DEBUG, "SERVER: Servicing QUIT command");
				server_exit.unlock();
				server.close();
				break;
			}
			else if (strstr(buff, "FILTER_MAC ") == buff)
			{
				log(SD_RPC_LOG_DEBUG, "SERVER: Servicing FILTER_MAC command");
				client_access_lock.lock();
				client.macFilter = buff + strlen("FILTER_MAC ");
				client_access_lock.unlock();
				log(SD_RPC_LOG_DEBUG, ("SERVER: Set MAC filter to '" + client.macFilter + "'").c_str());
			}
			else if (strstr(buff, "FILTER_NAME ") == buff)
			{
				log(SD_RPC_LOG_DEBUG, "SERVER: Servicing FILTER_NAME command");
				client_access_lock.lock();
				client.nameFilter = buff + strlen("FILTER_NAME ");
				client_access_lock.unlock();
				log(SD_RPC_LOG_DEBUG, ("SERVER: Set name filter to '" + client.nameFilter + "'").c_str());
			}
			else if (strcmp(buff, "START") == 0)
			{
				log(SD_RPC_LOG_DEBUG, "SERVER: Servicing START command");
				client_access_lock.lock();
				client.send = true;
				client_access_lock.unlock();
			}
			else if (strcmp(buff, "STOP") == 0)
			{
				log(SD_RPC_LOG_DEBUG, "SERVER: Servicing STOP command");
				client_access_lock.lock();
				client.send = false;
				client_access_lock.unlock();
			}
		}
		else 
		{
			// Read error, exit the loop
			log(SD_RPC_LOG_ERROR, ("CLIENT: Read error " + socket.last_error_str()).c_str());
			break;
		}
	}

	client_access_lock.lock();
	clients.erase(std::remove(clients.begin(), clients.end(), &client), clients.end());
	client_access_lock.unlock();

	log(SD_RPC_LOG_DEBUG, "Client disconnected.");
}

/**@brief Function called on BLE_GAP_EVT_ADV_REPORT event.
 *
 * @details Print out all advertising devices to stdout
 *
 * @param[in] p_ble_gap_evt Advertising Report Event.
 */
static void on_adv_report(const ble_gap_evt_t * const p_ble_gap_evt)
{
	uint32_t err_code;
	uint8_t  addr[STRING_BUFFER_SIZE] = { 0 };
	data_t   adv_data = 
	{ 
		(uint8_t *)p_ble_gap_evt->params.adv_report.data, 
		p_ble_gap_evt->params.adv_report.dlen
	};

	data_t   dev_name;

	// Log the Bluetooth device address of advertisement packet received.
	ble_address_to_string_convert(p_ble_gap_evt->params.adv_report.peer_addr, addr);

	// Get the device name (if any)
	const uint8_t offset = 6;

	err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
		&adv_data,
		&dev_name);

	if (err_code != NRF_SUCCESS)
	{
		dev_name.p_data = (uint8_t *)("<unknown>");
		dev_name.data_len = 0;
	}

	// Find what to do based on the mode we're in

	if (in_server_mode()) // We're in server mode, post to clients if any
	{
		// Format our output for all clients
		char send[STRING_BUFFER_SIZE];
		sprintf((char *)&send, "%s,%s\n", addr, (char *)dev_name.p_data);

		client_access_lock.lock();
		for (uint32_t i = 0; i < clients.size(); ++i) {
			if (clients[i]->send) { // If send is enabled

				// Do filtering
				if (clients[i]->nameFilter != "" &&
					strstr((char *)dev_name.p_data, clients[i]->nameFilter.c_str()) == NULL) {
					continue;
				}
				if (clients[i]->macFilter != "" &&
					strstr((char *)addr, clients[i]->macFilter.c_str()) != (char *)addr) {
					continue;
				}

				// If we pass the filter, send
				clients[i]->socket->write(send);
			}
		}
		client_access_lock.unlock();
	}
	else // We're in local mode, just print
	{
		printf("%s,%s\n", addr, (char *)dev_name.p_data);
		fflush(stdout);
	}
}


/** Event dispatcher */

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] adapter The transport adapter.
 * @param[in] p_ble_evt Bluetooth stack event.
 */
static void ble_evt_dispatch(adapter_t * adapter, ble_evt_t * p_ble_evt)
{
	if (p_ble_evt == NULL)
	{
		log(SD_RPC_LOG_ERROR, "Received an empty BLE event");
		return;
	}

	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_ADV_REPORT:
		on_adv_report(&(p_ble_evt->evt.gap_evt));
		break;
	default:
		// No event handler
		break;
	}
}


/** Main */

/**@brief Function for application main entry.
 *
 * @param[in] argc Number of arguments (program expects 0 or 1 arguments).
 * @param[in] argv The serial port of the target nRF5 device (Optional).
 */
int main(int argc, char * argv[])
{
	uint32_t error_code;
	char serial_port[10] = DEFAULT_UART_PORT_NAME;
	uint32_t baud_rate = DEFAULT_BAUD_RATE;
	uint8_t  cccd_value = 0;

	/**	<Configure Options> */

	cxxopts::Options options("BLE Scanner", "Simple BLE scanning utility\nBy Jason Storey. (v" + std::to_string(VERSION_MAJOR) +  "." + std::to_string(VERSION_MINOR) +  ".0)\n");

	options.add_options()
		("h,help", "Show this help message")
		("v,version", "Output the version number")
		("d,debug", "Enable verbose debug output")
		("p,port",  "Serial port to use (e.g. COM1).", cxxopts::value<std::string>()->default_value(DEFAULT_UART_PORT_NAME))
		("s,server", "Start the scanner as a local server on the given port. E.g. -s 3000 will listen on 127.0.0.1:3000.", cxxopts::value<std::uint32_t>()->default_value("0"))
		;
	
	try
	{
		cxxopts::ParseResult result = options.parse(argc, argv);

		if (result["help"].as<bool>()) 
		{
			// Print option help
			std::cout << options.help();

			// Print server API commands
			std::cout << std::endl
				<< "Server API commands:" << std::endl
				<< "  START			- Start sending discovered BLE advertisements in real-time" << std::endl
				<< "  STOP			- Stop sending" << std::endl
				<< "  FILTER_NAME <str>	- Filter device names. Names filtered by if they contain <str>." << std::endl
				<< "			  To stop filtering, set <str> to empty (send 'FILTER_NAME ')" << std::endl
				<< "  FILTER_MAC <str>	- Filter device MAC. Filtered by if they start with <str>." << std::endl
				<< "			  To stop filtering, set <str> to empty (send 'FILTER_MAC ')" << std::endl
				<< "  QUIT			- Force the server to exit" << std::endl
				<< std::endl
				<< "Device names are provided in the format <MAC>,<name> where <MAC> is the device address" << std::endl
				<< "and <name> is either '<unknown>' or the device complete local name. " << std::endl
				<< std::endl;
			return NRF_SUCCESS;
		}

		if (result["version"].as<bool>())
		{
			std::cout << VERSION_MAJOR << '.' << VERSION_MINOR << ".0" << std::endl;
			return NRF_SUCCESS;
		}

		verbose_mode = result["debug"].as<bool>();
		server_port = result["server"].as<uint32_t>();
		strcpy(serial_port, result["port"].as<std::string>().c_str());
	}
	catch (cxxopts::OptionException e)
	{
		// Options error has occurred
		std::cout << "Argument error: " << e.what() << std::endl;
		return NRF_ERROR_INVALID_PARAM;
	}
	// FENCE --- result guaranteed not NULL 


	if (!std::regex_match(serial_port, std::regex("COM\\d+")))
	{
		std::cout << "Error: Unexpected serial port format";
		return NRF_ERROR_INVALID_PARAM;
	}

	/** </Configure Options> */

	log(SD_RPC_LOG_DEBUG, "Verbose mode: %d", verbose_mode);
	log(SD_RPC_LOG_DEBUG, "Server mode: %d", in_server_mode());
	log(SD_RPC_LOG_DEBUG, "Serial port used: %s", serial_port);
	log(SD_RPC_LOG_DEBUG, "Baud rate used: %d", baud_rate);

	m_adapter = adapter_init(serial_port, baud_rate);
	sd_rpc_log_handler_severity_filter_set(m_adapter, SD_RPC_LOG_INFO);
	error_code = sd_rpc_open(m_adapter, status_handler, ble_evt_dispatch, log_handler);

	if (error_code != NRF_SUCCESS)
	{
		log(SD_RPC_LOG_ERROR, "Failed to open BLE Driver. Error code: 0x%02X", error_code);
		return error_code;
	}

	error_code = ble_stack_init();
	CHECK_ERROR(error_code);

	error_code = ble_options_set();
	CHECK_ERROR(error_code);

	error_code = scan_start();
	CHECK_ERROR(error_code);

	// Perform mode as required
	if (in_server_mode())
	{
		return server_mode_executive();
	}
	else // Local mode, accept character control inputs
	{
		return local_mode_executive();
	}
}