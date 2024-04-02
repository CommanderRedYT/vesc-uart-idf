#include "VescUart.h"

constexpr const char * const TAG = "VescUart";

// system includes
#include <cstdint>

// esp-idf includes
#include <esp_log.h>

// 3rdparty lib includes
#include <espchrono.h>

VescUart::VescUart(espchrono::milliseconds32 timeout_ms) : m_timeout(timeout_ms)
{}

void VescUart::setSerialPort(const vesc_uart_config_t& cfg)
{
    serialConfig = std::nullopt;

    if (const auto res = uart_driver_install(cfg.uart_num, 256, 0, 0, nullptr, 0); res != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(res));
        return;
    }

    if (const auto res = uart_param_config(cfg.uart_num, &cfg.config); res != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(res));
        return;
    }

    if (const auto res = uart_set_pin(cfg.uart_num, cfg.tx_pin, cfg.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); res != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(res));
        return;
    }

    serialConfig = cfg;
}

int VescUart::receiveUartMessage(uint8_t* payloadReceived)
{

	// Messages <= 255 starts with "2", 2nd byte is length
	// Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

	// Makes no sense to run this function if no serialPort is defined.
	if (!serialConfig)
		return -1;

    const auto start = espchrono::millis_clock::now();

	uint16_t counter{0};
	uint16_t endMessage{256};
	bool messageRead{false};
	uint8_t messageReceived[256];
	uint16_t lenPayload{0};
    bool exit{false};

    auto read = [&]() -> int {
        uint8_t c = 0;
        if (uart_read_bytes(serialConfig->uart_num, &c, 1, 0) == 1)
        {
            return c;
        }
        else
        {
            return -1;
        }
    };

	while (espchrono::ago(start) < m_timeout && !messageRead && !exit)
    {
        size_t length{};

        while (!exit)
        {
            if (const auto result = uart_get_buffered_data_len(serialConfig->uart_num, &length); result != ESP_OK)
            {
                ESP_LOGW(TAG, "uart_get_buffered_data_len() failed with %s", esp_err_to_name(result));
                break;
            }

            const auto c = read();

            // ESP_LOGI(TAG, "c=%d", c);
            messageReceived[counter++] = c;

            ESP_LOGI(TAG, "counter=%d messageReceived[%d]=%d messageReceived=%s", counter, counter - 1, messageReceived[counter - 1], serialPrint(messageReceived, counter).c_str());

            if (counter == 2)
            {
                switch (messageReceived[1])
                {
                    case 2:
                        endMessage = messageReceived[2] + 5;
                        lenPayload = messageReceived[2];
                        break;
                    case 3:
                        ESP_LOGW(TAG, "Message is larger than 256 bytes - not supported");
                        break;
                    default:
                        ESP_LOGW(TAG, "Unknown message type: %d", messageReceived[1]);
                        break;
                }
            }

            if (counter >= sizeof(messageReceived))
            {
                ESP_LOGW(TAG, "Message is too large");
                exit = true;
                break;
            }

            ESP_LOGI(TAG, "counter=%d endMessage=%d messageReceived[endMessage - 1]=%d", counter, endMessage, messageReceived[endMessage - 1]);

            if (counter == endMessage && messageReceived[endMessage - 1] == 3)
            {
                messageReceived[endMessage] = 0;
                ESP_LOGI(TAG, "Message received: %s", serialPrint(messageReceived, endMessage).c_str());
                messageRead = true;
                break;
            }
        }
	}

	if (!messageRead)
    {
		ESP_LOGE(TAG, "Timeout reached while waiting for message");
	}
	
	bool unpacked{false};

	if (messageRead)
    {
		unpacked = unpackPayload(messageReceived, endMessage, payloadReceived);
	}

	if (unpacked)
    {
		// Message was read
		return lenPayload; 
	}
	else
    {
		// No Message Read
		return 0;
	}
}


bool VescUart::unpackPayload(uint8_t* message, int lenMes, uint8_t* payload)
{

	uint16_t crcMessage;
	uint16_t crcPayload;

	// Rebuild crc:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];

    ESP_LOGI(TAG, "CRC received: %d", crcMessage);

	// Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);

    ESP_LOGI(TAG, "CRC calculated: %d", crcPayload);
	
	if (crcPayload == crcMessage)
    {
        ESP_LOGI(TAG, "message=%s payload=%s", message, payload);

		return true;
	}

    return false;
}


int VescUart::packSendPayload(uint8_t* payload, int lenPay)
{
	uint16_t crcPayload = crc16(payload, lenPay);

	int count = 0;
	uint8_t messageSend[256];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}

	memcpy(messageSend + count, payload, lenPay);
	count += lenPay;

	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	// messageSend[count] = NULL;

    ESP_LOGI(TAG, "Package to send: %s", serialPrint(messageSend, count).c_str());

	// Sending package
	if (serialConfig)
    {
        // serialConfig->write(messageSend, count);
        if (const auto writtenBytes = uart_write_bytes(serialConfig->uart_num, messageSend, count); writtenBytes < 0)
        {
            ESP_LOGE(TAG, "uart_write_bytes failed: %s", esp_err_to_name(writtenBytes));
            return -1;
        }
        else
        {
            ESP_LOGI(TAG, "successfully sent %d bytes", writtenBytes);
        }
    }

	// Returns number of send bytes
	return count;
}


bool VescUart::processReadPacket(uint8_t* message)
{

	COMM_PACKET_ID packetId;
	int32_t index = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; // Removes the packetId from the actual message (payload)

	switch (packetId)
    {
    case COMM_FW_VERSION: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

        fw_version.major = message[index++];
        fw_version.minor = message[index++];
        return true;
    case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

        data.tempMosfet 		= buffer_get_float16(message, 10.0, &index); 	// 2 bytes - mc_interface_temp_fet_filtered()
        data.tempMotor 			= buffer_get_float16(message, 10.0, &index); 	// 2 bytes - mc_interface_temp_motor_filtered()
        data.avgMotorCurrent 	= buffer_get_float32(message, 100.0, &index); // 4 bytes - mc_interface_read_reset_avg_motor_current()
        data.avgInputCurrent 	= buffer_get_float32(message, 100.0, &index); // 4 bytes - mc_interface_read_reset_avg_input_current()
        index += 4; // Skip 4 bytes - mc_interface_read_reset_avg_id()
        index += 4; // Skip 4 bytes - mc_interface_read_reset_avg_iq()
        data.dutyCycleNow 		= buffer_get_float16(message, 1000.0, &index); 	// 2 bytes - mc_interface_get_duty_cycle_now()
        data.rpm 				= buffer_get_float32(message, 1.0, &index);		// 4 bytes - mc_interface_get_rpm()
        data.inpVoltage 		= buffer_get_float16(message, 10.0, &index);		// 2 bytes - GET_INPUT_VOLTAGE()
        data.ampHours 			= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_amp_hours(false)
        data.ampHoursCharged 	= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_amp_hours_charged(false)
        data.wattHours			= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_watt_hours(false)
        data.wattHoursCharged	= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_watt_hours_charged(false)
        data.tachometer 		= buffer_get_int32(message, &index);				// 4 bytes - mc_interface_get_tachometer_value(false)
        data.tachometerAbs 		= buffer_get_int32(message, &index);				// 4 bytes - mc_interface_get_tachometer_abs_value(false)
        data.error 				= (mc_fault_code)message[index++];								// 1 byte  - mc_interface_get_fault()
        data.pidPos				= buffer_get_float32(message, 1000000.0, &index);	// 4 bytes - mc_interface_get_pid_pos_now()
        data.id					= message[index++];								// 1 byte  - app_get_configuration()->controller_id

        return true;

    /* case COMM_GET_VALUES_SELECTIVE:

        uint32_t mask = 0xFFFFFFFF; */

    default:
        return false;
	}
}

bool VescUart::getFWversion()
{
	return getFWversion(0);
}

bool VescUart::getFWversion(uint8_t canId)
{
	int32_t index = 0;

	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	
	if (canId != 0)
    {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}

	payload[index++] = { COMM_FW_VERSION };

	packSendPayload(payload, payloadSize);

	uint8_t message[256];
	int messageLength = receiveUartMessage(message);

    if (messageLength > 0)
    {
		return processReadPacket(message); 
	}

	return false;
}

bool VescUart::getVescValues()
{
	return getVescValues(0);
}

bool VescUart::getVescValues(uint8_t canId)
{
    ESP_LOGI(TAG, "Command: COMM_GET_VALUES %d", canId);

	int32_t index = 0;

	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];

	if (canId != 0)
    {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}

	payload[index++] = { COMM_GET_VALUES };

	if (const auto sent = packSendPayload(payload, payloadSize); sent < 0)
    {
        ESP_LOGE(TAG, "Failed to send payload");
        return false;
    }

	uint8_t message[256];
	int messageLength = receiveUartMessage(message);

	if (messageLength > 55)
    {
		return processReadPacket(message); 
	}

	return false;
}

void VescUart::setCurrent(float current)
{
	return setCurrent(current, 0);
}

void VescUart::setCurrent(float current, uint8_t canId)
{
	int32_t index = 0;

	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];

    if (canId != 0)
    {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}

	payload[index++] = { COMM_SET_CURRENT };
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	packSendPayload(payload, payloadSize);
}

void VescUart::setBrakeCurrent(float brakeCurrent)
{
	return setBrakeCurrent(brakeCurrent, 0);
}

void VescUart::setBrakeCurrent(float brakeCurrent, uint8_t canId)
{
	int32_t index = 0;

	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];

	if (canId != 0)
    {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}

	payload[index++] = { COMM_SET_CURRENT_BRAKE };
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);

	packSendPayload(payload, payloadSize);
}

void VescUart::setRPM(float rpm)
{
	return setRPM(rpm, 0);
}

void VescUart::setRPM(float rpm, uint8_t canId)
{
	int32_t index = 0;

	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];

	if (canId != 0)
    {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}

	payload[index++] = { COMM_SET_RPM };
	buffer_append_int32(payload, (int32_t)(rpm), &index);
	packSendPayload(payload, payloadSize);
}

void VescUart::setDuty(float duty)
{
	return setDuty(duty, 0);
}

void VescUart::setDuty(float duty, uint8_t canId)
{
	int32_t index = 0;

	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];

	if (canId != 0)
    {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}

	payload[index++] = { COMM_SET_DUTY };
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

	packSendPayload(payload, payloadSize);
}

void VescUart::sendKeepalive()
{
	return sendKeepalive(0);
}

void VescUart::sendKeepalive(uint8_t canId)
{
	int32_t index = 0;

	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];

	if (canId != 0)
    {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}

	payload[index++] = { COMM_ALIVE };
	packSendPayload(payload, payloadSize);
}

std::string VescUart::serialPrint(uint8_t* data, int len)
{
    std::string str;

    for (int i = 0; i <= len; i++)
    {
        str += std::to_string(data[i]);
        str += " ";
    }

    return str;
}

void VescUart::printVescValues() const
{
    ESP_LOGI(
            TAG,
            "avgMotorCurrent=%f"
            "avgInputCurrent=%f"
            "dutyCycleNow=%f"
            "rpm=%f"
            "inputVoltage=%f"
            "ampHours=%f"
            "ampHoursCharged=%f"
            "wattHours=%f"
            "wattHoursCharged=%f"
            "tachometer=%ld"
            "tachometerAbs=%ld"
            "tempMosfet=%f"
            "tempMotor=%f"
            "error=%d",
            data.avgMotorCurrent,
            data.avgInputCurrent,
            data.dutyCycleNow,
            data.rpm,
            data.inpVoltage,
            data.ampHours,
            data.ampHoursCharged,
            data.wattHours,
            data.wattHoursCharged,
            data.tachometer,
            data.tachometerAbs,
            data.tempMosfet,
            data.tempMotor,
            data.error
    );
}
