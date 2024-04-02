#ifndef VESCUART_h
#define VESCUART_h

// #include <Arduino.h> // no, bad boi!

// system includes
#include <optional>

// esp-idf includes
#include <driver/uart.h>

// 3rdparty lib includes
#include <espchrono.h>

// local includes
#include "datatypes.h"
#include "buffer.h"
#include "crc.h"

using namespace std::chrono_literals;

class VescUart
{

	//Timeout - specifies how long the function will wait for the vesc to respond
	const espchrono::milliseconds32 m_timeout;

	public:
		/**
		 * @brief      Class constructor
		 */
		explicit VescUart(espchrono::milliseconds32 timeout_ms = 100ms);

		/** Variabel to hold measurements returned from VESC */
		dataPackage data{};

       /** Variable to hold firmware version */
        FWversionPackage fw_version{};

        /**
         * @brief      Set the serial port for uart communication
         * @param      cfg  - Reference to Serial port (pointer)
         */
        void setSerialPort(const vesc_uart_config_t& cfg);

        /**
         * @brief      Populate the firmware version variables
         *
         * @return     True if successfull otherwise false
         */
        bool getFWversion();

        /**
         * @brief      Populate the firmware version variables
         *
         * @param      canId  - The CAN ID of the VESC
         * @return     True if successfull otherwise false
         */
        bool getFWversion(uint8_t canId);

        /**
         * @brief      Sends a command to VESC and stores the returned data
         *
         * @return     True if successfull otherwise false
         */
        bool getVescValues();

        /**
         * @brief      Sends a command to VESC and stores the returned data
         * @param      canId  - The CAN ID of the VESC
         *
         * @return     True if successfull otherwise false
         */
        bool getVescValues(uint8_t canId);

        /**
         * @brief      Set the current to drive the motor
         * @param      current  - The current to apply
         */
        void setCurrent(float current);

        /**
         * @brief      Set the current to drive the motor
         * @param      current  - The current to apply
         * @param      canId  - The CAN ID of the VESC
         */
        void setCurrent(float current, uint8_t canId);

        /**
         * @brief      Set the current to brake the motor
         * @param      brakeCurrent  - The current to apply
         */
        void setBrakeCurrent(float brakeCurrent);

        /**
         * @brief      Set the current to brake the motor
         * @param      brakeCurrent  - The current to apply
         * @param      canId  - The CAN ID of the VESC
         */
        void setBrakeCurrent(float brakeCurrent, uint8_t canId);


        /**
         * @brief      Set the rpm of the motor
         * @param      rpm  - The desired RPM (actually eRPM = RPM * poles)
         */
        void setRPM(float rpm);		

        /**
         * @brief      Set the rpm of the motor
         * @param      rpm  - The desired RPM (actually eRPM = RPM * poles)
         * @param      canId  - The CAN ID of the VESC
         */
        void setRPM(float rpm, uint8_t canId);

        /**
         * @brief      Set the duty of the motor
         * @param      duty  - The desired duty (0.0-1.0)
         */
        void setDuty(float duty);

        /**
         * @brief      Set the duty of the motor
         * @param      duty  - The desired duty (0.0-1.0)
         * @param      canId  - The CAN ID of the VESC
         */
        void setDuty(float duty, uint8_t canId);

        /**
         * @brief      Send a keepalive message
         */
        void sendKeepalive();

        /**
         * @brief      Send a keepalive message
         * @param      canId  - The CAN ID of the VESC
         */
        void sendKeepalive(uint8_t canId);

        /**
         * @brief      Help Function to print struct dataPackage over Serial for Debug
         */
        void printVescValues() const;

	private: 

		/** Variabel to hold the reference to the Serial object to use for UART */
        std::optional<vesc_uart_config_t> serialConfig{std::nullopt};

		/**
		 * @brief      Packs the payload and sends it over Serial
		 *
		 * @param      payload  - The payload as a unit8_t Array with length of int lenPayload
		 * @param      lenPay   - Length of payload
		 * @return     The number of bytes send
		 */
		int packSendPayload(uint8_t* payload, int lenPay);

		/**
		 * @brief      Receives the message over Serial
		 *
		 * @param      payloadReceived  - The received payload as a unit8_t Array
		 * @return     The number of bytes receeived within the payload
		 */
		int receiveUartMessage(uint8_t* payloadReceived);

		/**
		 * @brief      Verifies the message (CRC-16) and extracts the payload
		 *
		 * @param      message  - The received UART message
		 * @param      lenMes   - The lenght of the message
		 * @param      payload  - The final payload ready to extract data from
		 * @return     True if the process was a success
		 */
		static bool unpackPayload(uint8_t* message, int lenMes, uint8_t* payload);

		/**
		 * @brief      Extracts the data from the received payload
		 *
		 * @param      message  - The payload to extract data from
		 * @return     True if the process was a success
		 */
		bool processReadPacket(uint8_t* message);

		/**
		 * @brief      Help Function to print uint8_t array over Serial for Debug
		 *
		 * @param      data  - Data array to print
		 * @param      len   - Lenght of the array to print
		 */
		static std::string serialPrint(uint8_t* data, int len);
};

#endif // VESCUART_h
