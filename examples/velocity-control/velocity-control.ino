#include <SPI.h>
#include <driver/adc.h>
#include <mcp2515.h>
#include "esp_adc_cal.h"

// Debugging flag
#define DEBUG true  // Set to 'true' to enable debugging outputs

// Constants
const uint8_t CAN_DLC        = 0x08;      // CAN data length code
const uint16_t CAN_TIMEOUT   = 100;       // CAN timeout in milliseconds

// CAN controller initialization
MCP2515 canController(10);  // CS pin set to 10

// ADC Calibration Characteristics
esp_adc_cal_characteristics_t adc_chars;

/**
 * @brief Structure to hold motor control information.
 */
struct MotorControl {
    int      can_id;                // The CAN ID for the motor
    int      control_pin;           // GPIO pin for direction control
    String   motor_name;            // Name of the motor for debugging
    int32_t  current_speed;         // Current speed
    int32_t  max_speed;             // Maximum speed
    int32_t  adc_threshold_zero;
    int32_t  adc_threshold_lower;
    int32_t  adc_threshold_higher;
    int32_t  adc_threshold_max;

    // Last known motor data (updated after each read)
    int8_t   last_temperature;
    int16_t  last_torque;
    int16_t  last_speed;
    uint16_t last_position;

    // Pins to stream data out
    int tempPin;
    int torquePin;
    int speedPin;
    int positionPin;
};

/**
 * @brief Extractors for motor data from the 8-byte CAN data array
 */
int8_t extractMotorTemperatureFromCAN(const uint8_t data[8]) {
    // Motor temperature (int8_t, 1°C/LSB) at DATA[1]
    return static_cast<int8_t>(data[1]);
}

int16_t extractMotorTorqueFromCAN(const uint8_t data[8]) {
    // Combine DATA[2] and DATA[3] into torque data
    // Range:-2048~2048, real torque current range:-33A~33A
    int16_t torque = (data[3] << 8) | data[2];
    return torque;
}

int16_t extractMotorSpeedFromCAN(const uint8_t data[8]) {
    // Combine DATA[4] and DATA[5] into speed data (1dps/LSB)
    int16_t speed = (data[5] << 8) | data[4];
    return speed;
}

uint16_t extractMotorPositionFromCAN(const uint8_t data[8]) {
    // Combine DATA[6] and DATA[7] into position data
    // 14-bit encoder range: 0~16383
    uint16_t position = (data[7] << 8) | data[6];
    return position;
}

/**
 * @brief Sends a speed command to the motor via CAN bus.
 *
 * @param motor: Reference to the MotorControl structure for the motor.
 * @param speed: Speed value as a 32-bit signed integer.
 */
void sendMotorSpeed(MotorControl& motor, int32_t speed) {
    struct can_frame canMsgOut;
    canMsgOut.can_id  = motor.can_id;
    canMsgOut.can_dlc = CAN_DLC;

    // According to the datasheet, the host sends 0xA2 for speed control.
    // Speed is int32_t in 0.01dps/LSB
    canMsgOut.data[0] = 0xA2; // Command byte
    canMsgOut.data[1] = 0x00;
    canMsgOut.data[2] = 0x00;
    canMsgOut.data[3] = 0x00;

    // Split the 32-bit signed speed into bytes (little-endian)
    canMsgOut.data[4] = (uint8_t)(speed & 0xFF);        
    canMsgOut.data[5] = (uint8_t)((speed >> 8) & 0xFF); 
    canMsgOut.data[6] = (uint8_t)((speed >> 16) & 0xFF);
    canMsgOut.data[7] = (uint8_t)((speed >> 24) & 0xFF);

    // Send the CAN message
    canController.sendMessage(&canMsgOut);
}

/**
 * @brief Low-level function to output a given @p data value
 *        bit-by-bit on @p pin, LSB-first, with a small delay between bits.
 *
 * @param data  The unsigned data to send (0..(2^bits - 1)).
 * @param bits  Number of bits to shift out (8 for temperature, 16 for torque, speed, position).
 * @param pin   GPIO pin used for this data output.
 */
void streamDataToPin(uint32_t data, int bits, int pin)
{
    // Start bits
    for (int i = 0; i < 8; i++) {
        digitalWrite(pin, i < 5 ? LOW : HIGH);
        // Small pause so external hardware (NI USB-6009) can sample the state
        delayMicroseconds(100); 
    }
    // Data
    for (int i = 0; i < bits; i++) {
        // Extract bit i
        bool bitVal = (data >> i) & 0x01;
        digitalWrite(pin, bitVal ? HIGH : LOW);
        // Small pause so external hardware (NI USB-6009) can sample the state
        delayMicroseconds(100); 
    }
}

/**
 * @brief Reads the motor response via CAN bus (blocking up to CAN_TIMEOUT).
 *        If a message for this motor is received, updates motor.last_xxx fields
 *        and streams them out on dedicated pins.
 *
 * @param motor: Reference to the MotorControl structure for the motor.
 * @return bool: True if data was read successfully, false on timeout or other error.
 */
bool readMotorData(MotorControl& motor) {
    unsigned long startTime = millis();
    struct can_frame canMsgIn;

    // Try to read a CAN message until timeout
    while (canController.readMessage(&canMsgIn) != MCP2515::ERROR_OK) {
        if (millis() - startTime >= CAN_TIMEOUT) {
            // Timed out waiting for a CAN message
            return false; 
        }
    }

    // Check if the message is from the desired motor
    if (canMsgIn.can_id != motor.can_id) {
        return false; // Not our motor's response
    }

    // Update motor data
    motor.last_temperature = extractMotorTemperatureFromCAN(canMsgIn.data);
    motor.last_torque      = extractMotorTorqueFromCAN(canMsgIn.data);
    motor.last_speed       = extractMotorSpeedFromCAN(canMsgIn.data);
    motor.last_position    = extractMotorPositionFromCAN(canMsgIn.data);

    // Now stream them out on the corresponding pins (one pin per data type).
    // 1) Temperature is 8 bits
    //streamDataToPin((uint8_t)motor.last_temperature, 8, motor.tempPin);

    // 2) Torque is 16 bits
    //streamDataToPin((uint16_t)motor.last_torque, 16, motor.torquePin);

    // 3) Speed is 16 bits
    streamDataToPin((uint16_t)motor.last_speed, 16, motor.speedPin);

    // 4) Position is 16 bits
    //streamDataToPin((uint16_t)motor.last_position, 16, motor.positionPin);

    return true;
}

/**
 * @brief Reads and calibrates the ADC value with averaging.
 *
 * @param channel: The ADC1 channel to read from.
 * @return Calibrated voltage value in millivolts.
 */
int readCalibratedADC(adc1_channel_t channel) {
    uint32_t adc_reading = 0;
    const uint8_t samples = 10;

    // Average multiple samples
    for (uint8_t i = 0; i < samples; i++) {
        adc_reading += adc1_get_raw(channel);
    }
    adc_reading /= samples;

    // Convert ADC reading to voltage in millivolts
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
    return voltage;
}

/**
 * @brief Processes motor logic: reads ADC for user input, decides speed,
 *        sends speed command, then reads & streams motor data.
 */
void processMotor(MotorControl& motor, String& debugOutput) {
    // Read a specific ADC channel for each motor
    // Just an example: "Elbow" from ADC1_CHANNEL_3, others from ADC1_CHANNEL_1
    int adc_input = 0;
    if (motor.motor_name == "Elbow") {
        adc_input = readCalibratedADC(ADC1_CHANNEL_3);
    } else {
        adc_input = readCalibratedADC(ADC1_CHANNEL_1);
    }

    // Append to debugging output
    if (DEBUG) {
        debugOutput += "[Motor: " + motor.motor_name + "] ADC=" + String(adc_input);
    }

    // Decide motor speed based on threshold ranges
    if (adc_input >= motor.adc_threshold_zero && adc_input < motor.adc_threshold_lower) {
        // Stop
        motor.current_speed = 0;
    }
    else if (adc_input >= motor.adc_threshold_lower && adc_input < motor.adc_threshold_higher) {
        // Up
        motor.current_speed = motor.max_speed;
    }
    else if (adc_input >= motor.adc_threshold_higher && adc_input < motor.adc_threshold_max) {
        // Down
        motor.current_speed = -(motor.max_speed);
    }

    // Send the speed command to the motor
    sendMotorSpeed(motor, motor.current_speed);

    // Read back motor data (and stream it to pins)
    bool success = readMotorData(motor);

    // For debugging
    if (DEBUG) {
        debugOutput += ", SpeedCmd=" + String(motor.current_speed);
        debugOutput += (success ? " [Data OK]" : " [No Data]");
        debugOutput += " | ";
    }
}

// ------------------------------------------------------------------
//  MOTORS ARE DEFINED HERE
// ------------------------------------------------------------------
MotorControl motors[] = {
  // can_id, ctrl_pin,  Name,    current_spd, max_spd,   t0,  tLower, tHigher, tMax,
  // last-temp, last-torque, last-speed, last-pos,
  // tempPin, torquePin, speedPin, positionPin
  {
    321,  // can_id
    19,   // control_pin
    "B Shldr",
    1000,  // current_speed (initial)
    10000, // max_speed
    0, 1365, 2730, 4095, // motion thresholds
    0, 0, 0, 0,          // last-known data
    // Now map data outputs:
    7, // tempPin
    6, // torquePin
    5, // speedPin
    2  // positionPin
  },
  {
    325, // can_id
    20,  // control_pin
    "Elbow",
    15000,  // current_speed (initial)
    15000,  // max_speed
    0, 1365, 2730, 4095, // motion thresholds
    0, 0, 0, 0,          // last-known data
    8, // tempPin
    16, // torquePin
    17, // speedPin
    18  // positionPin
  }
};

const int NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

// ------------------------------------------------------------------

void setup() {
    Serial.begin(115200);

    // Initialize CAN communication
    canController.reset();
    canController.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    canController.setNormalMode();

    // Configure ADC width and attenuation
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11); // GPIO2
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11); // GPIO4

    // Initialize ADC calibration
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);

    // ------------------------------------------------------------------
    //  Configure the data output pins as outputs
    // ------------------------------------------------------------------
    for (int i = 0; i < NUM_MOTORS; i++) {
        pinMode(motors[i].tempPin,    OUTPUT);
        pinMode(motors[i].torquePin,  OUTPUT);
        pinMode(motors[i].speedPin,   OUTPUT);
        pinMode(motors[i].positionPin,OUTPUT);
    }
}

void loop() {
    String debugOutput = "";

    // Process each motor in turn
    for (int i = 0; i < NUM_MOTORS; i++) {
        processMotor(motors[i], debugOutput);
    }

    if (DEBUG) {
        Serial.println(debugOutput);
    }

    delay(100);
}
