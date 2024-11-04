#include <SPI.h>
#include <driver/adc.h>
#include <mcp2515.h>
#include "esp_adc_cal.h"

// Debugging flag
#define DEBUG true  // Set to 'true' to enable debugging outputs

// Constants
const uint16_t ADC_THRESHOLD = 2500;      // ADC value threshold
const uint8_t CAN_DLC = 0x08;             // CAN data length code
const uint16_t CAN_TIMEOUT = 100;         // CAN timeout in milliseconds
const int32_t SPEED_STEP = 500;           // Speed increment/decrement step

// CAN controller initialization
MCP2515 canController(10);  // CS pin set to 10

// ADC Calibration Characteristics
esp_adc_cal_characteristics_t adc_chars;

/**
 * @brief Structure to hold motor control information.
 */
struct MotorControl {
    int can_id;                // The CAN ID for the motor
    int control_pin;           // GPIO pin for forward direction control
    String motor_name;         // Name of the motor for debugging
    int32_t current_speed;     // Current speed
    int32_t max_speed;         // Maximum speed
    int32_t adc_threshold_zero;
    int32_t adc_threshold_lower;
    int32_t adc_threshold_higher;
    int32_t adc_threshold_max;
};

struct can_frame canMsg;  // Global CAN message frame

// Define the motors and their control pins
MotorControl motors[] = {
    // CAN ID, Control Pin, Name, Current Speed, Thresholds
    // {330, 1, "T Shldr", 1000, 10000,  0,    683,  1365, 2048},  // Top shoulder motor
    // {321, 1, "B Shldr", 1000, 10000, 2048, 2731, 3414, 4095},  // Mid shoulder motor
    {321, 19, "B Shldr", 1000, 10000, 0,    1365, 2730, 4095},  // Mid shoulder motor
    {325, 20, "Elbow", 15000, 15000, 0,    1365, 2730, 4095}   // Elbow motor
};

const int NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

/**
 * @brief Sends a speed command to the motor.
 *
 * @param motor: Reference to the MotorControl structure for the motor.
 * @param speed: Speed value as a 32-bit signed integer.
 */
void sendMotorSpeed(MotorControl& motor, int32_t speed) {
    struct can_frame canMsgOut;
    canMsgOut.can_id = motor.can_id;
    canMsgOut.can_dlc = CAN_DLC;

    canMsgOut.data[0] = 0xA2; // Command byte for speed control
    canMsgOut.data[1] = 0x00;
    canMsgOut.data[2] = 0x00;
    canMsgOut.data[3] = 0x00;

    // Split the 32-bit signed speed into bytes (little-endian)
    canMsgOut.data[4] = (uint8_t)(speed & 0xFF);         // Byte 0 (LSB)
    canMsgOut.data[5] = (uint8_t)((speed >> 8) & 0xFF);  // Byte 1
    canMsgOut.data[6] = (uint8_t)((speed >> 16) & 0xFF); // Byte 2
    canMsgOut.data[7] = (uint8_t)((speed >> 24) & 0xFF); // Byte 3 (MSB)

    // Send the CAN message
    canController.sendMessage(&canMsgOut);
}

/**
 * @brief Reads and calibrates the ADC value with averaging.
 *
 * @param channel: The ADC channel to read from.
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
 * @brief Processes the motor control logic for a single motor.
 *
 * @param motor: Reference to the MotorControl structure for the motor.
 * @param debugOutput: Reference to the string for accumulating debug output.
 */
void processMotor(MotorControl& motor, String& debugOutput) {
    // Read calibrated ADC values from the control pins
    int adc_input = 0;
    if (motor.motor_name == "Elbow") {
        adc_input = readCalibratedADC(ADC1_CHANNEL_3);
    } else {
        adc_input = readCalibratedADC(ADC1_CHANNEL_1);
    }

    // Append to debugging output
    if (DEBUG) {
        debugOutput += "MT: " + motor.motor_name;
        debugOutput += ", AI: " + String(adc_input);
    }

    // Motor control logic based on ADC input thresholds
    if (adc_input >= motor.adc_threshold_zero && adc_input < motor.adc_threshold_lower) {
        // Stop
        motor.current_speed = 0;
    } else if (adc_input >= motor.adc_threshold_lower && adc_input < motor.adc_threshold_higher) {
        // Up
        motor.current_speed = motor.max_speed;
    } else if (adc_input >= motor.adc_threshold_higher && adc_input < motor.adc_threshold_max) {
        // Down
        motor.current_speed = -(motor.max_speed);
    }

    // Send the speed command to the motor
    sendMotorSpeed(motor, motor.current_speed);

    if (DEBUG) {
        debugOutput += ", Speed: " + String(motor.current_speed);
    }

    debugOutput += " | ";
}

void setup() {
    Serial.begin(115200);

    // Initialize CAN communication
    canController.reset();
    canController.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    canController.setNormalMode();

    // Configure ADC width and attenuation
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);

    // Initialize ADC calibration
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
}

void loop() {
    String debugOutput = "";

    // Process each motor
    for (int i = 0; i < NUM_MOTORS; i++) {
        processMotor(motors[i], debugOutput);
    }

    // Output debugging information
    if (DEBUG) {
        Serial.println(debugOutput);
    }

    // Add a delay between cycles if needed
    delay(100);
}
