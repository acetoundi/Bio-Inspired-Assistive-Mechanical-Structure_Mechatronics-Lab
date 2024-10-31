#include <SPI.h>
#include <mcp2515.h>

// Debugging flag
#define DEBUG false  // Set to 'true' to enable debugging outputs

// Constants
const uint16_t ADC_THRESHOLD = 2500;      // ADC value threshold
const uint8_t CAN_DLC = 0x08;             // CAN data length code
const uint16_t CAN_TIMEOUT = 100;         // CAN timeout in milliseconds
const int32_t SPEED_STEP = 500;           // Speed increment/decrement step

// CAN controller initialization
MCP2515 canController(10);  // CS pin set to 10

/**
 * @brief Structure to hold motor control information.
 */
struct MotorControl {
    int can_id;                // The CAN ID for the motor
    int control_pin_forward;   // GPIO pin for forward direction control
    int control_pin_backward;  // GPIO pin for backward direction control
    String motor_name;         // Name of the motor for debugging
    int32_t current_speed;     // Current speed of the motor
    int32_t speed_min;         // Minimum speed for ramping
    int32_t speed_max;         // Maximum speed for ramping
    int analog_output_pin;     // Pin for analogWrite to output torque
};

struct can_frame canMsg;  // Global CAN message frame

// Define the motors and their control pins
MotorControl motors[] = {
    // CAN ID, Forward Pin, Backward Pin, Name, Current Speed, Speed Min, Speed Max, Analog Output Pin
    {330, 20, 19, "T Shldr", 0, 1000, 5000, 2},  // Top shoulder motor
    {321, 18, 17, "B Shldr", 0, 1000, 5000, 4},  // Mid shoulder motor
    {325, 16, 1,  "Elbow",   0, 15000, 20000, 5} // Elbow motor
};

const int NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

/**
 * @brief Extracts the motor torque current from the CAN data bytes.
 *
 * @param data: Array of 8 CAN data bytes.
 * @return float: The torque current in amperes.
 */
float extractMotorTorqueFromCAN(const uint8_t data[8]) {
    // Combine DATA[2] and DATA[3] into a signed 16-bit integer (little-endian)
    int16_t iq = (int16_t)((uint16_t)data[3] << 8 | data[2]);
    // Scale the value down by 100 times to get amperes
    float torqueCurrent = iq * 0.01f;
    return torqueCurrent;
}

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
    canMsgOut.data[1] = 0x00; // Reserved or motor temperature (set to 0)
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
 * @brief Processes the motor control logic for a single motor.
 *
 * @param motor: Reference to the MotorControl structure for the motor.
 * @param debugOutput: Reference to the string for accumulating debug output.
 */
void processMotor(MotorControl& motor, String& debugOutput) {
    // Read ADC values from the control pins
    int adc_forward = analogRead(motor.control_pin_forward);
    int adc_backward = analogRead(motor.control_pin_backward);

    // Append to debugging output
    if (DEBUG) {
        debugOutput += "Mt: " + motor.motor_name;
        debugOutput += ", In1: " + String(adc_forward);
        debugOutput += ", In2: " + String(adc_backward);
    }

    // Decide on movement
    bool move_forward = false;
    bool move_backward = false;

    if (adc_forward > ADC_THRESHOLD && adc_backward <= ADC_THRESHOLD) {
        move_forward = true;
    } else if (adc_backward > ADC_THRESHOLD && adc_forward <= ADC_THRESHOLD) {
        move_backward = true;
    } else if (adc_forward <= ADC_THRESHOLD && adc_backward <= ADC_THRESHOLD) {
        // Stop motor and reset current speed
        motor.current_speed = 0;
        sendMotorSpeed(motor, motor.current_speed);
        if (DEBUG) {
            debugOutput += ", STOP";
        }
        // Read and output torque
        float torque = readMotorTorque(motor);
        if (torque != -9999) {
            outputTorque(motor, torque, debugOutput);
        }
        debugOutput += " | ";
        return;
    } else {
        // Both controls are high, do nothing
        if (DEBUG) {
            debugOutput += ", Both controls high, doing nothing";
        }
        debugOutput += " | ";
        return;
    }

    // Ramp the motor speed
    if (move_forward) {
        // Increase speed towards speed_max
        if (motor.current_speed < motor.speed_min) {
            motor.current_speed = motor.speed_min;
        } else if (motor.current_speed < motor.speed_max) {
            motor.current_speed += SPEED_STEP;
            if (motor.current_speed > motor.speed_max) {
                motor.current_speed = motor.speed_max;
            }
        }
    } else if (move_backward) {
        // Decrease speed towards -speed_max
        if (motor.current_speed > -motor.speed_min) {
            motor.current_speed = -motor.speed_min;
        } else if (motor.current_speed > -motor.speed_max) {
            motor.current_speed -= SPEED_STEP;
            if (motor.current_speed < -motor.speed_max) {
                motor.current_speed = -motor.speed_max;
            }
        }
    }

    // Send the speed command
    sendMotorSpeed(motor, motor.current_speed);

    if (DEBUG) {
        debugOutput += ", Speed: " + String(motor.current_speed);
    }

    // Read and output torque
    float torque = readMotorTorque(motor);
    if (torque != -9999) {
        outputTorque(motor, torque, debugOutput);
    }

    debugOutput += " | ";
}

/**
 * @brief Outputs the torque value to the analog output pin after mapping.
 *
 * @param motor: Reference to the MotorControl structure for the motor.
 * @param torque: The torque current in amperes.
 * @param debugOutput: Reference to the string for accumulating debug output.
 */
void outputTorque(MotorControl& motor, float torque, String& debugOutput) {
    // Clamp torque between -3A and 3A
    if (torque < -3.0f) {
        torque = -3.0f;
    } else if (torque > 3.0f) {
        torque = 3.0f;
    }

    // Map torque (-3A to +3A) to analog value (0 to 4095)
    int analogValue = (int)((torque + 3.0f) * (4095.0f / 6.0f));

    // Output the analog value to the analog output pin
    analogWrite(motor.analog_output_pin, analogValue);

    if (DEBUG) {
        debugOutput += ", Torque: " + String(torque, 2) + " A";
        debugOutput += ", Analog: " + String(analogValue);
    }
}

/**
 * @brief Reads the torque value from the motor via CAN bus.
 *
 * @param motor: Reference to the MotorControl structure for the motor.
 * @return float: The torque current in amperes, or -9999 if an error occurs.
 */
float readMotorTorque(MotorControl& motor) {
    unsigned long startTime = millis();
    struct can_frame canMsgIn;

    // Wait for a CAN message or timeout
    while (canController.readMessage(&canMsgIn) != MCP2515::ERROR_OK) {
        if (millis() - startTime >= CAN_TIMEOUT) {
            return -9999; // Indicate timeout or error
        }
    }

    // Check if the message is from the desired motor
    if (canMsgIn.can_id != motor.can_id) {
        return -9999; // Not the message we're interested in
    }

    // Extract torque from the CAN message
    float torque = extractMotorTorqueFromCAN(canMsgIn.data);

    return torque;
}

void setup() {
    Serial.begin(115200);

    // Initialize CAN communication
    canController.reset();
    canController.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    canController.setNormalMode();

    // Set control pins as INPUT
    for (int i = 0; i < NUM_MOTORS; i++) {
        pinMode(motors[i].control_pin_forward, INPUT);
        pinMode(motors[i].control_pin_backward, INPUT);
    }

    // Set analog output pins as OUTPUT
    for (int i = 0; i < NUM_MOTORS; i++) {
        pinMode(motors[i].analog_output_pin, OUTPUT);
        analogWriteResolution(motors[i].analog_output_pin,12);
    }
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