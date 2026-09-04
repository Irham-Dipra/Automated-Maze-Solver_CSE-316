// //esp to atmega start_stop command

// HardwareSerial AtmegaSerial(2);

// #define ATMEGA_RX_PIN 16   // Not connected for now
// #define ATMEGA_TX_PIN 17   // GPIO17 -> ATmega PD0/RXD

// void setup()
// {
//     // USB serial debugging
//     Serial.begin(115200);

//     // UART communication with ATmega32
//     AtmegaSerial.begin(
//         9600,
//         SERIAL_8N1,
//         ATMEGA_RX_PIN,
//         ATMEGA_TX_PIN
//     );

//     Serial.println("ESP32 started.");

//     // Give ATmega enough time to boot
//     delay(2000);
// }

// void loop()
// {
//     // ---------------------------
//     // Tell ATmega to RUN
//     // ---------------------------

//     Serial.println("Sending RUN");

//     AtmegaSerial.write('R');

//     // Run for 5 seconds
//     delay(5000);


//     // ---------------------------
//     // Tell ATmega to STOP
//     // ---------------------------

//     Serial.println("Sending STOP");

//     AtmegaSerial.write('S');

//     // Stop for 2 seconds
//     delay(2000);


//     // loop() repeats:
//     // RUN 5 sec -> STOP 2 sec -> RUN...
// }





//esp to atmega command sent after esp receives signal from sensor

// #include <Wire.h>
// #include <VL53L0X.h>

// // -------------------------
// // TCA9548A
// // -------------------------

// #define TCA_ADDR 0x70
// #define SENSOR_CHANNEL 0


// // -------------------------
// // ESP32 I2C pins
// // -------------------------

// #define SDA_PIN 21
// #define SCL_PIN 22


// // -------------------------
// // ESP32 -> ATmega UART
// // -------------------------

// #define ATMEGA_RX_PIN 16     // unused for now
// #define ATMEGA_TX_PIN 17     // GPIO17 -> ATmega PD0/RXD

// HardwareSerial AtmegaSerial(2);


// // -------------------------
// // VL53L0X object
// // -------------------------

// VL53L0X sensor;


// // -------------------------
// // Obstacle threshold
// // -------------------------

// #define STOP_DISTANCE 150    // millimeters


// // -------------------------
// // Keep track of current state
// // -------------------------

// bool carStopped = true;


// // ==================================================
// // Select one TCA9548A channel
// // ==================================================

// void selectTCAChannel(uint8_t channel)
// {
//     if (channel > 7)
//         return;

//     Wire.beginTransmission(TCA_ADDR);

//     Wire.write(1 << channel);

//     Wire.endTransmission();
// }


// // ==================================================
// // Send RUN command
// // ==================================================

// void sendRun()
// {
//     AtmegaSerial.write('R');

//     Serial.println("COMMAND: RUN");

//     carStopped = false;
// }


// // ==================================================
// // Send STOP command
// // ==================================================

// void sendStop()
// {
//     AtmegaSerial.write('S');

//     Serial.println("COMMAND: STOP");

//     carStopped = true;
// }


// // ==================================================
// // SETUP
// // ==================================================

// void setup()
// {
//     // Debug output to laptop
//     Serial.begin(115200);

//     delay(1000);

//     Serial.println();
//     Serial.println("ESP32 starting...");


//     // -------------------------
//     // Start I2C
//     // -------------------------

//     Wire.begin(SDA_PIN, SCL_PIN);


//     // -------------------------
//     // Select front sensor
//     // -------------------------

//     selectTCAChannel(SENSOR_CHANNEL);

//     delay(100);


//     // -------------------------
//     // Initialize VL53L0X
//     // -------------------------

//     sensor.setTimeout(500);

//     if (!sensor.init())
//     {
//         Serial.println("ERROR: VL53L0X not detected!");

//         while (1)
//         {
//         }
//     }

//     Serial.println("VL53L0X initialized");


//     // Continuous distance measurement
//     sensor.startContinuous(50);


//     // -------------------------
//     // Start UART to ATmega
//     // -------------------------

//     AtmegaSerial.begin(
//         9600,
//         SERIAL_8N1,
//         ATMEGA_RX_PIN,
//         ATMEGA_TX_PIN
//     );

//     Serial.println("UART initialized");


//     // Initially tell ATmega to stop
//     sendStop();

//     delay(1000);

//     Serial.println("System ready");
// }


// // ==================================================
// // LOOP
// // ==================================================

// void loop()
// {
//     // Make sure TCA channel 0 is active
//     selectTCAChannel(SENSOR_CHANNEL);


//     // Read distance
//     uint16_t distance =
//         sensor.readRangeContinuousMillimeters();


//     // -------------------------
//     // Check for sensor timeout
//     // -------------------------

//     if (sensor.timeoutOccurred())
//     {
//         Serial.println("Sensor timeout!");

//         // Safety behavior:
//         // stop the car if sensor fails

//         if (!carStopped)
//         {
//             sendStop();
//         }

//         delay(100);

//         return;
//     }


//     // -------------------------
//     // Print measured distance
//     // -------------------------

//     Serial.print("Distance: ");
//     Serial.print(distance);
//     Serial.println(" mm");


//     // ==================================================
//     // OBSTACLE DETECTED
//     // ==================================================

//     if (distance > 0 &&
//         distance < STOP_DISTANCE)
//     {
//         /*
//            Only send STOP once.

//            Without this check ESP32 would send
//            S S S S S S S...
//            continuously.
//         */

//         if (!carStopped)
//         {
//             sendStop();
//         }
//     }


//     // ==================================================
//     // PATH CLEAR
//     // ==================================================

//     else
//     {
//         /*
//            Only send RUN when we previously
//            stopped.
//         */

//         if (carStopped)
//         {
//             sendRun();
//         }
//     }


//     delay(50);
// }


//esp32 to tca connection checking code

#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

#define TCA_ADDR 0x70

void selectTCA(uint8_t channel)
{
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);

    uint8_t result = Wire.endTransmission();

    Serial.print("TCA select result: ");
    Serial.println(result);
}

void scanBus()
{
    int devices = 0;

    for (uint8_t address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);

        uint8_t error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("FOUND: 0x");

            if (address < 16)
                Serial.print("0");

            Serial.println(address, HEX);

            devices++;
        }
    }

    if (devices == 0)
    {
        Serial.println("NO I2C DEVICES FOUND");
    }
}

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("Starting I2C test...");

    Wire.begin(SDA_PIN, SCL_PIN);

    Serial.println();
    Serial.println("1. Scanning before selecting TCA channel:");

    scanBus();


    Serial.println();
    Serial.println("2. Selecting TCA channel 0:");

    selectTCA(0);

    delay(100);


    Serial.println();
    Serial.println("3. Scanning after selecting channel 0:");

    scanBus();
}

void loop()
{
}


/* running commands-
arduino-cli compile --fqbn esp32:esp32:esp32 .
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 .

after this, to see progress in terminal-
arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=115200

installation-
setup_esp32.sh 
*/