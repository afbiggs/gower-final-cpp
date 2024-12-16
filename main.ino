#include <WiFi.h>
#define SIO_DEBUG 0
#include <SocketIoClient.h>
#include <ArduinoJson.h>

// Constants and variables
const int materialForwardRelay = 15; // Relay for Material Forward Button
const int manualShearRelay = 2;     // Relay for Manual Shear Button
const int shearStartSwitch = 27;     // Limit switch to detect shear start
const int shearEndSwitch = 33;       // Limit switch to detect shear end

// // Constants and variables
// const int materialForwardRelay = 27; // Relay for Material Forward Button
// const int manualShearRelay = 26;     // Relay for Manual Shear Button
// const int shearStartSwitch = 34;     // Limit switch to detect shear start
// const int shearEndSwitch = 35;       // Limit switch to detect shear end


// const char* ssid = "Special Projects-5GHz";
// const char* password = "sprojects1!";
// const char* host = "192.168.1.186";
// const int port = 4300;

const char* ssid = "gauer";
const char* password = "adminadmin";
const char* host = "192.168.4.1";
const int port = 4300;

const int encoderPinA = 17;
const int encoderPinB = 34;

// const int encoderPinA = 16;
// const int encoderPinB = 17;
const int pulsesPerRevolution = 1600;
float wheelDiameterInches = 4;
float inchesPerPulse = (3.14159 * wheelDiameterInches) / pulsesPerRevolution;

volatile long encoderCount = 0;
float travelDistanceInches = 0.0;

float inputLength = 0;  // Cut length in inches
int inputQuantity = 0;  // Desired number of cuts
int cutCount = 0;       // Current number of cuts

bool feedingMaterial = false;    // State flag for material feed
volatile bool isPaused = false; // Tracks if the feed motor is paused
bool cuttingMaterial = false;    // State flag for cutting process
volatile bool isReset = false;
// bool isMaterialForwardManual = false; // Tracks if manual control is active
// bool isMaterialForwardOn = false;    // Tracks the relay 
bool manualOverride = false; // Flag to indicate manual override for material forward relay
bool manualShearOverride = false; // Flag to indicate manual override for manual shear relay
bool isEStopActive = false; // Tracks if the E-Stop is active
volatile bool eStopResetRequired = false; // Tracks if the E-Stop needs a reset
volatile bool isResumeRequired = false;     // Tracks if Resume button needs to be pressed to continue






SocketIoClient socket;

// Function prototypes
void handleCutParameters(const char* data, size_t length);
void updateWheelDiameter(float newDiameter);
void sendCutCount();
void readSerialData(); // Use existing readSerialData function as is




void initializeRelays() {
    pinMode(materialForwardRelay, OUTPUT);
    digitalWrite(materialForwardRelay, LOW); // Ensure relay is OFF initially
    pinMode(manualShearRelay, OUTPUT);
    digitalWrite(manualShearRelay, LOW); // Ensure relay is OFF initially
    Serial.println("Relays initialized.");
}

void initializeLimitSwitches() {
    pinMode(shearStartSwitch, INPUT_PULLUP); // Limit switch for shear start
    pinMode(shearEndSwitch, INPUT_PULLUP);   // Limit switch for shear end
    Serial.println("Limit switches initialized.");
}

// bool isSwitchPressed(int pin) {
//     const int debounceDelay = 50; // milliseconds
//     if (digitalRead(pin) == LOW) { // Switch is pressed when pin reads LOW
//         delay(debounceDelay); // Wait for the signal to stabilize
//         if (digitalRead(pin) == LOW) {
//             return true; // Confirmed switch press
//         }
//     }
//     return false; // Switch is not pressed
// }


void updateWheelDiameter(float newDiameter) {
    // Update the wheel diameter and recalculate inches per pulse
    wheelDiameterInches = newDiameter;
    inchesPerPulse = (3.14159 * wheelDiameterInches) / pulsesPerRevolution;

    Serial.printf("Updated wheel diameter: %.3f inches\n", wheelDiameterInches);
    Serial.printf("Updated inches per pulse: %.6f\n", inchesPerPulse);
}


// bool isSwitchPressed(int pin) {
//     // Debounce logic for switches set as INPUT_PULLUP
//     const int debounceDelay = 50; // milliseconds
//     if (digitalRead(pin) == LOW) { // Switch is pressed when pin reads LOW
//         delay(debounceDelay);
//         if (digitalRead(pin) == LOW) {
//             return true; // Confirmed switch press
//         }
//     }
//     return false; // Switch is not pressed
// }


void initializeEncoders() {
    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), []() { encoderCount++; }, CHANGE);
    Serial.println("Encoder pins initialized.");
}



void connectToWiFi() {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");
}
void configureSocketIOHandlers() {
    socket.on("connect", [](const char* payload, size_t length) {
        Serial.println("Connected to Socket.IO server");
    });

    socket.on("confirm_reset", [](const char* payload, size_t length) {
        Serial.println("Confirm reset command received.");
        isReset = true; // Set the reset state
    });


   socket.on("material_forward_control", [](const char* payload, size_t length) {
    Serial.printf("Material Forward Control Command: %s\n", payload);

    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return;
    }

    if (doc.containsKey("materialForward")) {
        String command = doc["materialForward"].as<String>();

        // Handle ON command
        if (command == "ON") {
            manualOverride = true; // Enable manual override
            digitalWrite(materialForwardRelay, HIGH); // Turn relay ON
            Serial.println("Material Forward Relay ON (Manual Override).");
        }

        // Handle OFF command
        else if (command == "OFF") {
            manualOverride = false; // Disable manual override
            digitalWrite(materialForwardRelay, LOW); // Turn relay OFF
            Serial.println("Material Forward Relay OFF (Manual Override).");

            // Add a short delay to stabilize relay switching
            delay(50);
        }

        // Handle invalid command
        else {
            Serial.println("Invalid Material Forward Command.");
        }
    } else {
        Serial.println("Invalid JSON for material_forward_control.");
    }
});


    // // Add debounce logic to prevent rapid relay state changes
    // delay(100); // Add a small delay to debounce

    // Handle Manual Shear Button control
    socket.on("manual_shear_control", [](const char* payload, size_t length) {
    Serial.printf("Manual Shear Control Command: %s\n", payload);

    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return;
    }

    if (doc.containsKey("manualShear")) {
        String command = doc["manualShear"].as<String>();

        if (command == "ON") {
            manualShearOverride = true; // Enable manual override
            digitalWrite(manualShearRelay, HIGH); // Turn relay ON
            Serial.println("Manual Shear Relay ON (Manual Override).");
        } else if (command == "OFF") {
            manualShearOverride = false; // Disable manual override
            digitalWrite(manualShearRelay, LOW); // Turn relay OFF
            Serial.println("Manual Shear Relay OFF (Manual Override).");
        } else {
            Serial.println("Invalid Manual Shear Command.");
        }
    } else {
        Serial.println("Invalid JSON for manual_shear_control.");
    }
});


socket.on("motor_command", [](const char* payload, size_t length) {
    Serial.printf("Received motor_command: %.*s\n", length, payload); // Debug: Print raw payload

    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, payload, length); // Pass `length` for safer parsing

    if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return; // Exit if deserialization fails
    }

    if (!doc.containsKey("motor")) {
        Serial.println("Invalid JSON: Missing 'motor' key.");
        return; // Exit if required key is missing
    }

    String motorCommand = doc["motor"].as<String>();
    Serial.printf("Motor Command: %s\n", motorCommand.c_str()); // Debug: Log parsed command

    if (motorCommand == "PAUSE") {
        if (!isPaused) {
            isPaused = true;
            digitalWrite(materialForwardRelay, LOW);
            digitalWrite(manualShearRelay, LOW);
            Serial.println("Motor paused via socket command.");
        } else {
            Serial.println("Motor is already paused.");
        }
    } else if (motorCommand == "RESUME") {
        if (isPaused && !isEStopActive) {
            isPaused = false;
            Serial.println("Motor resumed via socket command.");
        } else if (isEStopActive) {
            Serial.println("Cannot resume. E-Stop is active.");
        } else {
            Serial.println("Resume not required. Machine is already running.");
        }
    } else {
        Serial.printf("Invalid motor command received: %s\n", motorCommand.c_str());
    }
});

socket.on("update_wheel_diameter", [](const char* payload, size_t length) {
    Serial.printf("Wheel Diameter Update Command: %s\n", payload);

    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return;
    }

    if (doc.containsKey("wheelDiameter")) {
        float newDiameter = doc["wheelDiameter"].as<float>();

        if (newDiameter > 0) {
            updateWheelDiameter(newDiameter); // Function to update wheel diameter and related calculations
            Serial.printf("Wheel diameter updated to: %.3f inches\n", newDiameter);
        } else {
            Serial.println("Invalid wheel diameter value.");
        }
    } else {
        Serial.println("Invalid JSON: No 'wheelDiameter' key.");
    }
});




   

    // Handle E-Stop activation
    socket.on("e_stop", [](const char* payload, size_t length) {
        isEStopActive = true;
        isResumeRequired = true;
        isPaused = true;
        digitalWrite(materialForwardRelay, LOW);
        digitalWrite(manualShearRelay, LOW);
        Serial.println("E-Stop activated: Relays OFF. Resume required.");
    });

    // Handle Reset E-Stop
    socket.on("reset_e_stop", [](const char* payload, size_t length) {
        isEStopActive = false;
        isResumeRequired = false;
        Serial.println("E-Stop reset: Ready for resume.");
    });

    // Handle Reset command
    socket.on("reset", [](const char* payload, size_t length) {
        isReset = true;
        Serial.println("Reset command received. Waiting for current cut to finish...");
    });

    // Handle Cut Parameters
    socket.on("set_cut_parameters", [](const char* payload, size_t length) {
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (!error && doc.containsKey("cutLength") && doc.containsKey("cutQuantity")) {
        inputLength = doc["cutLength"];
        inputQuantity = doc["cutQuantity"];
        Serial.printf("Cut parameters received - Length: %.3f, Quantity: %d\n", inputLength, inputQuantity);

        // Clear reset and pause states before starting
        if (isReset) {
            Serial.println("Clearing reset state.");
            isReset = false;
        }

        if (isPaused) {
            Serial.println("Clearing paused state.");
            isPaused = false;
        }

        if (inputLength > 0 && inputQuantity > 0) {
            encoderCount = 0;
            travelDistanceInches = 0.0;
            digitalWrite(materialForwardRelay, HIGH); // Start material feed
            Serial.println("Motor started for cutting process.");
        } else {
            Serial.println("Invalid parameters. Cannot start the machine.");
        }
    } else {
        Serial.println("Invalid cut parameters payload.");
    }
});

    Serial.println("Socket.IO event handlers configured.");
}



void setup() {
    Serial.begin(115200);
    // Serial1.begin(115200);
    Serial.println("Ready to receive JSON...");

    initializeRelays();
    initializeLimitSwitches();
    initializeEncoders();
    connectToWiFi();
    configureSocketIOHandlers();

    socket.begin(host, port);
    Serial.println("Setup completed.");
}



void loop() {
    static unsigned long lastEmitTime = 0;
    unsigned long currentTime = millis();

    // Check for E-Stop or Resume Requirement
    if (isEStopActive || isResumeRequired) {
        digitalWrite(materialForwardRelay, LOW);
        digitalWrite(manualShearRelay, LOW);
        cuttingMaterial = false;
        feedingMaterial = false;
        socket.loop();
        // readSerialData();
        delay(5);
        return; // Skip further processing
    }

    // Handle Manual Overrides
    if (manualOverride || manualShearOverride) {
        if (manualOverride) {
            digitalWrite(materialForwardRelay, HIGH);
        } else {
            digitalWrite(materialForwardRelay, LOW);
        }
        if (manualShearOverride) {
            digitalWrite(manualShearRelay, HIGH);
        } else {
            digitalWrite(manualShearRelay, LOW);
        }
        socket.loop();
        // readSerialData();
        delay(5);
        return; // Skip the rest of the loop
    }

    // Core Cutting Process
   if (!isPaused && !isReset && inputQuantity > 0) {
    travelDistanceInches = encoderCount * inchesPerPulse;

    if (travelDistanceInches < inputLength) {
        digitalWrite(materialForwardRelay, HIGH); // Continue feeding material
        cuttingMaterial = true;
    } else {
        digitalWrite(materialForwardRelay, LOW); // Stop feeding material
        Serial.println("Target length reached. Material feed stopped.");

        // Start Shear Process
        digitalWrite(manualShearRelay, HIGH);
        Serial.println("Shear process started.");
        sleep(10)

        digitalWrite(manualShearRelay, LOW);

        // // Wait for Shear End Switch
        // while (digitalRead(shearEndSwitch) == HIGH) {
        //     delay(10);
        // }
        // Serial.println("Shear End Switch triggered. Shear process complete.");
        // digitalWrite(manualShearRelay, LOW);

        // // Wait for Shear Start Switch
        // Serial.println("Waiting for Shear Start Switch...");
        // while (digitalRead(shearStartSwitch) == HIGH) {
        //     delay(10);
        // }
        Serial.println("Shear Start Switch triggered. Starting material feed...");

        // Increment Cut Count and Reset Encoder
        cutCount++;
        sendCutCount();

        if (isReset) {
            inputQuantity = 0;
            cuttingMaterial = false;
            Serial.println("Reset triggered after completing current cut.");
            return;
        }

        inputQuantity--;
        encoderCount = 0;
        travelDistanceInches = 0.0;
        cuttingMaterial = false;
    }
}

    
    else {
        digitalWrite(materialForwardRelay, LOW);
        digitalWrite(manualShearRelay, LOW);
        cuttingMaterial = false;
    }

    // Handle Reset State
    if (isReset) {
    if (cuttingMaterial || (inputQuantity > 0 && !isPaused)) {
        Serial.println("Cannot reset while cutting or feeding material. Pause the machine first.");
        isReset = false; // Clear the reset request to avoid infinite loop
        return; // Exit the loop early
    } else {
        // Perform reset when the machine is paused
        Serial.println("Resetting machine...");
        digitalWrite(materialForwardRelay, LOW);
        digitalWrite(manualShearRelay, LOW);
        encoderCount = 0;
        travelDistanceInches = 0.0;
        inputLength = 0;
        inputQuantity = 0;
        cutCount = 0;
        isPaused = false; // Ensure paused state is cleared
        cuttingMaterial = false; // Ensure no cutting is in progress
        isReset = false; // Clear reset state
        Serial.println("System reset completed. Ready for new parameters.");
    }
    return; // Exit the loop early to prevent further processing
}



    // Emit Travel Distance Every 50ms
    if (currentTime - lastEmitTime >= 50) {
        lastEmitTime = currentTime;
        sendTravelDistance();
    }

    socket.loop();
    // readSerialData();
    delay(5);
}


void sendTravelDistance() {
    float travelDistance = encoderCount * inchesPerPulse;

    // Create a JSON object to emit
    StaticJsonDocument<128> doc;
    doc["travelDistance"] = travelDistance;

    // // Serialize and emit the JSON
    String output;
    serializeJson(doc, output);
    socket.emit("travel_distance", output.c_str());

    // Ensure only JSON is sent to the serial port
    Serial.println(output);
}


void sendCutCount() {
    StaticJsonDocument<128> doc;
    doc["cutCount"] = cutCount;

    String output;
    serializeJson(doc, output);
    socket.emit("cut_status", output.c_str());

    Serial.print("Sent cut count to server: ");
    Serial.println(output);
}
