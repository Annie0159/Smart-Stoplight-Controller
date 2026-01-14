#include <Arduino.h>
#include <TM1637Display.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

// --- Pin Definitions
// Traffic Light 1
#define LED1_R_PIN 42 // Red (D1)
#define LED3_Y_PIN 41 // Yellow (D3)
#define LED2_G_PIN 40 // Green (D2)

// Traffic Light 2 
#define LED4_R_PIN 37 // Red (D4/D7)
#define LED5_Y_PIN 38 // Yellow (D5/D8)
#define LED6_G_PIN 39 // Green (D6/D9)

// Crosswalk Request/Status LEDs (White LEDs)
#define LED_CW_REQ_PIN 36 // Crosswalk Requested Indicator 

// 7-Segment Display (TM1637)
#define DIO_PIN 4
#define CLK_PIN 2

// PIR Sensors (Vehicle Detection)
#define PIR1_PIN 14 // Direction 1 PIR
#define PIR2_PIN 15 // Direction 2 PIR

// Push Buttons (Crosswalk Request)
#define BTN1_PIN 13 // Crosswalk Button 1
#define BTN2_PIN 12 // Crosswalk Button 2

// --- FreeRTOS Handles and Globals ---
// Semaphores
SemaphoreHandle_t xCrosswalkSemaphore = NULL;
SemaphoreHandle_t xGatekeeperSemaphore = NULL; // Protects shared state/data
// Queues
QueueHandle_t xSerialQueue = NULL; // Queue for Gatekeeper Task
// Timers
TimerHandle_t xLightTimer = NULL;
TimerHandle_t xCrosswalkTimer = NULL;

// Global State Variables
volatile bool isCrosswalkRequested = false;
volatile bool isVehicle1Detected = false;
volatile bool isVehicle2Detected = false;

// Traffic Light State Machine
enum LightState {
    TL2_TO_TL1_SAFETY,    // Both Red (Safety gap, next is TL1_GREEN)
    TL1_GREEN,
    TL1_YELLOW,
    TL1_TO_TL2_SAFETY,    // Both Red (Safety gap, next is TL2_GREEN)
    TL2_GREEN,
    TL2_YELLOW,
    CROSSWALK_ACTIVE      // All Red for Crosswalk (20s + 2s safety)
};

LightState nextStateAfterCrosswalk = TL1_GREEN;

// Update the initial state:
LightState currentState = TL2_TO_TL1_SAFETY; // Start by going to TL1_GREEN next

// Schedule Timings (in seconds)
#define SCHEDULE_TIME_R 26
#define SCHEDULE_TIME_G 18
#define SCHEDULE_TIME_Y 4
#define SAFETY_TIME 2
#define VEHICLE_REDUCE_TIME 8 // Minimum/Reduced Green time
#define CROSSWALK_TIME 20

// Display Object
TM1637Display display(CLK_PIN, DIO_PIN);


// --- Serial Gatekeeper Task Structure ---
struct SerialMessage {
    char message[64];
};

// --- Function Prototypes ---
void vLightControlTask(void* pvParameters);
void vStatusStreamTask(void* pvParameters);
void vGatekeeperTask(void* pvParameters);
void vCrosswalkDisplayTask(void* pvParameters);
void xLightTimerCallback(TimerHandle_t xTimer);
void xCrosswalkTimerCallback(TimerHandle_t xTimer);
void xPostSerialMessage(const char* format, ...);
void setupHardware();
void setLightState(LightState state);

// --- Light Control Logic ---

// Helper function to set the state of a single traffic light
void setTrafficLight(int redPin, int yellowPin, int greenPin, bool red, bool yellow, bool green) {
    digitalWrite(redPin, red ? HIGH : LOW);
    digitalWrite(yellowPin, yellow ? HIGH : LOW);
    digitalWrite(greenPin, green ? HIGH : LOW);
}

// Function to transition the state machine
void setLightState(LightState state) {
    currentState = state;
    int timerPeriod_ms = 0;

    // Reset vehicle detection after light goes green/yellow (vehicle is now moving)
    if (state == TL1_GREEN) {
        isVehicle1Detected = false;
    } else if (state == TL2_GREEN) {
        isVehicle2Detected = false;
    }

    switch (state) {
        case TL2_TO_TL1_SAFETY:
            // Both Red (Safety Gap, coming from TL2_YELLOW, going to TL1_GREEN)
            setTrafficLight(LED1_R_PIN, LED3_Y_PIN, LED2_G_PIN, true, false, false);
            setTrafficLight(LED4_R_PIN, LED5_Y_PIN, LED6_G_PIN, true, false, false);
            timerPeriod_ms = SAFETY_TIME * 1000;
            xPostSerialMessage("STATE: Both Red (Safety Gap TL2->TL1). Next: TL1_GREEN");
            break;

        case TL1_TO_TL2_SAFETY:
            // Both Red (Safety Gap, coming from TL1_YELLOW, going to TL2_GREEN)
            setTrafficLight(LED1_R_PIN, LED3_Y_PIN, LED2_G_PIN, true, false, false);
            setTrafficLight(LED4_R_PIN, LED5_Y_PIN, LED6_G_PIN, true, false, false);
            timerPeriod_ms = SAFETY_TIME * 1000;
            xPostSerialMessage("STATE: Both Red. Next: TL2_GREEN");
            break;

        case TL1_GREEN: {
            // Light 1 Green, Light 2 Red
            setTrafficLight(LED1_R_PIN, LED3_Y_PIN, LED2_G_PIN, false, false, true);
            setTrafficLight(LED4_R_PIN, LED5_Y_PIN, LED6_G_PIN, true, false, false);
            
            int greenTime = SCHEDULE_TIME_G;
            // Check for Light 2 vehicle presence 
            if (isVehicle2Detected) {
                greenTime = min(SCHEDULE_TIME_G, VEHICLE_REDUCE_TIME);
                xPostSerialMessage("VEHICLE LOGIC: Reducing TL1 Green Time to %ds due to waiting TL2 vehicle.", greenTime);
            }
            timerPeriod_ms = greenTime * 1000;
            xPostSerialMessage("STATE: TL1 Green (%ds). Next: TL1_YELLOW", greenTime);
            break;
        }

        case TL1_YELLOW:
            // Light 1 Yellow, Light 2 Red
            setTrafficLight(LED1_R_PIN, LED3_Y_PIN, LED2_G_PIN, false, true, false);
            setTrafficLight(LED4_R_PIN, LED5_Y_PIN, LED6_G_PIN, true, false, false);
            timerPeriod_ms = SCHEDULE_TIME_Y * 1000;
            xPostSerialMessage("STATE: TL1 Yellow (%ds). Next: Safety Gap TL1->TL2", SCHEDULE_TIME_Y);
            break;

        case TL2_GREEN: {
            // Light 2 Green, Light 1 Red
            setTrafficLight(LED1_R_PIN, LED3_Y_PIN, LED2_G_PIN, true, false, false);
            setTrafficLight(LED4_R_PIN, LED5_Y_PIN, LED6_G_PIN, false, false, true);
            
            int greenTime = SCHEDULE_TIME_G;
            // Check for Light 1 vehicle presence
            if (isVehicle1Detected) {
                greenTime = min(SCHEDULE_TIME_G, VEHICLE_REDUCE_TIME);
                // ADDED LOGGING HERE:
                xPostSerialMessage("VEHICLE LOGIC: Reducing TL2 Green Time to %ds due to waiting TL1 vehicle.", greenTime);
            }
            timerPeriod_ms = greenTime * 1000;
            xPostSerialMessage("STATE: TL2 Green (%ds). Next: TL2_YELLOW", greenTime);
            break;
        }

        case TL2_YELLOW:
            // Light 2 Yellow, Light 1 Red
            setTrafficLight(LED1_R_PIN, LED3_Y_PIN, LED2_G_PIN, true, false, false);
            setTrafficLight(LED4_R_PIN, LED5_Y_PIN, LED6_G_PIN, false, true, false);
            timerPeriod_ms = SCHEDULE_TIME_Y * 1000;
            xPostSerialMessage("STATE: TL2 Yellow (%ds). Next: Safety Gap TL2->TL1", SCHEDULE_TIME_Y);
            break;
            
        case CROSSWALK_ACTIVE:
            // Both Red for Crosswalk
            setTrafficLight(LED1_R_PIN, LED3_Y_PIN, LED2_G_PIN, true, false, false);
            setTrafficLight(LED4_R_PIN, LED5_Y_PIN, LED6_G_PIN, true, false, false);
            timerPeriod_ms = (CROSSWALK_TIME + SAFETY_TIME) * 1000; // 20s display + 2s safety
            
            // Start the crosswalk timer for the countdown display
            xTimerChangePeriod(xCrosswalkTimer, pdMS_TO_TICKS(1000), 0); 
            xTimerStart(xCrosswalkTimer, 0);
            xPostSerialMessage("STATE: CROSSWALK Active 20s", CROSSWALK_TIME + SAFETY_TIME);
            break;
    }

    // Set the new period and start/restart the main light timer
    xTimerChangePeriod(xLightTimer, pdMS_TO_TICKS(timerPeriod_ms), 0);
    xTimerStart(xLightTimer, 0);
}

// Timer Callback for the main light schedule
void xLightTimerCallback(TimerHandle_t xTimer) {
    if (xGatekeeperSemaphore == NULL) return;
    xSemaphoreTake(xGatekeeperSemaphore, portMAX_DELAY);

    // State machine transitions
    switch (currentState) {
      case TL2_TO_TL1_SAFETY:
          // Came from TL2_YELLOW. Go to the next major phase: TL1_GREEN.
          setLightState(TL1_GREEN);
          break;

      case TL1_GREEN:
          setLightState(TL1_YELLOW);
          break;

      case TL1_YELLOW:
          if (isCrosswalkRequested) {
              setLightState(CROSSWALK_ACTIVE);
              // Acknowledge request here before leaving Yellow phase
              nextStateAfterCrosswalk = TL1_TO_TL2_SAFETY;
              isCrosswalkRequested = false; 
              digitalWrite(LED_CW_REQ_PIN, LOW);
          } else {
              setLightState(TL1_TO_TL2_SAFETY);
          }
          break;

      case TL1_TO_TL2_SAFETY:
          // Came from TL1_YELLOW. Go to the next major phase: TL2_GREEN.
          setLightState(TL2_GREEN);
          break;

      case TL2_GREEN:
          setLightState(TL2_YELLOW);
          break;

      case TL2_YELLOW:
          if (isCrosswalkRequested) {
              setLightState(CROSSWALK_ACTIVE);
              // Acknowledge request here before leaving Yellow phase
              nextStateAfterCrosswalk = TL2_TO_TL1_SAFETY;
              isCrosswalkRequested = false;
              digitalWrite(LED_CW_REQ_PIN, LOW);
          } else {
              setLightState(TL2_TO_TL1_SAFETY);
          }
          break;
          
      case CROSSWALK_ACTIVE:
          xTimerStop(xCrosswalkTimer, 0); // Stop crosswalk countdown
          display.clear(); // Clear display
          setLightState(nextStateAfterCrosswalk);
          break;
  }

    xSemaphoreGive(xGatekeeperSemaphore);
}

// Timer Callback for the Crosswalk Countdown Display
void xCrosswalkTimerCallback(TimerHandle_t xTimer) {
    // Acquire the mutex for shared state access and display write
    if (xGatekeeperSemaphore == NULL) return;
    xSemaphoreTake(xGatekeeperSemaphore, portMAX_DELAY);

    static int countdown = CROSSWALK_TIME;
    
    // Only run if in CROSSWALK_ACTIVE state
    if (currentState != CROSSWALK_ACTIVE) {
        countdown = CROSSWALK_TIME; // Reset counter
        xSemaphoreGive(xGatekeeperSemaphore); // Release mutex
        return;
    }
    
    // Display the count
    display.showNumberDecEx(countdown, 0b01000000, true, 4, 0); // Show count with dot (for crosswalk)
    
    countdown--;
    
    // When countdown hits -2 (i.e., 0s on display + 2s safety), the main light timer takes over
    if (countdown < 0) { // Changed < 0 to < -1 to account for 0s display + 2s safety
        countdown = CROSSWALK_TIME; // Reset for next time
    }

    xSemaphoreGive(xGatekeeperSemaphore); // Release mutex
}


// --- Interrupt Service Routines (ISRs) ---

// PIR Sensor 1 ISR
void IRAM_ATTR pir1ISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (digitalRead(PIR1_PIN) == HIGH) {
        isVehicle1Detected = true;
        xPostSerialMessage("EVENT: Vehicle Detected (Direction 1)");
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// PIR Sensor 2 ISR
void IRAM_ATTR pir2ISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (digitalRead(PIR2_PIN) == HIGH) {
        isVehicle2Detected = true;
        xPostSerialMessage("EVENT: Vehicle Detected (Direction 2)");
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Crosswalk Button ISR
void IRAM_ATTR buttonISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Check if a request is already active
    if (!isCrosswalkRequested) {
        // Signal the LED/Light Control task via binary semaphore
        xSemaphoreGiveFromISR(xCrosswalkSemaphore, &xHigherPriorityTaskWoken);
        xPostSerialMessage("EVENT: Crosswalk Requested");
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// --- Tasks ---

// Crosswalk Request Task (Triggered by semaphore)
void vCrosswalkRequestTask(void* pvParameters) {
    for (;;) {
        // Wait for the semaphore to be given by an ISR
        if (xSemaphoreTake(xCrosswalkSemaphore, portMAX_DELAY) == pdTRUE) {
            // Semaphore received (Crosswalk requested)
            if (xGatekeeperSemaphore == NULL) continue;
            
            // Acquire the mutex before modifying global state
            xSemaphoreTake(xGatekeeperSemaphore, portMAX_DELAY);
            
            isCrosswalkRequested = true;
            digitalWrite(LED_CW_REQ_PIN, HIGH); // Turn on the crosswalk requested LED
            
            xSemaphoreGive(xGatekeeperSemaphore);
        }
    }
}

//Serial Communication Gatekeeper Task
void vGatekeeperTask(void* pvParameters) {
    SerialMessage message;

    for (;;) {
        // Wait for a message on the queue
        if (xQueueReceive(xSerialQueue, &message, portMAX_DELAY) == pdPASS) {
            // This task is the only one allowed to call Serial.print/write
            Serial.println(message.message);
        }
    }
}

// Helper function to post a message to the Gatekeeper Queue
void xPostSerialMessage(const char* format, ...) {
    SerialMessage message;
    va_list args;

    va_start(args, format);
    vsnprintf(message.message, sizeof(message.message), format, args);
    va_end(args);

    // Send the message to the queue. Non-blocking is preferred here.
    xQueueSend(xSerialQueue, &message, 0); 
}

// Status Streaming Task
void vStatusStreamTask(void* pvParameters) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Stream every 1 second (1d)

        if (xGatekeeperSemaphore == NULL) continue;

        xSemaphoreTake(xGatekeeperSemaphore, portMAX_DELAY);
        
        // Get remaining time on the current light timer
        TickType_t xRemainingTicks = xTimerGetExpiryTime(xLightTimer) - xTaskGetTickCount();
        int remainingSeconds = xRemainingTicks / configTICK_RATE_HZ; // Convert ticks to seconds
        if(currentState == TL1_GREEN && isVehicle1Detected)
        {
          isVehicle1Detected = false;
        }
        else if(currentState == TL2_GREEN && isVehicle2Detected)
        {
          isVehicle2Detected = false;
        }

        if (currentState == TL1_GREEN && isVehicle2Detected) {
            // TL1 is Green, but a vehicle is detected waiting at TL2 (which is Red)
            if (remainingSeconds > VEHICLE_REDUCE_TIME) {
                // If remaining time is longer than the minimum, reduce it.
                TickType_t newPeriodTicks = pdMS_TO_TICKS(VEHICLE_REDUCE_TIME * 1000);
                xTimerChangePeriod(xLightTimer, newPeriodTicks, 0);
                xPostSerialMessage("ACTION: TL1 Green reduced to 8s remaining due to TL2 vehicle.");
            }
        } else if (currentState == TL2_GREEN && isVehicle1Detected) {
            // TL2 is Green, but a vehicle is detected waiting at TL1 (which is Red)
            if (remainingSeconds > VEHICLE_REDUCE_TIME) {
                // If remaining time is longer than the minimum, reduce it.
                TickType_t newPeriodTicks = pdMS_TO_TICKS(VEHICLE_REDUCE_TIME * 1000);
                xTimerChangePeriod(xLightTimer, newPeriodTicks, 0);
                xPostSerialMessage("ACTION: TL2 Green reduced to 8s remaining due to TL1 vehicle.");
            }
        }
        
        // Map the enum state to a string for serial output 
        const char* stateStr;
        switch (currentState) {
            case TL2_TO_TL1_SAFETY: stateStr = "TL2_TO_TL1_SAFETY"; break;
            case TL1_TO_TL2_SAFETY: stateStr = "TL1_TO_TL2_SAFETY"; break;
            case TL1_GREEN: stateStr = "TL1_GREEN"; break;
            case TL1_YELLOW: stateStr = "TL1_YELLOW"; break;
            case TL2_GREEN: stateStr = "TL2_GREEN"; break;
            case TL2_YELLOW: stateStr = "TL2_YELLOW"; break;
            case CROSSWALK_ACTIVE: stateStr = "CROSSWALK_ACTIVE"; break;
            default: stateStr = "UNKNOWN"; break;
        }

        xPostSerialMessage(
            "STATUS: State=%s | CW_Req=%s | Veh1=%s | Veh2=%s", 
            stateStr,
            isCrosswalkRequested ? "YES" : "NO",
            isVehicle1Detected ? "YES" : "NO",
            isVehicle2Detected ? "YES" : "NO"
        );

        xSemaphoreGive(xGatekeeperSemaphore);
    }
}

// --- Setup and Main Loop ---

void setupHardware() {
    // Light Pins
    pinMode(LED1_R_PIN, OUTPUT);
    pinMode(LED3_Y_PIN, OUTPUT);
    pinMode(LED2_G_PIN, OUTPUT);
    pinMode(LED4_R_PIN, OUTPUT);
    pinMode(LED5_Y_PIN, OUTPUT);
    pinMode(LED6_G_PIN, OUTPUT);
    pinMode(LED_CW_REQ_PIN, OUTPUT);

    // PIR Pins 
    pinMode(PIR1_PIN, INPUT);
    pinMode(PIR2_PIN, INPUT);
    
    // Button Pins 
    pinMode(BTN1_PIN, INPUT_PULLUP);
    pinMode(BTN2_PIN, INPUT_PULLUP);

    // Attach Interrupts
    attachInterrupt(digitalPinToInterrupt(PIR1_PIN), pir1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(PIR2_PIN), pir2ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(BTN1_PIN), buttonISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN2_PIN), buttonISR, FALLING);

    // TM1637 Display
    display.setBrightness(0x0a);
}

void setup() {
    Serial.begin(115200);
    setupHardware();

    // Create Semaphores
    xCrosswalkSemaphore = xSemaphoreCreateBinary();
    xGatekeeperSemaphore = xSemaphoreCreateMutex();
    // Create Queue
    xSerialQueue = xQueueCreate(10, sizeof(SerialMessage));
    
    delay(2000); 
    display.clear();


    if (xCrosswalkSemaphore == NULL || xGatekeeperSemaphore == NULL || xSerialQueue == NULL) {
        Serial.println("FATAL: RTOS primitive creation failed.");
        while(1);
    }

    // Create Software Timers 
    // Main light timer 
    xLightTimer = xTimerCreate(
        "LightTimer",
        pdMS_TO_TICKS(1000), // Initial period 
        pdFALSE,             // Auto-reload
        (void*)0,
        xLightTimerCallback
    );
    
    // Crosswalk Countdown timer 
    xCrosswalkTimer = xTimerCreate(
        "CWTimer",
        pdMS_TO_TICKS(1000), // Period 1 second
        pdTRUE,              // Auto-reload
        (void*)1,
        xCrosswalkTimerCallback
    );

    if (xLightTimer == NULL || xCrosswalkTimer == NULL) {
        Serial.println("FATAL: Timer creation failed.");
        while(1);
    }
    
    // Create Tasks
    xTaskCreate(vGatekeeperTask, "Gatekeeper", 4096, NULL, 3, NULL);
    xTaskCreate(vCrosswalkRequestTask, "CrosswalkReq", 2048, NULL, 2, NULL);
    xTaskCreate(vStatusStreamTask, "StatusStream", 2048, NULL, 1, NULL);

    // Initial State Setup (Needs to be done after semaphores/timers are created)
    // Start the light sequence (The initial state TL1_RED_TL2_RED has a 2s safety gap before moving to TL1_GREEN)
    xPostSerialMessage("SYSTEM: Starting Traffic Light Simulation...");
    xSemaphoreTake(xGatekeeperSemaphore, portMAX_DELAY);
    // Change the starting state to the new safety gap leading to the first Green phase (TL1)
    setLightState(TL2_TO_TL1_SAFETY); 
    xSemaphoreGive(xGatekeeperSemaphore);
}

void loop() {
   delay(100);
}