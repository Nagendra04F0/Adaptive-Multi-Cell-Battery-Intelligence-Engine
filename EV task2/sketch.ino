#include <Arduino.h>

/* --- Hardware Pin Mapping --- */
#define PIN_RELAY             12
#define PIN_BUZZER            13
#define NUM_CELLS             4

/* --- Architectural System Constants --- */
#define TICK_INTERVAL_MS      50    // Kernel processing cycle rate (20Hz)
#define DEBOUNCE_MS           1000  // Fault validation window (1 second persistence)
#define COOLDOWN_MS           5000  // Anti-chatter isolation timeout (5 seconds)

/* --- Safety Thresholds --- */
#define V_CRITICAL_OVER       4.25f // Volts
#define V_WARNING_OVER        4.20f // Volts
#define V_WARNING_UNDER       2.80f // Volts
#define V_CRITICAL_UNDER      2.50f // Volts
#define MAX_VOLTAGE_DELTA     0.30f // Rapid fluctuation limit within one tick loop

/* --- Type Definitions --- */
typedef enum {
    KERNEL_NORMAL,
    KERNEL_WARNING,
    KERNEL_TRIPPED
} KernelState_t;

typedef enum {
    FAULT_NONE             = 0x00,
    FAULT_UNDER_VOLTAGE    = 0x01,
    FAULT_OVER_VOLTAGE     = 0x02,
    FAULT_RAPID_FLUC       = 0x04,
    FAULT_SENSOR_ANOMALY   = 0x08
} SafetyFault_t;

typedef struct {
    float cell_voltages[NUM_CELLS];
    float previous_voltages[NUM_CELLS];
    uint8_t active_fault_mask;
    uint32_t fault_timestamp;
    uint32_t state_change_timestamp;
    KernelState_t current_state;
    bool relay_closed;
} SafetyKernel_t;

// Global System Instance
static SafetyKernel_t Kernel = {0};

/* --- Non-Blocking Hardware Drivers (HAL) --- */

void HAL_ControlRelay(bool close_circuit) {
    Kernel.relay_closed = close_circuit;
    digitalWrite(PIN_RELAY, close_circuit ? HIGH : LOW);
}

void HAL_UpdateBuzzerNonBlocking(KernelState_t state, uint32_t current_time) {
    static uint32_t last_toggle = 0;
    static bool toggle_state = false;

    switch (state) {
        case KERNEL_NORMAL:
            digitalWrite(PIN_BUZZER, LOW);
            break;
            
        case KERNEL_WARNING:
            // Pulse buzzer every 500ms
            if (current_time - last_toggle >= 500) {
                toggle_state = !toggle_state;
                digitalWrite(PIN_BUZZER, toggle_state ? HIGH : LOW);
                last_toggle = current_time;
            }
            break;
            
        case KERNEL_TRIPPED:
            // Continuous High-Alert tone
            digitalWrite(PIN_BUZZER, HIGH);
            break;
    }
}

void HAL_UpdateDisplayNonBlocking(const SafetyKernel_t *kernel_ptr, uint32_t current_time) {
    static uint32_t last_display_update = 0;
    // Throttled presentation rate to prevent serial bus choking
    if (current_time - last_display_update < 500) return; 
    last_display_update = current_time;

    Serial.println("\n--- [SAFETY KERNEL TELEMETRY] ---");
    Serial.printf("State: %s | Relay: %s\n", 
                  (kernel_ptr->current_state == KERNEL_NORMAL) ? "NORMAL" :
                  (kernel_ptr->current_state == KERNEL_WARNING) ? "WARNING" : "TRIPPED!",
                  kernel_ptr->relay_closed ? "CLOSED (ENGAGED)" : "OPEN (ISOLATED)");
    
    Serial.printf("Fault Mask: 0x%02X\n", kernel_ptr->active_fault_mask);
    if (kernel_ptr->active_fault_mask & FAULT_UNDER_VOLTAGE)  Serial.println("  -> ERROR: UNDER VOLTAGE DETECTED");
    if (kernel_ptr->active_fault_mask & FAULT_OVER_VOLTAGE)   Serial.println("  -> ERROR: OVER VOLTAGE DETECTED");
    if (kernel_ptr->active_fault_mask & FAULT_RAPID_FLUC)     Serial.println("  -> ERROR: TRANSIENT VOLTAGE FLUCTUATION");
    if (kernel_ptr->active_fault_mask & FAULT_SENSOR_ANOMALY) Serial.println("  -> ERROR: SENSOR DISCONNECTED/INVALID");
}

/* --- Core Asynchronous Logic Engine --- */

void Safety_ReadTelemetry(SafetyKernel_t *kernel_ptr) {
    for (int i = 0; i < NUM_CELLS; i++) {
        kernel_ptr->previous_voltages[i] = kernel_ptr->cell_voltages[i];
        
        // Mocking analog reads. For hardware conversion: 
        // (analogRead(pin) * 3.3 / 4095.0) * ScaleFactor
        // Emulating a mild sensor open-circuit drop on Cell 2 for verification
        if (millis() > 15000 && millis() < 22000) {
            kernel_ptr->cell_voltages[0] = 3.30f;
            kernel_ptr->cell_voltages[1] = 0.40f; // Sensor failure drop simulation
            kernel_ptr->cell_voltages[2] = 3.32f;
            kernel_ptr->cell_voltages[3] = 3.29f;
        } else {
            kernel_ptr->cell_voltages[0] = 3.30f;
            kernel_ptr->cell_voltages[1] = 3.28f;
            kernel_ptr->cell_voltages[2] = 3.32f;
            kernel_ptr->cell_voltages[3] = 3.29f;
        }
    }
}

void Safety_EvaluateFaults(SafetyKernel_t *kernel_ptr) {
    uint8_t current_faults = FAULT_NONE;

    for (int i = 0; i < NUM_CELLS; i++) {
        float v = kernel_ptr->cell_voltages[i];
        float prev_v = kernel_ptr->previous_voltages[i];

        // 1. Physical Sensor Anomaly Boundary Checks
        if (v < 0.5f || v > 5.0f) {
            current_faults |= FAULT_SENSOR_ANOMALY;
        }
        // 2. Overvoltage / Under-voltage Checks
        if (v >= V_WARNING_OVER || v <= V_WARNING_UNDER) {
            if (v >= V_CRITICAL_OVER || v <= V_CRITICAL_UNDER) {
                current_faults |= (v >= V_CRITICAL_OVER) ? FAULT_OVER_VOLTAGE : FAULT_UNDER_VOLTAGE;
            } else {
                current_faults |= (v >= V_WARNING_OVER) ? FAULT_OVER_VOLTAGE : FAULT_UNDER_VOLTAGE;
            }
        }
        // 3. Transient Fluctuation Rate Delta Check ($dV/dt$ Check)
        if (prev_v > 0.1f && abs(v - prev_v) > MAX_VOLTAGE_DELTA) {
            current_faults |= FAULT_RAPID_FLUC;
        }
    }

    kernel_ptr->active_fault_mask = current_faults;
}

void Safety_ExecuteStateMachine(SafetyKernel_t *kernel_ptr, uint32_t current_time) {
    bool faults_present = (kernel_ptr->active_fault_mask != FAULT_NONE);

    switch (kernel_ptr->current_state) {
        case KERNEL_NORMAL:
            if (faults_present) {
                kernel_ptr->fault_timestamp = current_time;
                kernel_ptr->current_state = KERNEL_WARNING;
                kernel_ptr->state_change_timestamp = current_time;
            }
            // Fail-safe layout: verify the relay remains forced shut during normal operation
            if (!kernel_ptr->relay_closed) {
                HAL_ControlRelay(true); 
            }
            break;

        case KERNEL_WARNING:
            if (!faults_present) {
                kernel_ptr->current_state = KERNEL_NORMAL;
                kernel_ptr->state_change_timestamp = current_time;
            } else {
                // Determine severity level. Critical faults or sustained warnings trigger immediate trip
                bool is_critical = (kernel_ptr->active_fault_mask & (FAULT_SENSOR_ANOMALY | FAULT_RAPID_FLUC)) ||
                                   (kernel_ptr->cell_voltages[0] >= V_CRITICAL_OVER || kernel_ptr->cell_voltages[0] <= V_CRITICAL_UNDER); // generic loop index reduction alternative

                if (is_critical || (current_time - kernel_ptr->fault_timestamp >= DEBOUNCE_MS)) {
                    HAL_ControlRelay(false); // OPEN RELAY IMMEDIATELY
                    kernel_ptr->current_state = KERNEL_TRIPPED;
                    kernel_ptr->state_change_timestamp = current_time;
                }
            }
            break;

        case KERNEL_TRIPPED:
            // Anti-Relay Chatter Check: System must lock down until Cooldown window expires
            if (current_time - kernel_ptr->state_change_timestamp >= COOLDOWN_MS) {
                // If hazards are completely mitigated, clear the isolation trip
                if (!faults_present) {
                    Serial.println("[SYSTEM] Safety conditions restored. Re-engaging systems...");
                    HAL_ControlRelay(true); 
                    kernel_ptr->current_state = KERNEL_NORMAL;
                    kernel_ptr->state_change_timestamp = current_time;
                }
            }
            break;
    }
}

/* --- Framework Standard Vectors --- */

void setup() {
    Serial.begin(115200);
    pinMode(PIN_RELAY, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);

    // Initial safe defaults configuration
    HAL_ControlRelay(true); 
    Kernel.current_state = KERNEL_NORMAL;
    
    Serial.println("[KERNEL] Event-Driven Protection Core Live.");
}

void loop() {
    uint32_t current_time = millis();
    static uint32_t last_tick_time = 0;

    // Asynchronous Execution Ticker (Non-blocking loop separation)
    if (current_time - last_tick_time >= TICK_INTERVAL_MS) {
        last_tick_time = current_time;

        // 1. Data Ingestion Vector
        Safety_ReadTelemetry(&Kernel);

        // 2. Synchronous Matrix Fault Checking
        Safety_EvaluateFaults(&Kernel);

        // 3. State Machine Transition Execution Logic
        Safety_ExecuteStateMachine(&Kernel, current_time);
    }

    // 4. Fully decoupled async driver tasks running on unique time arrays
    HAL_UpdateBuzzerNonBlocking(Kernel.current_state, current_time);
    HAL_UpdateDisplayNonBlocking(&Kernel, current_time);
}