#include <Arduino.h>

#define NUM_CELLS             4
#define ADC_MAX_VALUE         4095
#define ADC_REF_VOLTAGE       3.3f
#define VOLTAGE_DIVIDER_RATIO 5.0f 

// Thresholds for Cell Health Classification
#define CELL_MIN_SAFE_V       2.7f  
#define CELL_MAX_SAFE_V       4.25f 
#define IMBALANCE_MINOR       2.0f  
#define IMBALANCE_CRITICAL    5.0f  

// ESP32 ADC Pins for the 4 cell stack nodes
const int ADC_PINS[NUM_CELLS] = {32, 33, 34, 35}; 

// Low-Pass Filter Constant (Alpha: 0.0 to 1.0) -> Lower = smoother but slower response
const float FILTER_ALPHA = 0.2f; 

typedef enum {
    PACK_STATE_HEALTHY,
    PACK_STATE_MINOR_IMBALANCE,
    PACK_STATE_CRITICAL_IMBALANCE,
    PACK_STATE_FAILURE
} PackHealthState_t;

typedef struct {
    float node_voltages[NUM_CELLS];
    float cell_voltages[NUM_CELLS];
    float pack_voltage;
    float avg_voltage;
    float imbalance_pct;
    uint8_t strongest_cell_idx;
    uint8_t weakest_cell_idx;
    PackHealthState_t health_state;
} BatteryPack_t;

// Global System Instances
BatteryPack_t pack_instance = {0};
float filtered_adc[NUM_CELLS] = {0.0f};

/* --- Hardware Abstraction Layer (HAL) --- */

void HAL_ADC_Init() {
    for (int i = 0; i < NUM_CELLS; i++) {
        pinMode(ADC_PINS[i], INPUT);
    }
    // Configure ESP32 ADC attenuation for up to ~3.1V range safely
    analogSetAttenuation(ADC_11db); 
}

void HAL_ADC_ReadNodes(float *filtered_out) {
    for (int i = 0; i < NUM_CELLS; i++) {
        int raw_reading = analogRead(ADC_PINS[i]);
        
        // Exponential Moving Average (EMA) Noise Filter
        filtered_out[i] = (FILTER_ALPHA * (float)raw_reading) + ((1.0f - FILTER_ALPHA) * filtered_out[i]);
    }
}

/* --- Core Battery Intelligence Engine --- */

void Battery_ConvertADC(const float *filtered_adc_in, BatteryPack_t *pack) {
    for (int i = 0; i < NUM_CELLS; i++) {
        // Convert filtered ADC steps back to real-world analog voltage metrics
        pack->node_voltages[i] = (filtered_adc_in[i] * ADC_REF_VOLTAGE / (float)ADC_MAX_VALUE) * VOLTAGE_DIVIDER_RATIO;
    }
}

void Battery_CalculateMetrics(BatteryPack_t *pack) {
    float max_v = -100.0f;
    float min_v = 100.0f;
    
    // 1. Calculate Individual Differential Cell Voltages
    pack->cell_voltages[0] = pack->node_voltages[0];
    for (int i = 1; i < NUM_CELLS; i++) {
        pack->cell_voltages[i] = pack->node_voltages[i] - pack->node_voltages[i - 1];
    }

    // 2. Compute Analytics (Pack Total, Min/Max Indexes)
    pack->pack_voltage = 0.0f;
    for (uint8_t i = 0; i < NUM_CELLS; i++) {
        float v = pack->cell_voltages[i];
        pack->pack_voltage += v;

        if (v > max_v) {
            max_v = v;
            pack->strongest_cell_idx = i;
        }
        if (v < min_v) {
            min_v = v;
            pack->weakest_cell_idx = i;
        }
    }

    pack->avg_voltage = pack->pack_voltage / NUM_CELLS;

    // 3. Compute Imbalance Percentage
    if (pack->pack_voltage > 0.1f) {
        pack->imbalance_pct = ((max_v - min_v) / pack->pack_voltage) * 100.0f;
    } else {
        pack->imbalance_pct = 0.0f;
    }
}

void Battery_ClassifyHealth(BatteryPack_t *pack) {
    bool has_fault = false;

    // Check for physical safety violations (Over/Under voltage checks)
    for (int i = 0; i < NUM_CELLS; i++) {
        if (pack->cell_voltages[i] < CELL_MIN_SAFE_V || pack->cell_voltages[i] > CELL_MAX_SAFE_V) {
            has_fault = true;
            break;
        }
    }

    if (has_fault) {
        pack->health_state = PACK_STATE_FAILURE;
    } else if (pack->imbalance_pct >= IMBALANCE_CRITICAL) {
        pack->health_state = PACK_STATE_CRITICAL_IMBALANCE;
    } else if (pack->imbalance_pct >= IMBALANCE_MINOR) {
        pack->health_state = PACK_STATE_MINOR_IMBALANCE;
    } else {
        pack->health_state = PACK_STATE_HEALTHY;
    }
}

/* --- Telemetry Presentation Layer --- */

void Display_SystemStatus(const BatteryPack_t *pack) {
    const char* state_strings[] = {
        "HEALTHY", 
        "MINOR IMBALANCE", 
        "CRITICAL IMBALANCE", 
        "PACK FAILURE (FAULT)"
    };

    Serial.println("\n==================================================");
    Serial.println("        BATTERY INTELLIGENCE ENGINE REPORT        ");
    Serial.println("==================================================");
    Serial.printf("Pack State        : %s\n", state_strings[pack->health_state]);
    Serial.printf("Total Pack Voltage: %.3f V\n", pack->pack_voltage);
    Serial.printf("Average Cell Volt : %.3f V\n", pack->avg_voltage);
    Serial.printf("Cell Imbalance    : %.2f %%\n", pack->imbalance_pct);
    Serial.println("--------------------------------------------------");
    
    for(int i = 0; i < NUM_CELLS; i++) {
        Serial.printf("  Cell [%d] Voltage: %.3f V\n", i + 1, pack->cell_voltages[i]);
    }
    Serial.println("--------------------------------------------------");
    Serial.printf("Strongest Cell   : Cell [%d] (%.3f V)\n", pack->strongest_cell_idx + 1, pack->cell_voltages[pack->strongest_cell_idx]);
    Serial.printf("Weakest Cell     : Cell [%d] (%.3f V)\n", pack->weakest_cell_idx + 1, pack->cell_voltages[pack->weakest_cell_idx]);
    Serial.println("==================================================");
}

/* --- Arduino Core Execution Entry Points --- */

void setup() {
    Serial.begin(115200);
    delay(1000); // Warmup pause for UART stability
    Serial.println("[SYSTEM] Initializing Battery Intelligence Engine...");
    
    HAL_ADC_Init();
    
    // Seed our filter buffer with initial baseline readings
    for (int i = 0; i < NUM_CELLS; i++) {
        filtered_adc[i] = (float)analogRead(ADC_PINS[i]);
    }
    Serial.println("[SYSTEM] Initialization Complete.");
}

void loop() {
    // 1. Telemetry streaming collection
    HAL_ADC_ReadNodes(filtered_adc);

    // 2. Data transformation to architectural domains
    Battery_ConvertADC(filtered_adc, &pack_instance);

    // 3. Mathematical matrix processing 
    Battery_CalculateMetrics(&pack_instance);

    // 4. Runtime diagnostics state machine logic
    Battery_ClassifyHealth(&pack_instance);

    // 5. Output Telematics to Serial Console
    Display_SystemStatus(&pack_instance);

    // Run evaluations at 1Hz (Every 1000ms)
    delay(2000); 
}