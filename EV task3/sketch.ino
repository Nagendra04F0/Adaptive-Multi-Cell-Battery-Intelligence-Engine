#include <Arduino.h>

/* --- Architectural Configuration Constants --- */
#define PAGE_ROTATION_MS     4000  // Time each screen stays active during normal mode
#define DATA_REFRESH_MS      250   // How often variable text values refresh (4Hz)
#define NUM_CELLS            4

/* --- Mocking Shared Core Engine Framework Architectures --- */
typedef enum {
    HMI_NORMAL,
    HMI_WARNING,
    HMI_TRIPPED
} SafetyState_t;

typedef struct {
    float cell_voltages[NUM_CELLS];
    float pack_voltage;
    float avg_voltage;
    float imbalance_pct;
    uint8_t strongest_cell_idx;
    uint8_t weakest_cell_idx;
    uint8_t fault_mask;
    SafetyState_t system_safety;
} SharedTelemetry_t;

/* --- Mock LiquidCrystal Class to make it compilation ready --- */
class MockLCD {
public:
    void begin(int cols, int rows) { (void)cols; (void)rows; }
    void clear() { Serial.println("[LCD CLEAR]"); }
    void setCursor(int col, int row) { (void)col; (void)row; }
    void print(const char* text) { Serial.print(text); }
    void print(float val, int dec = 2) { Serial.print(val, dec); }
    void print(int val) { Serial.print(val); }
};
static MockLCD lcd; 

/* --- HMI State Structures --- */
typedef enum {
    PAGE_LIVE_VOLTAGE,
    PAGE_IMBALANCE_ANALYTICS,
    PAGE_PROTECTION_STATUS,
    PAGE_FAULT_OVERRIDE
} HMIPage_t;

typedef struct {
    HMIPage_t active_page;
    uint32_t last_page_switch;
    uint32_t last_data_refresh;
    bool force_frame_redraw;
} HMI_Engine_t;

// Global Instances
static HMI_Engine_t HmiCore = {PAGE_LIVE_VOLTAGE, 0, 0, true};
static SharedTelemetry_t TelemetryData; // To be updated by core engines

/* --- Hardware Abstraction Layer Rendering Engines --- */

void HMI_RenderStaticFrames(HMIPage_t page) {
    lcd.clear(); // Safe to execute ONLY during transitions to eliminate flicker
    
    switch (page) {
        case PAGE_LIVE_VOLTAGE:
            lcd.setCursor(0, 0); lcd.print("PACK VOLT: ");
            lcd.setCursor(0, 1); lcd.print("AVG CELL: ");
            break;
        case PAGE_IMBALANCE_ANALYTICS:
            lcd.setCursor(0, 0); lcd.print("IMBAL: %  ");
            lcd.setCursor(0, 1); lcd.print("S:C | W:C ");
            break;
        case PAGE_PROTECTION_STATUS:
            lcd.setCursor(0, 0); lcd.print("RELAY STATE: ");
            lcd.setCursor(0, 1); lcd.print("SYS STATUS: ");
            break;
        case PAGE_FAULT_OVERRIDE:
            lcd.setCursor(0, 0); lcd.print("!! CRIT FAULT !! ");
            lcd.setCursor(0, 1); lcd.print("CODE: 0x  ");
            break;
    }
}

void HMI_UpdateDynamicData(const SharedTelemetry_t *data, HMIPage_t page) {
    // Overwrites only specified character spans to keep updates seamless
    switch (page) {
        case PAGE_LIVE_VOLTAGE:
            lcd.setCursor(11, 0); lcd.print(data->pack_voltage, 2); lcd.print("V");
            lcd.setCursor(11, 1); lcd.print(data->avg_voltage, 2);  lcd.print("V");
            break;
            
        case PAGE_IMBALANCE_ANALYTICS:
            lcd.setCursor(7, 0);  lcd.print(data->imbalance_pct, 2);
            lcd.setCursor(3, 1);  lcd.print(data->strongest_cell_idx + 1);
            lcd.setCursor(11, 1); lcd.print(data->weakest_cell_idx + 1);
            break;
            
        case PAGE_PROTECTION_STATUS:
            lcd.setCursor(12, 0);
            lcd.print((data->system_safety == HMI_TRIPPED) ? "OPEN" : "CLSD");
            lcd.setCursor(12, 1);
            lcd.print((data->system_safety == HMI_NORMAL) ? "OK" : "WARN");
            break;
            
        case PAGE_FAULT_OVERRIDE:
            lcd.setCursor(8, 1);
            // Print hexadecimal fault payload
            if(data->fault_mask < 0x10) lcd.print("0");
            Serial.print(data->fault_mask, HEX); 
            break;
    }
    Serial.println(""); // Flush line buffer out to developer terminal
}

/* --- HMI Operational Controller --- */

void HMI_RunEngineCycle(const SharedTelemetry_t *data, uint32_t current_time) {
    HMIPage_t targeted_page = HmiCore.active_page;

    // 1. Priority Fault State Override Logic
    if (data->system_safety == HMI_TRIPPED) {
        if (HmiCore.active_page != PAGE_FAULT_OVERRIDE) {
            targeted_page = PAGE_FAULT_OVERRIDE;
            HmiCore.force_frame_redraw = true;
        }
    } else {
        // Safe execution loop: If clearing faults, kick out of priority display page
        if (HmiCore.active_page == PAGE_FAULT_OVERRIDE) {
            targeted_page = PAGE_LIVE_VOLTAGE;
            HmiCore.force_frame_redraw = true;
        }

        // 2. Normal Mode Sequence Rotator
        if (targeted_page != PAGE_FAULT_OVERRIDE) {
            if (current_time - HmiCore.last_page_switch >= PAGE_ROTATION_MS) {
                HmiCore.last_page_switch = current_time;
                
                // Mathematical indexing for step rotation [0 -> 1 -> 2 -> 0]
                targeted_page = (HMIPage_t)((int)HmiCore.active_page + 1);
                if (targeted_page >= PAGE_FAULT_OVERRIDE) { 
                    targeted_page = PAGE_LIVE_VOLTAGE; 
                }
                HmiCore.force_frame_redraw = true;
            }
        }
    }

    // 3. Execution Processing Core Vector
    if (HmiCore.force_frame_redraw) {
        HmiCore.active_page = targeted_page;
        HMI_RenderStaticFrames(HmiCore.active_page);
        HMI_UpdateDynamicData(data, HmiCore.active_page);
        HmiCore.force_frame_redraw = false;
        HmiCore.last_data_refresh = current_time;
    } 
    // Regular throttled window update
    else if (current_time - HmiCore.last_data_refresh >= DATA_REFRESH_MS) {
        HmiCore.last_data_refresh = current_time;
        HMI_UpdateDynamicData(data, HmiCore.active_page);
    }
}

/* --- Verification Mock Data Feed Harness --- */

void Mock_UpdateTelemetrySystem() {
    uint32_t current_timestamp = millis();
    
    // Simulate active system variables
    TelemetryData.cell_voltages[0] = 3.35f;
    TelemetryData.cell_voltages[1] = 3.31f;
    TelemetryData.cell_voltages[2] = 3.36f;
    TelemetryData.cell_voltages[3] = 3.34f;
    TelemetryData.pack_voltage = 13.36f;
    TelemetryData.avg_voltage = 3.34f;
    TelemetryData.imbalance_pct = 0.37f;
    TelemetryData.strongest_cell_idx = 2;
    TelemetryData.weakest_cell_idx = 1;

    // Simulate an emergency fault incident at the 10-second mark
    if (current_timestamp >= 10000 && current_timestamp <= 16000) {
        TelemetryData.system_safety = HMI_TRIPPED;
        TelemetryData.fault_mask = 0x04; // Fluctuation/Trip bitmask sample
    } else {
        TelemetryData.system_safety = HMI_NORMAL;
        TelemetryData.fault_mask = 0x00;
    }
}

/* --- Main Framework Setup Loop Links --- */

void setup() {
    Serial.begin(115200);
    lcd.begin(16, 2);
    Serial.println("[HMI CORE] Display Thread Interface Engine Active.");
}

void loop() {
    uint32_t current_time = millis();

    // 1. Ingest telemetry data arrays 
    Mock_UpdateTelemetrySystem();

    // 2. Feed updates to the asynchronous display engine
    HMI_RunEngineCycle(&TelemetryData, current_time);
}