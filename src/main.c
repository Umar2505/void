
#include <stdio.h>
#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <string.h>
#include <tkjhat/sdk.h>

// Default stack size for the tasks
#define DEFAULT_STACK_SIZE 2048

// Morse code configuration
#define DOT_DURATION_MS     200
#define DASH_DURATION_MS    600
#define TONE_FREQUENCY      800

// IMU thresholds for detecting tilt
#define TILT_THRESHOLD_DOT   0.3f  // Tilt forward for dot
#define TILT_THRESHOLD_DASH -0.3f  // Tilt backward for dash

// Maximum message length
#define MAX_MESSAGE_LENGTH 256

// Morse code states
enum state {
    IDLE = 0,
    RECORDING,
    SENDING
};

// Global variables
static enum state programState = IDLE;
static char dMessage[MAX_MESSAGE_LENGTH];
static char mMessage[MAX_MESSAGE_LENGTH*4]; // Morse message can be longer
static uint16_t dMesI = 0;
static uint16_t mMesI = 0;
static volatile bool btn1_pressed = false;
static volatile bool btn2_pressed = false;
static volatile bool btn1_last_state = false;
static volatile bool btn2_last_state = false;

// Morse code lookup table (A-Z, 0-9)
typedef struct {
    char character;
    const char* code;
} MorseCode;

static const MorseCode morseTable[] = {
    {'A', ".-"},    {'B', "-..."},  {'C', "-.-."},  {'D', "-.."},
    {'E', "."},     {'F', "..-."},  {'G', "--."},   {'H', "...."},
    {'I', ".."},    {'J', ".---"},  {'K', "-.-"},   {'L', ".-.."},
    {'M', "--"},    {'N', "-."},    {'O', "---"},   {'P', ".--."},
    {'Q', "--.-"},  {'R', ".-."},   {'S', "..."},   {'T', "-"},
    {'U', "..-"},   {'V', "...-"},  {'W', ".--"},   {'X', "-..-"},
    {'Y', "-.--"},  {'Z', "--.."},
    {'0', "-----"}, {'1', ".----"}, {'2', "..---"}, {'3', "...--"},
    {'4', "....-"}, {'5', "....."}, {'6', "-...."}, {'7', "--..."},
    {'8', "---.."}, {'9', "----."},
    {' ', "/"}  // Space character
};

/**
 * @brief Convert morse code string to character
 */
static char morse_to_char(const char* morse) {
    if (morse[0] == '/') return ' ';
    
    for (int i = 0; i < sizeof(morseTable) / sizeof(MorseCode); i++) {
        if (strcmp(morse, morseTable[i].code) == 0) {
            return morseTable[i].character;
        }
    }
    return '?'; // Unknown morse code
}

/**
 * @brief Button monitoring task
 */
static void button_task(void *arg) {
    (void)arg;
    
    for (;;) {
        bool btn1_current = gpio_get(SW1_PIN);
        bool btn2_current = gpio_get(SW2_PIN);
        
        // Detect button 1 press (rising edge)
        if (btn1_current && !btn1_last_state) {
            btn1_pressed = true;
        }
        btn1_last_state = btn1_current;
        
        // Detect button 2 press (rising edge)
        if (btn2_current && !btn2_last_state) {
            btn2_pressed = true;
        }
        btn2_last_state = btn2_current;
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void imu_task(void *pvParameters) {
    (void)pvParameters;
    
    float ax, ay, az, gx, gy, gz, t;
    char currentMorse[10] = "";
    uint8_t morsePos = 0;
    // Setting up the sensor. 
    if (init_ICM42670() == 0) {
        printf("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            printf("ICM-42670P could not initialize accelerometer or gyroscope");
        }
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }
    // Start collection data here. Infinite loop. 
    while (1) {
        if (btn1_pressed) {
            btn1_pressed = false;
            
            if (programState == IDLE) {
                // Start recording
                programState = RECORDING;
                dMesI = 0;
                memset(dMessage, 0, MAX_MESSAGE_LENGTH);
                memset(mMessage, 0, MAX_MESSAGE_LENGTH);
                memset(currentMorse, 0, sizeof(currentMorse));
                morsePos = 0;
                printf("\n=== RECORDING STARTED ===\n");
                printf("Tilt down for DOT, up for DASH\n");
                printf("Press Button 2 for SPACE between letters\n");
                printf("Press Button 1 again to SEND message\n\n");
                
                // Blink LED to indicate recording started
                for (int i = 0; i < 3; i++) {
                    set_led_status(true);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    set_led_status(false);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
            else if (programState == RECORDING) {
                // Finish current character if any
                if (morsePos > 0) {
                    currentMorse[morsePos] = '\0';
                    char decoded = morse_to_char(currentMorse);
                    if (dMesI < MAX_MESSAGE_LENGTH - 1) {
                        dMessage[dMesI++] = decoded;
                    }
                    memset(currentMorse, 0, sizeof(currentMorse));
                    morsePos = 0;
                }
                
                // Stop recording and send message
                programState = SENDING;
                dMessage[dMesI] = '\0';
                mMessage[mMesI] = '\r\n'; // End of morse message

                printf("\n=== RECORDING STOPPED ===\n");
                printf("Morse message: %s\n", mMessage);
                printf("Decoded message: %s\n", dMessage);
                printf("Sending via USB...\n");
                
                // Long beep to indicate sending
                buzzer_play_tone(TONE_FREQUENCY, 1000);
                
                programState = IDLE;
                printf("\n=== Ready for new message ===\n\n");
            }
        }
        
        // Handle button 2 press - space between letters
        if (btn2_pressed && programState == RECORDING) {
            btn2_pressed = false;
            
            if (morsePos > 0) {
                // Convert current morse code to character
                currentMorse[morsePos] = '\0';
                char decoded = morse_to_char(currentMorse);
                
                if (dMesI < MAX_MESSAGE_LENGTH - 1) {
                    dMessage[dMesI++] = decoded;
                    printf("Letter: %c (Morse: %s)\n", decoded, currentMorse);
                }
                memcpy(&mMessage[mMesI], currentMorse, morsePos);
                mMesI += morsePos;
                mMessage[mMesI++] = ' '; // Add space in morse message


                // Reset for next character
                memset(currentMorse, 0, sizeof(currentMorse));
                morsePos = 0;
                
                // Short beep for space
                buzzer_play_tone(TONE_FREQUENCY / 2, 100);
            }
        }
        
        // Read IMU only when recording
        if (programState == RECORDING) {
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                // Check for dot (tilt forward - positive ay)
                if (ay > TILT_THRESHOLD_DOT && morsePos < 9) {
                    currentMorse[morsePos++] = '.';
                    printf(".");
                    fflush(stdout);
                    
                    // Play dot sound
                    buzzer_play_tone(TONE_FREQUENCY, DOT_DURATION_MS);
                    set_led_status(true);
                    vTaskDelay(pdMS_TO_TICKS(DOT_DURATION_MS));
                    set_led_status(false);
                    
                    // Wait for neutral position
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                // Check for dash (tilt backward - negative ay)
                else if (ay < TILT_THRESHOLD_DASH && morsePos < 9) {
                    currentMorse[morsePos++] = '-';
                    printf("-");
                    fflush(stdout);
                    
                    // Play dash sound
                    buzzer_play_tone(TONE_FREQUENCY, DASH_DURATION_MS);
                    set_led_status(true);
                    vTaskDelay(pdMS_TO_TICKS(DASH_DURATION_MS));
                    set_led_status(false);
                    
                    // Wait for neutral position
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main() {
    stdio_init_all();
    // Uncomment this lines if you want to wait till the serial monitor is connected
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.

    init_led();
    init_sw1();
    init_sw2();
    init_buzzer();

    printf("\n");
    printf("=====================================\n");
    printf("   MORSE CODE IMU TRANSLATOR\n");
    printf("=====================================\n");
    printf("\nInitializing...\n");

    printf("\n=== System Ready ===\n");
    printf("Press Button 1 to START recording\n");
    printf("Press Button 2 for SPACE between letters\n");
    printf("Press Button 1 again to SEND message\n\n");
    
    // Create tasks
    TaskHandle_t ButtonTask = NULL;
    TaskHandle_t IMUTask = NULL;

    BaseType_t result1 = xTaskCreate(
        button_task,
        "ButtonTask",
        DEFAULT_STACK_SIZE,
        NULL,
        3,
        &ButtonTask
    );
    
    BaseType_t result2 = xTaskCreate(
        imu_task,
        "IMUTask",
        DEFAULT_STACK_SIZE,
        NULL,
        2,
        &IMUTask
    );
    
    if (result1 != pdPASS || result2 != pdPASS) {
        printf("ERROR: Task creation failed\n");
        return -1;
    }

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    return 0;
}

