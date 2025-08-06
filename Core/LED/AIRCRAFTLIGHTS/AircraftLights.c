#include "AircraftLights.h"
#include "targets.h"
#include "cmsis_os.h"
#include "semphr.h"

// External handles
extern TIM_HandleTypeDef AIRCRAFTLIGHTS_TIMER;
extern DMA_HandleTypeDef AIRCRAFTLIGHTS_DMA;
extern SemaphoreHandle_t aircraftlights_dma_semaphore;

volatile uint8_t aircraftlights_data_sent_flag = 0;

// Double buffers to avoid flicker during updates
static uint32_t pwm_buffer_a[AIRCRAFT_BUFFER_SIZE];
static uint32_t pwm_buffer_b[AIRCRAFT_BUFFER_SIZE];
static uint32_t *active_buffer = pwm_buffer_a;
static uint32_t *prepare_buffer = pwm_buffer_b;

static uint8_t led_data[AIRCRAFT_LED_COUNT][3];
static float global_brightness = 1.0f;

#define WS2812_LATCH_DELAY_MS 1   // >50us required for WS2812 reset

// ----------------------------
// Helper: Swap Buffers Safely
// ----------------------------
static inline void swap_buffers(void) {
    uint32_t *temp = active_buffer;
    active_buffer = prepare_buffer;
    prepare_buffer = temp;
}

// ----------------------------
// Initialize (Clear LEDs)
// ----------------------------
void AIRCRAFTLIGHTS_Init(TIM_HandleTypeDef *htim) {
    for (uint32_t i = 0; i < AIRCRAFT_BUFFER_SIZE; i++) {
        pwm_buffer_a[i] = DUTY_RESET;
        pwm_buffer_b[i] = DUTY_RESET;
    }
}

// ----------------------------
// Global Brightness Control
// ----------------------------
void AIRCRAFTLIGHTS_SetGlobalBrightness(float brightness) {
    if (brightness < 0.0f) brightness = 0.0f;
    if (brightness > 1.0f) brightness = 1.0f;
    global_brightness = brightness;
}

// ----------------------------
// Set Color for Individual LED
// ----------------------------
void aircraftlights(uint32_t led_index, uint8_t red, uint8_t green, uint8_t blue, float brightness) {
    if (led_index < AIRCRAFT_LED_COUNT) {
        if (brightness < 0.0f) brightness = 0.0f;
        if (brightness > 1.0f) brightness = 1.0f;

        float total_brightness = brightness * global_brightness;
        led_data[led_index][0] = (uint8_t)(green * total_brightness); // WS2812 uses GRB
        led_data[led_index][1] = (uint8_t)(red   * total_brightness);
        led_data[led_index][2] = (uint8_t)(blue  * total_brightness);
    }
}

// ----------------------------
// Encode LED Data → PWM Buffer
// ----------------------------
static void encode_leds_to_pwm(uint32_t *buffer) {
    uint32_t buffer_index = 0;

    // Clear buffer
    for (uint32_t i = 0; i < AIRCRAFT_BUFFER_SIZE; i++) {
        buffer[i] = DUTY_RESET;
    }

    // Encode GRB
    for (uint32_t led = 0; led < AIRCRAFT_LED_COUNT; led++) {
        for (uint32_t color = 0; color < 3; color++) {
            for (int8_t bit = 7; bit >= 0; bit--) {
                buffer[buffer_index++] = (led_data[led][color] & (1 << bit)) ? DUTY_1 : DUTY_0;
            }
        }
    }
}

// ----------------------------
// Send Data via DMA (No Flicker)
// ----------------------------
void aircraftlights_update(void) {
    // Encode into the non-active buffer
    encode_leds_to_pwm(prepare_buffer);
    swap_buffers();

    // Disable interrupts briefly (timing critical)
    taskENTER_CRITICAL();
    __disable_irq();

    // Stop any ongoing DMA
    HAL_TIM_PWM_Stop_DMA(&AIRCRAFTLIGHTS_TIMER, AIRCRAFTLIGHTS_TIMER_TIMER_CHANNEL);
    HAL_TIM_Base_Stop(&AIRCRAFTLIGHTS_TIMER);

    // Re-enable DMA IRQ
    HAL_NVIC_SetPriority(AIRCRAFTLIGHTS_TIMER_DMA_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(AIRCRAFTLIGHTS_TIMER_DMA_IRQn);

    aircraftlights_data_sent_flag = 0;

    // Start DMA with timing protected
    HAL_TIM_Base_Start(&AIRCRAFTLIGHTS_TIMER);
    HAL_TIM_PWM_Start_DMA(&AIRCRAFTLIGHTS_TIMER, AIRCRAFTLIGHTS_TIMER_TIMER_CHANNEL,
                          active_buffer, AIRCRAFT_BUFFER_SIZE);

    // Re-enable IRQs immediately after DMA start
    __enable_irq();
    taskEXIT_CRITICAL();

    // Wait for DMA to complete (use semaphore or fallback)
    if (aircraftlights_dma_semaphore != NULL) {
        xSemaphoreTake(aircraftlights_dma_semaphore, pdMS_TO_TICKS(2));
    } else {
        while (!aircraftlights_data_sent_flag) {
            if (HAL_DMA_GetState(&AIRCRAFTLIGHTS_DMA) == HAL_DMA_STATE_READY) {
                aircraftlights_data_sent_flag = 1;
            }
        }
    }

    // WS2812 latch delay to ensure proper reset
    vTaskDelay(pdMS_TO_TICKS(WS2812_LATCH_DELAY_MS));
}

// ----------------------------
// DMA Complete ISR
// ----------------------------
void AircraftLights_PWM_Callback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == AIRCRAFTLIGHTS_TIMER.Instance) {
        HAL_TIM_PWM_Stop_DMA(&AIRCRAFTLIGHTS_TIMER, AIRCRAFTLIGHTS_TIMER_TIMER_CHANNEL);
        HAL_TIM_Base_Stop(&AIRCRAFTLIGHTS_TIMER);
        aircraftlights_data_sent_flag = 1;

        if (aircraftlights_dma_semaphore != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(aircraftlights_dma_semaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
