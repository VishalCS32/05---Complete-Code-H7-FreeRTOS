#include "ws2812.h"
#include "targets.h"
#include "cmsis_os.h"
#include "semphr.h"

// External handles
extern TIM_HandleTypeDef WS2812_TIMER;
extern DMA_HandleTypeDef WS2812_DMA;
extern SemaphoreHandle_t ws2812_dma_semaphore;

volatile uint8_t data_sent_flag = 0;
static uint32_t pwm_buffer[BUFFER_SIZE];
static uint8_t led_data[LED_COUNT][3];
static float global_brightness = 1.0;

void WS2812_Init(TIM_HandleTypeDef *htim) {
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
        pwm_buffer[i] = DUTY_RESET;
    }
}

void WS2812_SetGlobalBrightness(float brightness) {
    if (brightness < 0.0) brightness = 0.0;
    if (brightness > 1.0) brightness = 1.0;
    global_brightness = brightness;
}

void WS2812_SetColor(uint32_t led_index, uint8_t red, uint8_t green, uint8_t blue, float brightness) {
    if (led_index < LED_COUNT) {
        if (brightness < 0.0) brightness = 0.0;
        if (brightness > 1.0) brightness = 1.0;
        float total_brightness = brightness * global_brightness;
        led_data[led_index][0] = (uint8_t)(green * total_brightness);
        led_data[led_index][1] = (uint8_t)(red * total_brightness);
        led_data[led_index][2] = (uint8_t)(blue * total_brightness);
    }
}

void WS2812_Send(void) {
    uint32_t buffer_index = 0;

    // Fill reset first
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
        pwm_buffer[i] = DUTY_RESET;
    }

    // Encode GRB bits
    for (uint32_t led = 0; led < LED_COUNT; led++) {
        for (uint32_t color = 0; color < 3; color++) {
            for (int8_t bit = 7; bit >= 0; bit--) {
                pwm_buffer[buffer_index++] = (led_data[led][color] & (1 << bit)) ? DUTY_1 : DUTY_0;
            }
        }
    }

    // Stop old DMA
    HAL_TIM_PWM_Stop_DMA(&WS2812_TIMER, WS2812_TIMER_CHANNEL);
    HAL_TIM_Base_Stop(&WS2812_TIMER);

    // Ensure NVIC
    HAL_NVIC_SetPriority(WS2812_DMA_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(WS2812_DMA_IRQn);

    // Reset flag
    data_sent_flag = 0;

    // Start DMA transfer
    HAL_TIM_Base_Start(&WS2812_TIMER);
    HAL_TIM_PWM_Start_DMA(&WS2812_TIMER, WS2812_TIMER_CHANNEL, pwm_buffer, BUFFER_SIZE);
}

// Non-blocking main_led (works in FreeRTOS)
void main_led(uint32_t led_index, uint8_t red, uint8_t green, uint8_t blue, float brightness, uint32_t main_led_time) {
    WS2812_SetColor(led_index, red, green, blue, brightness);
    WS2812_Send();

    // Wait for DMA to finish using semaphore (if available)
    if (ws2812_dma_semaphore != NULL) {
        xSemaphoreTake(ws2812_dma_semaphore, pdMS_TO_TICKS(50));
    } else {
        // Fallback: Busy wait like original
        uint32_t start = HAL_GetTick();
        while (!data_sent_flag && (HAL_GetTick() - start) < 100) {
            if (HAL_DMA_GetState(&WS2812_DMA) == HAL_DMA_STATE_READY) {
                data_sent_flag = 1;
            }
        }
    }

    // Keeping LED ON for requested duration
    if (main_led_time > 0) {
        vTaskDelay(pdMS_TO_TICKS(main_led_time));
    }
}
