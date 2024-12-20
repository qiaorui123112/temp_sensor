#include "stm32f4xx.h"  // Include STM32F4 Standard Peripheral Library to access microcontroller peripherals and registers

// Definitions
#define TEMP_THRESHOLD 50  // Define the temperature threshold (50°C) for fire detection
#define TEMP_SENSOR_ADC_CHANNEL 0  // Define ADC Channel 0 for the temperature sensor connected to PA0

// Global Flag for High Temperature Detection
volatile uint8_t high_temp_detected = 0;  // Declare a volatile global flag to indicate high temperature detection
                                          // 0 = Normal temperature, 1 = High temperature detected

// Function prototypes for system clock configuration, GPIO initialization, ADC initialization, and temperature reading
void SystemClock_Config(void);
void GPIO_Init(void);
void ADC_Init(void);
float Read_Temperature(void);

int main(void) {
    float temperature;  // Declare a variable to store the temperature value

    // Initialize peripherals
    SystemClock_Config();  // Configure the system clock to use HSI (High-Speed Internal oscillator)
    GPIO_Init();           // Initialize GPIO for analog input (temperature sensor pin)
    ADC_Init();            // Initialize the ADC peripheral to read temperature sensor data

    while (1) {  // Infinite loop to continuously monitor temperature
        // Read temperature
        temperature = Read_Temperature();  // Call function to read temperature value from the ADC

        // Check for high temperature
        if (temperature >= TEMP_THRESHOLD) {  // If the temperature exceeds the defined threshold
            high_temp_detected = 1;  // Set the flag to indicate high temperature
        } else {
            high_temp_detected = 0;  // Reset the flag when the temperature is below the threshold
        }
    }
}

void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSION;  // Enable the High-Speed Internal (HSI) clock (16 MHz)
    while (!(RCC->CR & RCC_CR_HSIRDY));  // Wait until the HSI clock is ready (HSIRDY bit is set)

    RCC->CFGR |= RCC_CFGR_SW_HSI;  // Select HSI as the system clock source
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // Wait until HSI is confirmed as the system clock
}

void GPIO_Init(void) {
    // Enable GPIOA clock for PA0 (temperature sensor pin)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable the clock for GPIOA peripheral

    // Configure PA0 as analog input
    GPIOA->MODER |= (3U << (0 * 2));  // Set PA0 to analog mode by writing '11' to MODER[1:0]
}


void ADC_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;  // Enable the clock for ADC1 peripheral

    // Configure ADC
    ADC1->CR2 |= ADC_CR2_ADON;        // Turn ON the ADC by setting the ADON bit in CR2
    ADC1->SMPR2 |= ADC_SMPR2_SMP0;    // Set sampling time for ADC Channel 0 (longer sampling improves accuracy)
}

//Read temperature from ADC
float Read_Temperature(void) {
    uint16_t adc_value;  // Declare a variable to hold the ADC digital output value

    // Start ADC conversion on Channel 0
    ADC1->SQR3 = TEMP_SENSOR_ADC_CHANNEL;  // Select ADC Channel 0 by setting SQR3 register
    ADC1->CR2 |= ADC_CR2_SWSTART;          // Start the ADC conversion using software trigger (SWSTART bit)

    // Wait for conversion to complete
    while (!(ADC1->SR & ADC_SR_EOC));  // Wait until the End of Conversion (EOC) flag is set

    adc_value = ADC1->DR;  // Read the converted ADC value from the Data Register (DR)

    // Convert ADC value to temperature
    // LM35 outputs 10mV per °C, with Vref = 3.3V and 12-bit ADC resolution (0 - 4095)
    float temperature = ((adc_value * 3.3) / 4096) * 100;  // Calculate temperature in °C

    return temperature;  // Return the calculated temperature value
}
