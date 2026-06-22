#include "stm32f446xx.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>


#define ECU_MAX_SIM_RPM         2500U
#define ECU_IDLE_SIM_RPM        500U

#define ECU_MIN_FUEL_PW         2U
#define ECU_MAX_FUEL_PW         8U
#define ECU_MIN_ADVANCE         5U
#define ECU_MAX_ADVANCE         25U

/* Plausibility Bounds */
#define ECU_ADC_MIN_VALID       0U
#define ECU_ADC_MAX_VALID       4095U
#define ECU_ADC_MAX_RES         4095U


#define POT_ADC_MIN             0U
#define POT_ADC_MAX             4090U

/* Motor PWM Range — matched to TIM2 ARR=999 */
#define MOTOR_PWM_MAX           999U

/* ADC Filter */
#define ADC_FILTER_TAPS         8
volatile uint16_t adc_buffer[ADC_FILTER_TAPS] = {0};
volatile uint8_t  adc_filter_idx = 0;
volatile uint32_t adc_sum = 0;

/* ---------------------------------------------------------------------------
 * Hardware Pin Mappings
 * ---------------------------------------------------------------------------*/
#define INJ_LED_PIN             3       // PB3
#define IGN_LED_PIN             4       // PB4
#define MOTOR_IN1_PIN           5       // PB5: L298N IN1
#define MOTOR_IN2_PIN           8       // PA8: L298N IN2

/* NOTE ON PA15:
 * PA15 is JTDI by default. Set your debugger to SWD-only mode,
 * NOT JTAG, or TIM2_CH1 PWM will not appear on the L298N ENA pin.
 */

/* ---------------------------------------------------------------------------
 * Global State
 * ---------------------------------------------------------------------------*/
typedef enum { ENG_OFF, ENG_RUNNING, ENG_LIMP } EngineState_t;
volatile EngineState_t current_state = ENG_OFF;

volatile uint32_t adc_filtered   = 0;
volatile uint8_t  tps_percent    = 0;
volatile uint16_t sim_rpm        = 0;
volatile uint8_t  fuel_pw        = 0;
volatile uint8_t  ignition_adv   = 0;
volatile uint16_t motor_pwm      = 0;

volatile bool tps_fault           = false;
volatile bool adc_hardware_fault  = false;

volatile uint32_t sys_ticks       = 0;
uint32_t last_adc_read            = 0;
uint32_t last_math_calc           = 0;
uint32_t last_diag_tx             = 0;
uint32_t last_adc_success_time    = 0;

uint32_t engine_cycle_start  = 0;
uint32_t injector_off_time   = 0;
uint32_t ignition_on_time    = 0;
uint32_t ignition_off_time   = 0;
bool     injector_active     = false;
bool     ignition_active     = false;

#define UART_TX_BUF_SIZE 256
volatile char     uart_tx_buf[UART_TX_BUF_SIZE];
volatile uint16_t tx_head    = 0;
volatile uint16_t tx_tail    = 0;
volatile bool     tx_overflow = false;

/* ---------------------------------------------------------------------------
 * Prototypes
 * ---------------------------------------------------------------------------*/
void System_Hardware_Init(void);
void UART_Write_Queue(const char* str);
void Task_AcquireTPS(void);
void Task_CalculateECU(void);
void Task_UpdateActuators(void);
void Task_EngineScheduler(void);
void Task_SendDiagnostics(void);

/* ---------------------------------------------------------------------------
 * Interrupt Handlers
 * ---------------------------------------------------------------------------*/
void SysTick_Handler(void) { sys_ticks++; }

void USART2_IRQHandler(void) {
    if ((USART2->SR & USART_SR_TXE) && (USART2->CR1 & USART_CR1_TXEIE)) {
        if (tx_head != tx_tail) {
            USART2->DR = uart_tx_buf[tx_tail];
            tx_tail = (tx_tail + 1) % UART_TX_BUF_SIZE;
        } else {
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

/* ---------------------------------------------------------------------------
 * Main
 * ---------------------------------------------------------------------------*/
int main(void) {
    System_Hardware_Init();

    // L298N forward: IN1=High, IN2=Low
    GPIOB->BSRR = (1U << MOTOR_IN1_PIN);
    GPIOA->BSRR = (1U << (MOTOR_IN2_PIN + 16));

    // Prime ADC filter
    while (!(ADC1->SR & ADC_SR_EOC));
    uint32_t boot_prime_val = ADC1->DR;
    for (int i = 0; i < ADC_FILTER_TAPS; i++) adc_buffer[i] = boot_prime_val;
    adc_sum      = boot_prime_val * ADC_FILTER_TAPS;
    adc_filtered = boot_prime_val;

    last_adc_success_time = sys_ticks;
    last_adc_read         = sys_ticks;
    last_math_calc        = sys_ticks;
    last_diag_tx          = sys_ticks;

    UART_Write_Queue("\r\n=== ASEP V1 ECU PROTOTYPE BOOTED ===\r\n");
    UART_Write_Queue("POT FULL LEFT=MOTOR STOP | POT FULL RIGHT=FULL SPEED\r\n---\r\n");

    while (1) {
        uint32_t current_time = sys_ticks;

        // 10ms: ADC + actuator
        if (current_time - last_adc_read >= 10) {
            last_adc_read = current_time;
            Task_AcquireTPS();
            Task_UpdateActuators();
        }

        // 50ms: ECU math
        if (current_time - last_math_calc >= 50) {
            last_math_calc = current_time;
            Task_CalculateECU();
        }

        // 500ms: UART diagnostics
        if (current_time - last_diag_tx >= 500) {
            last_diag_tx = current_time;
            Task_SendDiagnostics();
        }

        Task_EngineScheduler();
    }
}

/* ---------------------------------------------------------------------------
 * Hardware Initialization
 * ---------------------------------------------------------------------------*/
void System_Hardware_Init(void) {
    // 1. PLL: HSI 16MHz -> 84MHz
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR      |= PWR_CR_VOS;
    FLASH->ACR   &= ~FLASH_ACR_LATENCY;
    FLASH->ACR   |= FLASH_ACR_LATENCY_2WS | FLASH_ACR_PRFTEN;
    RCC->CFGR    &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR    |= RCC_CFGR_PPRE1_DIV2;
    RCC->PLLCFGR  = (16 << RCC_PLLCFGR_PLLM_Pos) | (336 << RCC_PLLCFGR_PLLN_Pos) |
                    (1  << RCC_PLLCFGR_PLLP_Pos)  | RCC_PLLCFGR_PLLSRC_HSI;
    RCC->CR      |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR    &= ~RCC_CFGR_SW;
    RCC->CFGR    |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // 2. SysTick 1ms
    SysTick_Config(84000000 / 1000);

    // 3. GPIO clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // PA0 — Analog TPS input
    GPIOA->MODER |= (3U << (0 * 2));

    // PA15 — TIM2_CH1 PWM output (AF1) -> L298N ENA
    GPIOA->MODER  &= ~(3U << (15 * 2));
    GPIOA->MODER  |=  (2U << (15 * 2));
    GPIOA->AFR[1] &= ~(0xF << 28);
    GPIOA->AFR[1] |=  (1U  << 28);

    // PA2/PA3 — USART2 TX/RX (AF7)
    GPIOA->MODER  &= ~((3U << (2 * 2)) | (3U << (3 * 2)));
    GPIOA->MODER  |=   (2U << (2 * 2)) | (2U << (3 * 2));
    GPIOA->AFR[0] &= ~((0xF << 8) | (0xF << 12));
    GPIOA->AFR[0] |=   (7U  << 8)  | (7U  << 12);

    // PA8 — L298N IN2 (output)
    GPIOA->MODER &= ~(3U << (8 * 2));
    GPIOA->MODER |=  (1U << (8 * 2));

    // PB3/PB4/PB5 — Injector LED / Ignition LED / L298N IN1 (outputs)
    GPIOB->MODER &= ~((3U << (3 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2)));
    GPIOB->MODER |=   (1U << (3 * 2)) | (1U << (4 * 2)) | (1U << (5 * 2));

    // 4. ADC1 — continuous on CH0 (PA0)
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC->CCR     |= ADC_CCR_ADCPRE_0;
    ADC1->SMPR2  |= ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_2;
    ADC1->CR2    |= ADC_CR2_CONT;
    ADC1->SQR3    = 0;
    ADC1->CR2    |= ADC_CR2_ADON;
    ADC1->CR2    |= ADC_CR2_SWSTART;

    // 5. TIM2 — 8kHz PWM on CH1 (PA15) — higher freq for smoother high-speed torque
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC     = 9;      // 84MHz / 10 = 8.4MHz timer clock
    TIM2->ARR     = 999;    // 8.4MHz / 1000 = 8.4kHz PWM
    TIM2->CCMR1  &= ~TIM_CCMR1_OC1M;
    TIM2->CCMR1  |=  TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
    TIM2->CCER   &= ~TIM_CCER_CC1P;
    TIM2->CCER   |=  TIM_CCER_CC1E;
    TIM2->CCR1    = 0;
    TIM2->EGR    |= TIM_EGR_UG;
    TIM2->CR1    |= TIM_CR1_CEN;

    // 6. USART2 — 115200 baud on 42MHz APB1
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR   = 365;
    USART2->CR1  |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    NVIC_EnableIRQ(USART2_IRQn);
}

void UART_Write_Queue(const char* str) {
    while (*str) {
        uint16_t next_head = (tx_head + 1) % UART_TX_BUF_SIZE;
        if (next_head != tx_tail) {
            uart_tx_buf[tx_head] = *str;
            tx_head = next_head;
        } else {
            tx_overflow = true;
            break;
        }
        str++;
    }
    USART2->CR1 |= USART_CR1_TXEIE;
}

/* ---------------------------------------------------------------------------
 * Tasks
 * ---------------------------------------------------------------------------*/
void Task_AcquireTPS(void) {
    uint32_t current_time = sys_ticks;

    if (ADC1->SR & ADC_SR_EOC) {
        last_adc_success_time = current_time;
        adc_hardware_fault    = false;

        // Moving average filter
        adc_sum -= adc_buffer[adc_filter_idx];
        adc_buffer[adc_filter_idx] = ADC1->DR;
        adc_sum += adc_buffer[adc_filter_idx];
        adc_filter_idx = (adc_filter_idx + 1) % ADC_FILTER_TAPS;
        adc_filtered   = adc_sum / ADC_FILTER_TAPS;

        // No plausibility rejection at extremes — your pot reaches 0 and 4090+
        tps_fault = false;

        // Calibrated stretch: 0..4090 -> 0..100%
        if (adc_filtered <= POT_ADC_MIN) {
            tps_percent = 0;
        } else if (adc_filtered >= POT_ADC_MAX) {
            tps_percent = 100;
        } else {
            tps_percent = (uint8_t)(((adc_filtered - POT_ADC_MIN) * 100UL)
                          / (POT_ADC_MAX - POT_ADC_MIN));
        }

        // Engine is always RUNNING in V1 (no key/crank input exists yet).
        // tps_percent=0 just means "idle/motor stopped", not "engine off".
        // This prevents the motor getting stuck at PWM=0 near pot rest position.
        current_state = ENG_RUNNING;
    }

    // ADC watchdog: limp if no conversion for 100ms
    if (current_time - last_adc_success_time > 100) {
        adc_hardware_fault = true;
        current_state      = ENG_LIMP;
    }
}

void Task_CalculateECU(void) {
    if (current_state == ENG_RUNNING) {
        sim_rpm      = ECU_IDLE_SIM_RPM + ((tps_percent * (ECU_MAX_SIM_RPM - ECU_IDLE_SIM_RPM)) / 100);
        fuel_pw      = ECU_MIN_FUEL_PW  + ((tps_percent * (ECU_MAX_FUEL_PW  - ECU_MIN_FUEL_PW))  / 100);
        ignition_adv = ECU_MIN_ADVANCE  + ((tps_percent * (ECU_MAX_ADVANCE  - ECU_MIN_ADVANCE))  / 100);

        // Squared response curve: tps 0% = PWM 0 (fully stopped),
        // tps 100% = PWM 999 (max speed). Curve stays gentle through the
        // low/mid range then rockets up hard near full pot travel —
        // gives a "very fast at the end" feel instead of flat linear.
        motor_pwm = (uint16_t)(((uint32_t)tps_percent * tps_percent * MOTOR_PWM_MAX) / 10000);

    } else if (current_state == ENG_LIMP) {
        sim_rpm      = ECU_IDLE_SIM_RPM;
        fuel_pw      = ECU_MIN_FUEL_PW + 1;
        ignition_adv = ECU_MIN_ADVANCE;
        motor_pwm    = 0;

    } else {
        sim_rpm      = 0;
        fuel_pw      = 0;
        ignition_adv = 0;
        motor_pwm    = 0;
    }
}

void Task_UpdateActuators(void) {
    TIM2->CCR1 = motor_pwm;
}

void Task_EngineScheduler(void) {
    uint32_t current_time = sys_ticks;

    if ((current_state == ENG_RUNNING || current_state == ENG_LIMP) && sim_rpm > 0) {
        uint32_t cycle_duration_ms = 60000 / sim_rpm;

        if (current_time - engine_cycle_start >= cycle_duration_ms) {
            engine_cycle_start = current_time;

            GPIOB->BSRR   = (1U << INJ_LED_PIN);
            injector_active   = true;
            injector_off_time = current_time + fuel_pw;

            uint32_t tdc_time_ms = cycle_duration_ms / 2;
            uint32_t advance_ms  = (ignition_adv * cycle_duration_ms) / 360;
            ignition_on_time  = (tdc_time_ms > advance_ms)
                                 ? current_time + tdc_time_ms - advance_ms
                                 : current_time;
            ignition_off_time = ignition_on_time + 2;
        }

        if (injector_active && (current_time >= injector_off_time)) {
            GPIOB->BSRR   = (1U << (INJ_LED_PIN + 16));
            injector_active = false;
        }

        if (!ignition_active && (current_time >= ignition_on_time) && (current_time < ignition_off_time)) {
            GPIOB->BSRR   = (1U << IGN_LED_PIN);
            ignition_active = true;
        }

        if (ignition_active && (current_time >= ignition_off_time)) {
            GPIOB->BSRR   = (1U << (IGN_LED_PIN + 16));
            ignition_active = false;
        }

    } else {
        GPIOB->BSRR = (1U << (INJ_LED_PIN  + 16));
        GPIOB->BSRR = (1U << (IGN_LED_PIN  + 16));
        injector_active = false;
        ignition_active = false;
    }
}

void Task_SendDiagnostics(void) {
    char diag_buf[160];

    if (tx_overflow) {
        UART_Write_Queue("[UART OVERFLOW - DATA DROP WARNING]\r\n");
        tx_overflow = false;
    }

    const char* state_str = (current_state == ENG_RUNNING) ? "RUNNING" :
                            (current_state == ENG_LIMP)    ? "LIMP"    : "OFF";
    const char* fault_str = adc_hardware_fault ? "YES" : "NO";

    snprintf(diag_buf, sizeof(diag_buf),
             "STATE=%s\r\nTPS_RAW=%lu\r\nTPS=%u%%\r\nRPM=%u\r\nFUEL_PW=%ums\r\nIGN_ADV=%udeg\r\nPWM=%u\r\nFAULT=%s\r\n---\r\n",
             state_str, adc_filtered, tps_percent, sim_rpm, fuel_pw, ignition_adv, motor_pwm, fault_str);

    UART_Write_Queue(diag_buf);
}

