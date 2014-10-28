#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f1/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/scb.h>

// Test program to run a bipolar stepper motor with a smooth sine voltage,
// using a L293D as driver.
// (Note: It didn't work so far - PWM generation seems okay, the motor turns,
//        but I didn't manage to get smooth steps out of the motor.)

#define DEBUG1_SET GPIO_BSRR(GPIOC) = GPIO11
#define DEBUG1_CLEAR GPIO_BSRR(GPIOC) = (GPIO11 << 16)

static void flash(void);

static void setup(void)
{
  SCB_VTOR = 0x08000000; // required when starting from STM32 built-in bootloader (e.g. BOOT0 pin was high during last reset)
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_GPIOD);
  rcc_periph_clock_enable(RCC_AFIO);  // alternate function I/O
  rcc_periph_clock_enable(RCC_TIM3);

  // remap timer3 to achieve that TIM3_CH4 can be 5V on Embedded Pi's D6 header (=PC9)
  gpio_primary_remap(0, AFIO_MAPR_TIM3_REMAP_FULL_REMAP); 		
  
  // PWM outputs
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO6); // timer3 PWM outputs
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7); // timer3 PWM outputs
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO8); // timer3 PWM outputs
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9); // timer3 PWM outputs
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO11); // osz. trigger
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12); // channel enable
  gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);  // channel enable
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13); // Embedded Pi LED: GPIO Bank B, no 13
  
  // enable chip
  gpio_set(GPIOC, GPIO12);
  gpio_set(GPIOD, GPIO2);
  
  // Timer setup
  timer_reset(TIM3);
  timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  
  timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
  timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
  timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
  timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM1);

  //timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_FORCE_LOW);
  //timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_FORCE_LOW);
  //timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_FORCE_LOW);
  //timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_FORCE_LOW);

  timer_enable_oc_output(TIM3, TIM_OC1);
  timer_enable_oc_output(TIM3, TIM_OC2);
  timer_enable_oc_output(TIM3, TIM_OC3);
  timer_enable_oc_output(TIM3, TIM_OC4);
  //timer_enable_break_main_output(TIM1); // only needed for advanced timers (e.g. TIM1)
  timer_set_prescaler(TIM3, 0);
  timer_set_period(TIM3, 72000000 / 32000); // 32kHz / 44.4us
  timer_set_oc_value(TIM3, TIM_OC1, 1000);
  timer_set_oc_value(TIM3, TIM_OC2, 1000);
  timer_set_oc_value(TIM3, TIM_OC3, 1000);
  timer_set_oc_value(TIM3, TIM_OC4, 1000);

  // "The update event is sent when the counter reaches the overflow
  // (or underflow when downcounting) and if the UDIS (= disable
  // update events) bit equals 0 in the TIMx_CR1 register."
  timer_enable_counter(TIM3);
  timer_enable_irq(TIM3, TIM_DIER_UIE);
  nvic_enable_irq(NVIC_TIM3_IRQ);

  /*
  // exti test (it works; not used right now)
  nvic_enable_irq(NVIC_EXTI0_IRQ);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);
  exti_select_source(EXTI0, GPIOA);
  exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
  exti_enable_request(EXTI0);
  */
}

/*
//exti test
void exti0_isr (void)
{ 
while (1) flash(); 
} 
*/

static void motpin_set(int wire_idx, bool high)
{
  enum tim_oc_mode force_mode = high ? TIM_OCM_FORCE_HIGH : TIM_OCM_FORCE_LOW;
  enum tim_oc_id channel;
  channel = TIM_OC1;
  switch (wire_idx) {
  case 0: channel = TIM_OC1; break;
  case 1: channel = TIM_OC2; break;
  case 2: channel = TIM_OC3; break;
  case 3: channel = TIM_OC4; break;
  default: return;
  }
  timer_set_oc_mode(TIM3, channel, force_mode);
}

static void motpin_pwm(int wire_idx, uint32_t pwm)
{
  enum tim_oc_id channel;
  switch (wire_idx) {
  case 0: channel = TIM_OC1; break;
  case 1: channel = TIM_OC2; break;
  case 2: channel = TIM_OC3; break;
  case 3: channel = TIM_OC4; break;
  default: return;
  }
  timer_set_oc_mode(TIM3, channel, TIM_OCM_PWM1); // non-inverted PWM
  timer_set_oc_value(TIM3, channel, pwm);
}

static void flash(void)
{
  gpio_set(GPIOB, GPIO13);
  for (int i = 0; i < 8800000; i++) __asm__("nop");
  gpio_clear(GPIOB, GPIO13);
  for (int i = 0; i < 1000000; i++) __asm__("nop");
}

static void pause(void)
{
  for (int i = 0; i < 4000000; i++) __asm__("nop");
}

#include "halfsin_lut.c"

void tim3_isr(void)
{
  DEBUG1_SET;
  TIM_SR(TIM3) &= ~TIM_SR_UIF; // inline from timer_clear_flag()
  static uint32_t phase=0;

  uint32_t speed = 420000;
  phase += speed; // with 32bit overflow

  const int HALFSIN_LUT_BITS = 12;
  const int HALFSIN_LUT_SIZE = 1<<HALFSIN_LUT_BITS;

  uint32_t phase2 = phase + (1<<29); // add 1/2 pi for cosine
  uint32_t sin_amp = halfsin_lut[(phase  & 0x7FFFFFFF) >> (31 - HALFSIN_LUT_BITS)]; // first coil
  uint32_t cos_amp = halfsin_lut[(phase2 & 0x7FFFFFFF) >> (31 - HALFSIN_LUT_BITS)]; // second coil

  const double amplitude = 0.6;
  const uint32_t pwm_max = amplitude*(72000000 / 32000 + 1);
  uint16_t pwm1 = sin_amp / (0xFFFFFFFFu / pwm_max);
  uint16_t pwm2 = pwm1;
  if (phase > 0x7FFFFFFF) {
    pwm1 = 0; // positive
  } else {
    pwm2 = 0; // negative
  }

  uint16_t pwm3 = cos_amp / (0xFFFFFFFFu / pwm_max);
  uint16_t pwm4 = pwm3;
  if (phase2 > 0x7FFFFFFF) {
    pwm3 = 0; // positive
  } else {
    pwm4 = 0; // negative
  }
 
  TIM_CCR1(TIM3) = pwm1; // inline from timer_set_oc_value()
  TIM_CCR2(TIM3) = pwm2;
  TIM_CCR3(TIM3) = pwm3;
  TIM_CCR4(TIM3) = pwm4;

  DEBUG1_CLEAR;
}

int main(void)
{
  setup();
  flash();
  pause();

  /* Blink the LEDs on the board. */
  int p = 0;
  while (1) {
    flash();
    pause();
    pause();
    pause();
    pause();
  }

  return 0;
}
