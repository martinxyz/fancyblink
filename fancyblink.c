#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/scb.h>

static void clock_setup(void)
{
  SCB_VTOR = 0x08000000; // required if you use interrupts and start from STM32 built-in bootloader (e.g. BOOT0 pin was high during last reset)
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  rcc_periph_clock_enable(RCC_GPIOB);
}

static void gpio_setup(void)
{
  // LED = GPIO Bank B, no 13
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

// sine lookup table
#include "fancyblink_lut.c"

int main(void)
{
  clock_setup();
  gpio_setup();

  while (1) {
    const int slowdown = 8;
    for (int j=0; j<1000*slowdown; j++) {
      int n1 = j / slowdown;
      n1 = lut[n1];
      int n2 = 1000-n1; 
      gpio_set(GPIOB, GPIO13);
      for (int i = 0; i < n1; i++) __asm__("nop");
      gpio_clear(GPIOB, GPIO13);
      for (int i = 0; i < n2; i++) __asm__("nop");
    }
  }
  return 0;
}
