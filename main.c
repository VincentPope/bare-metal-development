// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

//Modified by EXAMPLE

#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>

#define FREQ 4000000UL  // CPU frequency, 4 Mhz
#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

static inline void spin(volatile uint32_t count) {
  while (count--) asm("nop");
}

struct pwr {
  volatile uint32_t CR1, CR2;
};

#define PWR ((struct pwr *)0x40007000UL)

struct systick {
  volatile uint32_t CSR, RVR, CVR, CALIB;
};
#define SYSTICK ((struct systick *) 0xe000e010)  // 2.2.2

struct rcc {
  volatile uint32_t CR, ICSCR, CFGR, PLLCFGR,       //0x00
      PLLSAI1CFGR, PLLSAI2CFGR, CIER, CIFR,         //0x10
      CICR, RESERVED0, AHB1RSTR, AHB2RSTR,          //0x20
      AHB3RSTR, RESERVED1, APB1RSTR1, APB1RSTR2,    //0x30
      APB2RSTR, RESERVED2, AHB1ENR, AHB2ENR,        //0x40
      AHB3ENR, RESERVED3, APB1ENR1, APB1ENR2,       //0x50
      APB2ENR, RESERVED4, AHB1SMENR, AHB2SMENR,     //0x60
      AHB3SMENR, RESERVED5, APB1SMENR1, APB1SMENR2, //0x70
      APB2SMENR, RESERVED6, CCIPR, RESERVED7,       //0x80
      BDCR, CSR, CRRCR, CCIPR2,                     //0x90
      RESERVED8, DLYCFGR;                           //0xA0
};
#define RCC ((struct rcc *) 0x40021000)

static inline void systick_init(uint32_t ticks) {
  if ((ticks - 1) > 0xffffff) return; //24 bit timer
  SYSTICK->RVR = ticks - 1;
  SYSTICK->CVR = 0;
  SYSTICK->CSR = BIT(0) | BIT(1) | BIT(2);
}

struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2], BRR;
};
#define GPIO(bank) ((struct gpio *) (0x48000000 + 0x400 * (bank)))

// Enum values are per datasheet: 0, 1, 2, 3
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                      // Pin number
  RCC->AHB2ENR |= BIT(PINBANK(pin));
  (void)RCC->AHB2ENR;  // Allow clock enable to take effect
  gpio->MODER &= ~(3U << (n * 2));         // Clear existing setting
  gpio->MODER |= (mode & 3U) << (n * 2);   // Set new mode
}

static inline void gpio_set_af(uint16_t pin, uint8_t af_num) {
  struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                      // Pin number
  gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4));
  gpio->AFR[n >> 3] |= ((uint32_t) af_num) << ((n & 7) * 4);
}

static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16);
}

struct uart
{
  volatile uint32_t CR1, CR2, CR3, BRR, GTPR, RTOR, RQR, ISR, ICR, RDR, TDR, PRESC;
};
#define UART1 ((struct uart *)0x40013800UL)
#define UART2 ((struct uart *)0x40004400UL)
#define UART3 ((struct uart *)0x40004800UL)
#define LPUART1 ((struct uart *)0x40008000UL)


static inline void uart_init(struct uart *uart, unsigned long baud) {
  uint8_t af = 7;
  uint16_t rx, tx;

  if (baud == 0) return;

  if (uart == UART1) {
    RCC->APB2ENR |= BIT(14);
    (void)RCC->APB2ENR;
    RCC->CCIPR &= ~(3UL << 0);  // USART1 clock = PCLK2
    tx = PIN('A', 9);
    rx = PIN('A', 10);
  } else if (uart == UART2) {
    RCC->APB1ENR1 |= BIT(17);
    (void)RCC->APB1ENR1;
    RCC->CCIPR &= ~(3UL << 2);  // USART2 clock = PCLK1
    tx = PIN('A', 2);
    rx = PIN('A', 3);
  } else if (uart == UART3) {
    RCC->APB1ENR1 |= BIT(18);
    (void)RCC->APB1ENR1;
    RCC->CCIPR &= ~(3UL << 4);  // USART3 clock = PCLK1
    tx = PIN('D', 8);
    rx = PIN('D', 9);
  } else if (uart == LPUART1) {
    RCC->APB1ENR1 |= BIT(28);   // PWR clock
    (void)RCC->APB1ENR1;
    PWR->CR2 |= BIT(9);         // Enable VDDIO2 pins

    RCC->APB1ENR2 |= BIT(0);
    (void)RCC->APB1ENR2;
    RCC->CCIPR &= ~(3UL << 10); // LPUART1 clock = PCLK1

    tx = PIN('G', 7);
    rx = PIN('G', 8);
    af = 8;
  } else {
    return;
  }

  uart->CR1 = 0;

  gpio_set_mode(tx, GPIO_MODE_AF);
  gpio_set_af(tx, af);
  gpio_set_mode(rx, GPIO_MODE_AF);
  gpio_set_af(rx, af);

  uart->CR2 = 0;
  uart->CR3 = 0;
  uart->PRESC = 0;

  if (uart == LPUART1) {
    uart->BRR =
        (uint32_t)((256UL * FREQ + baud / 2UL) / baud);
  } else {
    uart->BRR = (uint32_t)((FREQ + baud / 2UL) / baud);
  }

  uart->CR1 = BIT(0) | BIT(2) | BIT(3);  // UE, RE, TE

  while ((uart->ISR & (BIT(21) | BIT(22))) !=
         (BIT(21) | BIT(22))) {
    spin(1);
  }
}

static inline void uart_write_byte(struct uart *uart, uint8_t byte) {
  while ((uart->ISR & BIT(7)) == 0) spin(1);
  uart->TDR = byte;
}

static inline void uart_write_buf(struct uart *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}

static inline int uart_read_ready(struct uart *uart) {
  return uart->ISR & BIT(5);  // If RXNE bit is set, data is ready
}

static inline uint8_t uart_read_byte(struct uart *uart) {
  return (uint8_t) (uart->RDR & 255);
}

static volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

// t: expiration time, prd: period, now: current time. Return true if expired
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}

int main(void) {
  uint16_t led = PIN('B', 7);            // Blue LED
  systick_init(FREQ / 1000UL);         // Tick every 1 ms
  gpio_set_mode(led, GPIO_MODE_OUTPUT);  // Set blue LED to output mode
  uart_init(LPUART1, 115200);              // Initialise UART
  uint32_t timer = 0, period = 500;      // Declare timer and 500ms period
  for (;;) {
    if (timer_expired(&timer, period, s_ticks)) {
      static bool on;                      // This block is executed
      gpio_write(led, on);                 // Every `period` milliseconds
      on = !on;                            // Toggle LED state
      uart_write_buf(LPUART1, "hi\r\n", 4);  // Write message
    }
    // Here we could perform other activities!
  }
  return 0;
}

// Startup code
__attribute__((noreturn)) void _reset(void) {
  // Initialise memory
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;

  // Call main()
  main();
  for (;;) (void) 0;  // Infinite loop
}

extern void _estack(void);  // Defined in link.ld

// 16 standard and 91 STM32-specific handlers
#if defined(__GNUC__)
__attribute__((section(".vectors")))
#endif
void (*const tab[16 + 95])(void) = {
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler};
