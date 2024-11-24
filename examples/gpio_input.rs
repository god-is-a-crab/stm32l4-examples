#![no_std]
#![no_main]

use panic_semihosting as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use cortex_m_rt::entry;
use stm32l4::stm32l4x2::Peripherals as DevicePeripherals;

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let dp = DevicePeripherals::take().unwrap();

    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());

    // Chapter 8
    dp.GPIOA.moder().write(|w| w.moder0().input().moder9().output());
    dp.GPIOA.otyper().write(|w| w.ot9().push_pull());
    dp.GPIOA.ospeedr().write(|w| w.ospeedr9().low_speed());
    dp.GPIOA.pupdr().write(|w| w.pupdr0().pull_down());

    // Poll A0, set A9 accordingly
    // A0 <- 10k resistor <- GND/3V3 VCC
    loop {
        if dp.GPIOA.idr().read().idr0().bit_is_set() {
            // Set A9 low if A0 is high
            dp.GPIOA.bsrr().write(|w| w.br9().set_bit());
        } else {
            // Set A9 high if A0 is low
            dp.GPIOA.bsrr().write(|w| w.bs9().set_bit());
        }
    }
}
