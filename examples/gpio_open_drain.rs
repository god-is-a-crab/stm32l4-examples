#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::entry;
use panic_semihosting as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use stm32l4::stm32l4x2::Peripherals as DevicePeripherals;

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let dp = DevicePeripherals::take().unwrap();

    // Chapter 6 - Reset and clock control (RCC)
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());

    // Chapter 8
    dp.GPIOA.moder().write(|w| w.moder9().output());
    // Only the low level is driven, high level is HI-Z
    dp.GPIOA.otyper().write(|w| w.ot9().open_drain());
    dp.GPIOA.ospeedr().write(|w| w.ospeedr9().low_speed());
    // External pull-up pulls A9 high when in HI-Z
    // Connect a 10k pull resistor to the HI-Z output signal and then connect that resistor
    // to GND/3V3 VCC
    dp.GPIOA.bsrr().write(|w| w.bs9().set_bit());

    loop {
        asm::wfi();
    }
}
