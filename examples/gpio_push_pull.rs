#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::entry;
use panic_semihosting as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use stm32l4::stm32l412::Peripherals as DevicePeripherals;

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let dp = DevicePeripherals::take().unwrap();

    // Chapter 6 - Reset and clock control (RCC)
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());

    // Chapter 8
    dp.GPIOA.moder().write(|w| w.moder9().output());
    dp.GPIOA.otyper().write(|w| w.ot9().push_pull());
    dp.GPIOA.ospeedr().write(|w| w.ospeedr9().low_speed());
    dp.GPIOA.bsrr().write(|w| w.bs9().set_bit());

    loop {
        asm::wfi();
    }
}
