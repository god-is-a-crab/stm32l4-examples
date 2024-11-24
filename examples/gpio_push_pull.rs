#![no_std]
#![no_main]

use panic_semihosting as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use cortex_m_rt::entry;
use cortex_m::asm;
use stm32l4::stm32l4x2::Peripherals as DevicePeripherals;

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let dp = DevicePeripherals::take().unwrap();

    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());

    dp.GPIOA.moder().write(|w| w.moder9().output());
    dp.GPIOA.otyper().write(|w| w.ot9().push_pull());
    dp.GPIOA.ospeedr().write(|w| w.ospeedr9().low_speed());
    dp.GPIOA.bsrr().write(|w| w.bs9().set_bit());

    loop {
        asm::wfi();
    }
}
