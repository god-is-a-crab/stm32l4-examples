#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::entry;
use panic_semihosting as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use stm32l4::stm32l4x2::{interrupt, Interrupt, Peripherals as DevicePeripherals};

static mut DEVICE_PERIPHERALS: Option<DevicePeripherals> = None;

#[interrupt]
fn TIM1_UP_TIM16() {
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    if dp.TIM16.sr().read().uif().bit_is_set() {
        // A9 will blink for 0.5s
        dp.GPIOA.bsrr().write(|w| w.br9().set_bit());
        dp.TIM16.sr().write(|w| w.uif().clear_bit());
    }
}

#[interrupt]
fn TIM6_DACUNDER() {
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    if dp.TIM6.sr().read().uif().bit_is_set() {
        // Blink A9 every 2s
        dp.GPIOA.bsrr().write(|w| w.bs9().set_bit());
        dp.TIM16.cr1().write(|w| w.opm().set_bit().cen().set_bit());
        dp.TIM6.sr().write(|w| w.uif().clear_bit());
    }
}

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let dp = DevicePeripherals::take().unwrap();

    // Chapter 6 - Reset and clock control (RCC)
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());
    dp.RCC.apb1enr1().write(|w| w.tim6en().set_bit());
    dp.RCC.apb2enr().write(|w| w.tim16en().set_bit());

    // Chapter 8 - GPIO
    dp.GPIOA.moder().write(|w| w.moder9().output());
    dp.GPIOA.otyper().write(|w| w.ot9().push_pull());
    dp.GPIOA.ospeedr().write(|w| w.ospeedr9().low_speed());

    // Chapter 28 - General purpose timer TIM16
    // Set 0.5s interval
    dp.TIM16.psc().write(|w| w.psc().set(99)); // 4Mhz / (99 + 1) = 4 kHz
    dp.TIM16.arr().write(|w| unsafe { w.arr().bits(20000) }); // 40000 * 0.5
    dp.TIM16.dier().write(|w| w.uie().set_bit());

    // Chapter 29 - Basic timer TIM6
    // Set 2s interval
    dp.TIM6.psc().write(|w| w.psc().set(49999)); // 4Mhz / (49999 + 1) = 80 Hz
    dp.TIM6.arr().write(|w| w.arr().set(160)); // 80 * 2 = 160
    dp.TIM6.dier().write(|w| w.uie().set_bit());

    dp.TIM6.cr1().write(|w| w.cen().set_bit());

    unsafe {
        DEVICE_PERIPHERALS = Some(dp);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM6_DACUNDER);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM1_UP_TIM16);
    }

    loop {
        asm::wfi();
    }
}
