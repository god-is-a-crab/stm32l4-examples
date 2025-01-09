#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::entry;
use panic_semihosting as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use stm32f1::stm32f103::{interrupt, Interrupt, Peripherals as DevicePeripherals};

static mut DEVICE_PERIPHERALS: Option<DevicePeripherals> = None;

#[interrupt]
fn TIM2() {
    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    if dp.TIM2.sr().read().uif().bit_is_set() {
        if dp.GPIOA.odr().read().odr0().bit_is_clear() {
            dp.GPIOA.bsrr().write(|w| w.bs0().set_bit());
        } else {
            dp.GPIOA.bsrr().write(|w| w.br0().set_bit());
        }
        dp.TIM2.sr().write(|w| w.uif().clear_bit());
    }
}

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let dp = DevicePeripherals::take().unwrap();

    // Chapter 6 - Reset and clock control (RCC)
    dp.RCC.apb2enr().write(|w| w.iopaen().set_bit());
    dp.RCC.apb1enr().write(|w| w.tim2en().set_bit());

    // Chapter 8 - GPIO
    dp.GPIOA
        .crl()
        .write(|w| w.mode0().output().cnf0().push_pull());
    dp.GPIOA.bsrr().write(|w| w.bs0().set_bit());

    // Chapter 29 - Basic timer TIM6
    // Set 1s interval
    dp.TIM2.psc().write(|w| w.psc().set(49999)); // 4Mhz / (49999 + 1) = 80 Hz
    dp.TIM2.arr().write(|w| w.arr().set(80));

    // Enable TIM6 update interrupt
    dp.TIM2.dier().write(|w| w.uie().set_bit());
    dp.TIM2.cr1().write(|w| w.cen().set_bit());

    unsafe {
        DEVICE_PERIPHERALS = Some(dp);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
    }

    loop {
        asm::wfi();
    }
}
