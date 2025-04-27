#![no_std]
#![no_main]

use cortex_m::{Peripherals as CorePeripherals, asm};
use cortex_m_rt::entry;
use panic_semihosting as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use stm32l4::stm32l4x2::{Interrupt, Peripherals as DevicePeripherals, interrupt};

static mut DEVICE_PERIPHERALS: Option<DevicePeripherals> = None;

#[interrupt]
fn TIM6_DACUNDER() {
    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    if dp.TIM6.sr().read().uif().bit_is_set() {
        if dp.GPIOA.odr().read().odr9().bit_is_clear() {
            dp.GPIOA.bsrr().write(|w| w.bs9().set_bit());
        } else {
            dp.GPIOA.bsrr().write(|w| w.br9().set_bit());
        }
        dp.TIM6.sr().write(|w| w.uif().clear_bit());
    }
}

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let cp = CorePeripherals::take().unwrap();
    let dp = DevicePeripherals::take().unwrap();

    // SLEEPDEEP | SLEEPONEXIT | Res.
    const SLEEPONEXIT_ON: u32 = 0b010;
    const _SLEEPONEXIT_OFF: u32 = 0b0;

    // Chapter 6 - Reset and clock control (RCC)
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());
    dp.RCC.apb1enr1().write(|w| w.tim6en().set_bit());

    // Chapter 8 - GPIO
    dp.GPIOA
        .moder()
        .write(|w| w.moder9().output().moder11().output());
    dp.GPIOA
        .otyper()
        .write(|w| w.ot9().push_pull().ot11().push_pull());

    // Chapter 29 - Basic timer TIM6
    // Set 1s interval
    dp.TIM6.psc().write(|w| w.psc().set(49999)); // 4Mhz / (49999 + 1) = 80 Hz
    dp.TIM6.arr().write(|w| w.arr().set(80));
    dp.TIM6.dier().write(|w| w.uie().set_bit());
    dp.TIM6.cr1().write(|w| w.cen().set_bit());

    unsafe {
        DEVICE_PERIPHERALS = Some(dp);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM6_DACUNDER);
        cp.SCB.scr.write(SLEEPONEXIT_ON);
    }

    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();
    asm::wfi();

    loop {
        // Should never go high
        dp.GPIOA.bsrr().write(|w| w.bs11().set_bit());
        asm::wfi();
    }
}
