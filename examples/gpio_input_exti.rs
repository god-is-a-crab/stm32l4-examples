#![no_std]
#![no_main]

use cortex_m::{asm, Peripherals as CorePeripherals};
use cortex_m_rt::entry;
use panic_semihosting as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use stm32l4::stm32l4x2::{interrupt, Interrupt, Peripherals as DevicePeripherals};

static mut DEVICE_PERIPHERALS: Option<DevicePeripherals> = None;

#[interrupt]
fn EXTI1() {
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    if dp.EXTI.pr1().read().pr1().bit_is_set() {
        if dp.GPIOA.idr().read().idr1().bit_is_set() {
            // Set A9 low if A0 is high
            dp.GPIOA.bsrr().write(|w| w.br9().set_bit());
        } else {
            // Set A9 high if A0 is low
            dp.GPIOA.bsrr().write(|w| w.bs9().set_bit());
        }
        dp.EXTI.pr1().write(|w| w.pr1().clear_bit_by_one());
    }
}

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let cp = CorePeripherals::take().unwrap();
    let dp = DevicePeripherals::take().unwrap();

    // Chapter 6 - Reset and clock control (RCC)
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());

    // Chapter 13 - Extended interrupts and events controller (EXTI)
    // Map EXTI line 1 to PA
    dp.SYSCFG
        .exticr1()
        .write(|w| unsafe { w.exti1().bits(0b000) });
    // Enable rising and falling trigger
    dp.EXTI.ftsr1().write(|w| w.tr1().set_bit());
    dp.EXTI.rtsr1().write(|w| w.tr1().set_bit());
    // Unmask interrupt
    dp.EXTI.imr1().write(|w| w.mr1().set_bit());

    // Chapter 8
    dp.GPIOA
        .moder()
        .write(|w| w.moder1().input().moder9().output());
    dp.GPIOA.otyper().write(|w| w.ot9().push_pull());
    dp.GPIOA.pupdr().write(|w| w.pupdr1().pull_down());

    unsafe {
        DEVICE_PERIPHERALS = Some(dp);
        cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI1);
        cp.SCB.scr.write(0b010);
    }

    loop {
        asm::wfi();
    }
}
