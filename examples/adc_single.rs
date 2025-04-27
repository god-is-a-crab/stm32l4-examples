#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use cortex_m::asm;
use cortex_m_rt::entry;
use panic_semihosting as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use stm32l4::stm32l4x2::{Interrupt, Peripherals as DevicePeripherals, interrupt};

static mut DEVICE_PERIPHERALS: Option<DevicePeripherals> = None;

#[interrupt]
fn ADC1_2() {
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    if dp.ADC1.isr().read().eoc().bit_is_set() {
        // Reading ADC1 DR resets EOC
        dp.USART2
            .tdr()
            .write(|w| w.tdr().set(dp.ADC1.dr().read().rdata().bits()));
    }
}

#[interrupt]
fn USART2() {
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    if dp.USART2.isr().read().rxne().bit_is_set() {
        let _received_byte = dp.USART2.rdr().read().rdr().bits();
        dp.ADC1.cr().modify(|_, w| w.adstart().set_bit());
    }
    if dp.USART2.isr().read().ore().bit_is_set() {
        dp.USART2.icr().write(|w| w.orecf().set_bit());
    }
}

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let dp = unsafe {
        DEVICE_PERIPHERALS = Some(DevicePeripherals::take().unwrap());
        DEVICE_PERIPHERALS.as_mut().unwrap()
    };

    // Chapter 6 - Reset and clock control (RCC)
    dp.RCC
        .ahb2enr()
        .write(|w| w.gpioaen().set_bit().adcen().set_bit());
    dp.RCC.apb1enr1().write(|w| w.usart2en().set_bit());
    // Ref manual p.234 - ADC needs clock source
    dp.RCC.ccipr().write(|w| w.adcsel().sysclk());

    // Chapter 8
    // PA1: analog (ADC channel 6)
    // See datasheet table 14 for pin to ADC channel mapping
    // See table 64 for ADC sampling time constraints
    dp.GPIOA.moder().write(|w| {
        w.moder1()
            .analog()
            .moder2()
            .alternate()
            .moder3()
            .alternate()
    });
    dp.GPIOA.afrl().write(|w| w.afrl2().af7().afrl3().af7());

    // USART2
    dp.USART2.brr().write(|w| unsafe { w.bits(417) });
    dp.USART2.cr1().write(|w| {
        w.rxneie()
            .set_bit()
            .re()
            .set_bit()
            .te()
            .set_bit()
            .ue()
            .set_bit()
    });

    // ADC1
    dp.ADC1
        .cr()
        .write(|w| w.deeppwd().clear_bit().advregen().set_bit());
    // See datasheet table 63 "ADC voltage regulator start-up time" p.115 (20 us)
    for _ in 0..80 {
        asm::nop(); // A single cycle should be 0.25 us
    }
    // ADC calibration p.383
    dp.ADC1.cr().modify(|_, w| w.adcal().set_bit());
    while dp.ADC1.cr().read().adcal().bit_is_set() {}

    // Enable EOC and clear ADRDY
    dp.ADC1.ier().write(|w| w.eocie().set_bit());
    dp.ADC1.isr().write(|w| w.adrdy().set_bit());
    dp.ADC1.cr().modify(|_, w| w.aden().set_bit());
    while dp.ADC1.isr().read().adrdy().bit_is_clear() {}

    // Set 8-bit resolution
    dp.ADC1.cfgr().write(|w| unsafe { w.res().bits(0b10) });
    // Set sequence length to 1, first conversion to channel 6
    dp.ADC1
        .sqr1()
        .write(|w| unsafe { w.l().bits(0).sq1().bits(6) });
    // Sample time = 12.5 ADC cycles
    dp.ADC1.smpr1().write(|w| unsafe { w.smp6().bits(0b010) });

    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::ADC1_2); // EOC
        cortex_m::peripheral::NVIC::unmask(Interrupt::USART2); // RXNE
    }

    loop {
        asm::wfi();
    }
}
