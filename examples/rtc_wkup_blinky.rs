#![no_std]
#![no_main]

use cortex_m::{asm, Peripherals as CorePeripherals};
use cortex_m_rt::entry;
use panic_semihosting as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use stm32l4::stm32l4x2::{interrupt, Interrupt, Peripherals as DevicePeripherals};

static mut DEVICE_PERIPHERALS: Option<DevicePeripherals> = None;

#[interrupt]
fn RTC_WKUP() {
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    if dp.RTC.isr().read().wutf().bit_is_set() {
        dp.RTC.isr().modify(|_, w| w.wutf().clear_bit());
        dp.EXTI.pr1().write(|w| w.pr20().clear_bit_by_one());

        if dp.GPIOA.odr().read().odr11().bit_is_clear() {
            dp.GPIOA.bsrr().write(|w| w.bs11().set_bit());
        } else {
            dp.GPIOA.bsrr().write(|w| w.br11().set_bit());
        }
    }
}

#[interrupt]
fn TIM6_DACUNDER() {
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

    // Chapter 6 - Reset and clock control (RCC)
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());
    dp.RCC
        .apb1enr1()
        .write(|w| w.tim6en().set_bit().pwren().set_bit().rtcapben().set_bit());

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

    // Chapter 36 - RTC
    // Configure periodic wakeup
    const SLEEP_DURATION: u16 = 1;
    // Remove write protection from BDCR and RTC registers
    dp.PWR.cr1().write(|w| w.dbp().set_bit());

    // Select LSI as RTC clock source
    dp.RCC.csr().write(|w| w.lsion().set_bit());
    while dp.RCC.csr().read().lsirdy().bit_is_clear() {}
    dp.RCC.bdcr().write(|w| w.rtcsel().lsi().rtcen().set_bit());

    // Remove write protection from RTC registers
    dp.RTC.wpr().write(|w| unsafe { w.key().bits(0xCA) });
    dp.RTC.wpr().write(|w| unsafe { w.key().bits(0x53) });

    // Enter init mode to set prescaler values
    dp.RTC.isr().write(|w| w.init().set_bit());
    while dp.RTC.isr().read().initf().bit_is_clear() {}
    dp.RTC
        .prer()
        .write(|w| unsafe { w.prediv_a().bits(127).prediv_s().bits(249) });
    dp.RTC.isr().write(|w| w.init().clear_bit());

    // Turn off wake-up timer
    dp.RTC.cr().write(|w| w.wute().clear_bit());
    while dp.RTC.isr().read().wutwf().bit_is_clear() {}

    // Write wake-up timer registers
    dp.RTC
        .wutr()
        .write(|w| unsafe { w.wut().bits(SLEEP_DURATION - 1) });

    // Enable rising edge trigger on line 20
    dp.EXTI.imr1().write(|w| w.mr20().set_bit());
    dp.EXTI.rtsr1().write(|w| w.tr20().set_bit());
    dp.RTC.cr().write(|w| {
        unsafe { w.wucksel().bits(0b100) }
            .wutie()
            .set_bit()
            .wute()
            .set_bit()
    });

    unsafe {
        DEVICE_PERIPHERALS = Some(dp);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM6_DACUNDER);
        cortex_m::peripheral::NVIC::unmask(Interrupt::RTC_WKUP);
        cp.SCB.scr.write(0b010);
    }

    loop {
        asm::wfi();
    }
}
