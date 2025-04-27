#![no_std]
#![no_main]

use cortex_m::{Peripherals as CorePeripherals, asm};
use cortex_m_rt::entry;
use panic_semihosting as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use stm32l4::stm32l4x2::{Interrupt, Peripherals as DevicePeripherals, interrupt};

const SLEEPDEEP_ON: u32 = 0b110;
const SLEEPDEEP_OFF: u32 = 0b010;
static mut CORE_PERIPHERALS: Option<CorePeripherals> = None;
static mut DEVICE_PERIPHERALS: Option<DevicePeripherals> = None;

#[interrupt]
fn EXTI1() {
    #[allow(static_mut_refs)]
    let cp = unsafe { CORE_PERIPHERALS.as_mut() }.unwrap();
    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    if dp.EXTI.pr1().read().pr1().bit_is_set() {
        if dp.GPIOA.idr().read().idr1().bit_is_set() {
            unsafe {
                // Enter Stop 2
                cp.SCB.scr.write(SLEEPDEEP_ON);
                dp.PWR.cr1().write(|w| w.lpms().bits(2).dbp().set_bit());
            }
        } else {
            unsafe {
                // Exit Stop 2
                cp.SCB.scr.write(SLEEPDEEP_OFF);
                // Enter run mode
                dp.PWR.cr1().write(|w| w.lpms().bits(1).dbp().set_bit());
            }
        }
        dp.EXTI.pr1().write(|w| w.pr1().clear_bit_by_one());
    }
}

#[interrupt]
fn RTC_WKUP() {
    #[allow(static_mut_refs)]
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

    // Chapter 6 - Reset and clock control (RCC)
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());
    dp.RCC
        .apb1enr1()
        .write(|w| w.tim6en().set_bit().pwren().set_bit().rtcapben().set_bit());

    // Chapter 8 - GPIO
    dp.GPIOA
        .moder()
        .write(|w| w.moder1().input().moder9().output().moder11().output());
    dp.GPIOA
        .otyper()
        .write(|w| w.ot9().push_pull().ot11().push_pull());
    dp.GPIOA.pupdr().write(|w| w.pupdr1().pull_down());

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
    dp.EXTI.imr1().write(|w| w.mr1().set_bit().mr20().set_bit());
    dp.EXTI
        .rtsr1()
        .write(|w| w.tr1().set_bit().tr20().set_bit());
    dp.EXTI.ftsr1().write(|w| w.tr1().set_bit());
    dp.RTC.cr().write(|w| {
        unsafe { w.wucksel().bits(0b100) }
            .wutie()
            .set_bit()
            .wute()
            .set_bit()
    });

    unsafe {
        CORE_PERIPHERALS = Some(cp);
        DEVICE_PERIPHERALS = Some(dp);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM6_DACUNDER);
        cortex_m::peripheral::NVIC::unmask(Interrupt::RTC_WKUP);
        cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI1);
    }

    loop {
        asm::wfi();
    }
}
