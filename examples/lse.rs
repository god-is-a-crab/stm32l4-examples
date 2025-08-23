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
    dp.RCC.apb1enr1().write(|w| w.pwren().set_bit());
    dp.PWR.cr1().write(|w| w.dbp().set_bit()); // Remove domain backup protection
    dp.RCC.bdcr().write(|w| unsafe { w.lseon().set_bit().lsedrv().bits(0b00) });
    dp.RCC.cfgr().write(|w| unsafe { w.mcosel().bits(0b0111) });

    // Chapter 8
    dp.GPIOA.moder().write(|w| w.moder8().alternate().moder11().output());
    dp.GPIOA.afrh().write(|w| w.afrh8().af0());
    dp.GPIOA.otyper().write(|w| w.ot11().push_pull());
    while dp.RCC.bdcr().read().lserdy().bit_is_clear() {}
    dp.GPIOA.bsrr().write(|w| w.bs11().set_bit());

    loop {
        asm::wfi();
    }
}
