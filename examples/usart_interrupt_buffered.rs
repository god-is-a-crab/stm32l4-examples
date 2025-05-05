#![no_std]
#![no_main]

use cortex_m::{Peripherals as CorePeripherals, asm};
use cortex_m_rt::entry;
use heapless::spsc::Queue;
use panic_semihosting as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use stm32l4::stm32l4x2::{Interrupt, Peripherals as DevicePeripherals, interrupt};

static mut DEVICE_PERIPHERALS: Option<DevicePeripherals> = None;
static mut BUFFER: Option<Queue<u16, 8>> = None;

#[interrupt]
fn USART2() {
    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();
    #[allow(static_mut_refs)]
    let buffer = unsafe { BUFFER.as_mut() }.unwrap();

    if dp.USART2.isr().read().rxne().bit_is_set() {
        // Read data, this clears RXNE
        let received_byte = dp.USART2.rdr().read().rdr().bits();

        // Queue byte - 32, do nothing if queue is full
        if buffer.enqueue(received_byte - 32).is_ok() {
            // Enable TXE interrupt as buffer is now non-empty
            dp.USART2.cr1().modify(|_, w| w.txeie().enabled());
        }
    }
    if dp.USART2.isr().read().txe().bit_is_set() {
        match buffer.dequeue() {
            // Write dequeued byte
            Some(byte) => {
                dp.USART2.tdr().write(|w| w.tdr().set(byte));
                if buffer.is_empty() {
                    dp.USART2.cr1().modify(|_, w| w.txeie().disabled());
                }
            }
            // Buffer is empty, disable TXE interrupt
            None => {
                dp.USART2.cr1().modify(|_, w| w.txeie().disabled());
            }
        }
    }
    if dp.USART2.isr().read().ore().bit_is_set() {
        dp.USART2.icr().write(|w| w.orecf().clear_bit_by_one());
    }
}

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let cp = CorePeripherals::take().unwrap();
    let dp = DevicePeripherals::take().unwrap();

    // Chapter 6 - Reset and clock control (RCC)
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());
    dp.RCC.apb1enr1().write(|w| w.usart2en().set_bit());

    // Chapter 8 - GPIO
    // A2 (TX), A3 (RX)
    dp.GPIOA
        .moder()
        .write(|w| w.moder2().alternate().moder3().alternate());
    // See datasheet chapter 4 - alternate function table
    dp.GPIOA.afrl().write(|w| w.afrl2().af7().afrl3().af7());

    // Chapter 38 - USART
    dp.USART2.brr().write(|w| unsafe { w.bits(417) }); // 4Mhz / 9600 approx. 417
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

    unsafe {
        DEVICE_PERIPHERALS = Some(dp);
        BUFFER = Some(Queue::new());
        cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
        cp.SCB.scr.write(0b010);
    }

    loop {
        asm::wfi();
    }
}
