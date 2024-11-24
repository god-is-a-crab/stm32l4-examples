#![no_std]
#![no_main]

use cortex_m::{asm, Peripherals as CorePeripherals};
use cortex_m_rt::entry;
use panic_semihosting as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use stm32l4::stm32l4x2::{interrupt, Interrupt, Peripherals as DevicePeripherals};

const USART2_RDR: u32 = 0x4000_4424;
const USART1_TDR: u32 = 0x4001_3828;
const DOUBLE_BUFFER_SIZE: usize = 256;

static mut DEVICE_PERIPHERALS: Option<DevicePeripherals> = None;
static mut DOUBLE_BUFFER: [u8; DOUBLE_BUFFER_SIZE] = [0; DOUBLE_BUFFER_SIZE];

#[inline]
fn send_payload(buffer_ptr: u32, dp: &mut DevicePeripherals) {
    // Set memory address
    dp.DMA1
        .ch4()
        .mar()
        .write(|w| unsafe { w.ma().bits(buffer_ptr) });
    // Set transaction size
    dp.DMA1
        .ch4()
        .ndtr()
        .write(|w| unsafe { w.bits(DOUBLE_BUFFER_SIZE as u32 / 2) });
    // Enable DMA ch7
    dp.DMA1.ch4().cr().write(|w| {
        w.msize()
            .bits8()
            .psize()
            .bits8()
            .minc()
            .set_bit()
            .dir()
            .set_bit()
            .tcie()
            .set_bit()
            .en()
            .set_bit()
    });
}

#[interrupt]
fn DMA1_CH6() {
    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    if dp.DMA1.isr().read().htif6().bit_is_set() {
        // Send payload in the first half of the buffer
        send_payload(
            unsafe { DOUBLE_BUFFER[..DOUBLE_BUFFER_SIZE / 2].as_ptr() as u32 },
            dp,
        );
        dp.DMA1.ifcr().write(|w| w.chtif6().set_bit());
    } else if dp.DMA1.isr().read().tcif6().bit_is_set() {
        // Send payload in the second half of the buffer
        send_payload(
            unsafe { DOUBLE_BUFFER[DOUBLE_BUFFER_SIZE / 2..].as_ptr() as u32 },
            dp,
        );
        dp.DMA1.ifcr().write(|w| w.ctcif6().set_bit());
    }
}

#[interrupt]
fn DMA1_CH4() {
    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    if dp.DMA1.isr().read().tcif4().bit_is_set() {
        // Disable DMA CH4
        dp.DMA1.ch4().cr().modify(|_, w| w.en().clear_bit());
        dp.DMA1.ifcr().write(|w| w.ctcif4().set_bit());
    }
}

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let cp = CorePeripherals::take().unwrap();
    let dp = DevicePeripherals::take().unwrap();

    // Chapter 6 - Reset and clock control (RCC)
    dp.RCC.ahb1enr().write(|w| w.dma1en().set_bit());
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());
    dp.RCC.apb1enr1().write(|w| w.usart2en().set_bit());
    dp.RCC.apb2enr().write(|w| w.usart1en().set_bit());

    // Chapter 8 - GPIO
    // A3 (USART2 RX), A9 (USART1 TX)
    dp.GPIOA
        .moder()
        .write(|w| w.moder3().alternate().moder9().alternate());
    // See datasheet chapter 4 - alternate function table
    dp.GPIOA.afrl().write(|w| w.afrl3().af7());
    dp.GPIOA.afrh().write(|w| w.afrh9().af7());

    // Chapter 11 - DMA
    // See chapter 11.3 table 41: DMA channel mappings
    dp.DMA1.cselr().write(|w| w.c4s().map2().c6s().map2());

    // CH4: USART2 TX DMA
    dp.DMA1
        .ch4()
        .par()
        .write(|w| unsafe { w.pa().bits(USART1_TDR) });

    // CH6: USART2 RX DMA
    dp.DMA1
        .ch6()
        .par()
        .write(|w| unsafe { w.pa().bits(USART2_RDR) });
    #[allow(static_mut_refs)]
    dp.DMA1
        .ch6()
        .mar()
        .write(|w| unsafe { w.ma().bits(DOUBLE_BUFFER.as_ptr() as u32) });
    dp.DMA1
        .ch6()
        .ndtr()
        .write(|w| unsafe { w.bits(DOUBLE_BUFFER_SIZE as u32) });
    dp.DMA1.ch6().cr().write(|w| {
        w.msize()
            .bits8()
            .psize()
            .bits8()
            .minc()
            .set_bit()
            .circ()
            .set_bit()
            .htie()
            .set_bit()
            .tcie()
            .set_bit()
            .en()
            .set_bit()
    });

    // Chapter 38 - USART
    // Need 2 UARTS, the transmitting UART needs to be faster than the receiving one
    dp.USART2.brr().write(|w| unsafe { w.bits(104) }); // 4Mhz / 38400 approx. 104
    dp.USART2.cr3().write(|w| w.dmar().set_bit());
    dp.USART2.cr1().write(|w| w.re().set_bit().ue().set_bit());

    dp.USART1.brr().write(|w| unsafe { w.bits(69) }); // 4Mhz / 57600 approx. 69
    dp.USART1.cr3().write(|w| w.dmat().set_bit());
    dp.USART1.cr1().write(|w| w.te().set_bit().ue().set_bit());

    unsafe {
        DEVICE_PERIPHERALS = Some(dp);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH4);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH6);
        cp.SCB.scr.write(0b010);
    }

    loop {
        asm::wfi();
    }
}
