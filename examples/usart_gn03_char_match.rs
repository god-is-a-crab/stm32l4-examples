#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use cortex_m::{Peripherals as CorePeripherals, asm};
use cortex_m_rt::entry;
use panic_semihosting as _;
use stm32l4::stm32l4x2::{Interrupt, Peripherals as DevicePeripherals, interrupt};

// Peripherals
static mut CORE_PERIPHERALS: Option<CorePeripherals> = None;
static mut DEVICE_PERIPHERALS: Option<DevicePeripherals> = None;

// Constants
const USART1_RDR: u32 = 0x4001_3824;
const USART1_TDR: u32 = 0x4001_3828;
const USART2_TDR: u32 = 0x4000_4428;
/// Disable all sentences except GGA
const GNSS_SET_SENTENCES: [u8; 43] = *b"$PCAS03,1,0,0,0,0,0,0,0,0,0,,,0,0,,,,0*33\r\n";

static mut NMEA_BUFFER: [u8; 1024] = [0; 1024];
static mut SENTENCE_BEGIN: u32 = 0;
static mut SENTENCE: [u8; 128] = [0; 128];

// USART1(GNSS) TX
#[interrupt]
fn DMA1_CH4() {
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();
    if dp.DMA1.isr().read().tcif4().bit_is_set() {
        dp.USART1.cr1().modify(|_, w| w.te().clear_bit());
        dp.DMA1.ch4().cr().write(|w| w.en().clear_bit());
        dp.DMA1.ifcr().write(|w| w.ctcif4().set_bit());
    }
}

#[interrupt]
fn DMA1_CH7() {
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();
    if dp.DMA1.isr().read().tcif7().bit_is_set() {
        dp.DMA1.ch7().cr().write(|w| w.en().clear_bit());
        dp.DMA1.ifcr().write(|w| w.ctcif7().set_bit());
    }
}

#[interrupt]
fn USART1() {
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();
    if dp.USART1.isr().read().cmf().bit_is_set() {
        const NMEA_BUFFER_SIZE: usize = unsafe { NMEA_BUFFER.len() };

        // Calculate sentence length
        let bytes_read = NMEA_BUFFER_SIZE as u32 - dp.DMA1.ch5().ndtr().read().bits();
        let sentence_length = u32::min(
            bytes_read - unsafe { SENTENCE_BEGIN },
            NMEA_BUFFER_SIZE as u32 - unsafe { SENTENCE_BEGIN } + bytes_read,
        );

        // Send sentence over USART2 TX
        dp.DMA1
            .ch7()
            .ndtr()
            .write(|w| w.ndt().set(sentence_length as u16));
        unsafe {
            // Copy sentence from buffer to SENTENCE array
            let mut buffer_idx = SENTENCE_BEGIN as usize;
            for b in SENTENCE.iter_mut().take(sentence_length as usize) {
                *b = NMEA_BUFFER[buffer_idx];
                buffer_idx = (buffer_idx + 1) & (NMEA_BUFFER_SIZE - 1);
            }
        }
        dp.DMA1.ch7().cr().write(|w| {
            w.minc()
                .set_bit()
                .dir()
                .from_memory()
                .tcie()
                .set_bit()
                .en()
                .set_bit()
        });

        // Update sentence position
        unsafe {
            SENTENCE_BEGIN = (SENTENCE_BEGIN + sentence_length) & ((NMEA_BUFFER_SIZE as u32) - 1);
        }

        dp.USART1
            .icr()
            .write(|w| w.cmcf().clear_bit_by_one().orecf().clear_bit_by_one());
    }
    if dp.USART1.isr().read().ore().bit_is_set() {
        dp.USART1.icr().write(|w| w.orecf().clear_bit_by_one());
    }
}

#[entry]
fn main() -> ! {
    // Defaults to 4MHz clock

    let (_cp, dp) = unsafe {
        CORE_PERIPHERALS = Some(CorePeripherals::take().unwrap());
        DEVICE_PERIPHERALS = Some(DevicePeripherals::take().unwrap());
        (
            CORE_PERIPHERALS.as_mut().unwrap(),
            DEVICE_PERIPHERALS.as_mut().unwrap(),
        )
    };

    // Set system clock speed to 200 kHz
    dp.RCC
        .cr()
        .write(|w| w.msirange().range400k().msirgsel().set_bit());

    dp.RCC.ahb1enr().write(|w| w.dma1en().set_bit());
    dp.RCC
        .ahb2enr()
        .write(|w| w.gpioaen().set_bit().gpioben().set_bit());
    dp.RCC.apb1enr1().write(|w| w.usart2en().set_bit());
    dp.RCC.apb2enr().write(|w| w.usart1en().set_bit());

    // GPIOA configuration: AF table p.56
    // USART2: A2(TX), A3(RX)
    dp.GPIOA
        .moder()
        .write(|w| w.moder2().alternate().moder3().alternate());
    dp.GPIOA
        .ospeedr()
        .write(|w| w.ospeedr2().low_speed().ospeedr3().low_speed());
    dp.GPIOA.afrl().write(|w| w.afrl2().af7().afrl3().af7());

    // GPIOB configuration: AF table p.56
    // USART1(GNSS): B5(ON/OFF), B6(TX), B7(RX)
    dp.GPIOB.moder().write(|w| {
        w.moder5()
            .output()
            .moder6()
            .alternate()
            .moder7()
            .alternate()
    });
    dp.GPIOB.otyper().write(|w| w.ot5().push_pull());
    dp.GPIOB.bsrr().write(|w| w.br5().set_bit());
    dp.GPIOB.ospeedr().write(|w| {
        w.ospeedr5()
            .low_speed()
            .ospeedr6()
            .low_speed()
            .ospeedr7()
            .low_speed()
    });
    dp.GPIOB.afrl().write(|w| w.afrl6().af7().afrl7().af7());

    // DMA channel selection p.299
    // C4: USART1 TX
    // C5: USART1 RX
    // C7: USART2 TX
    dp.DMA1
        .cselr()
        .write(|w| w.c4s().map2().c5s().map2().c7s().map2());

    // DMA C4 USART1 TX: enable immediately
    dp.DMA1
        .ch4()
        .par()
        .write(|w| unsafe { w.pa().bits(USART1_TDR) });
    dp.DMA1
        .ch4()
        .mar()
        .write(|w| unsafe { w.ma().bits(GNSS_SET_SENTENCES.as_ptr() as u32) });
    dp.DMA1
        .ch4()
        .ndtr()
        .write(|w| w.ndt().set(GNSS_SET_SENTENCES.len() as u16));
    dp.DMA1.ch4().cr().write(|w| {
        w.minc()
            .set_bit()
            .dir()
            .from_memory()
            .tcie()
            .set_bit()
            .en()
            .set_bit()
    });

    // DMA C5 USART1 RX: enable immediately
    dp.DMA1
        .ch5()
        .par()
        .write(|w| unsafe { w.pa().bits(USART1_RDR) });
    dp.DMA1
        .ch5()
        .mar()
        .write(|w| unsafe { w.ma().bits(NMEA_BUFFER.as_ptr() as u32) });
    dp.DMA1
        .ch5()
        .ndtr()
        .write(|w| w.ndt().set(unsafe { NMEA_BUFFER }.len() as u16));
    dp.DMA1
        .ch5()
        .cr()
        .write(|w| w.minc().set_bit().circ().set_bit().en().set_bit());

    // DMA C7 USART2 TX
    dp.DMA1
        .ch7()
        .par()
        .write(|w| unsafe { w.pa().bits(USART2_TDR) });
    dp.DMA1
        .ch7()
        .mar()
        .write(|w| unsafe { w.ma().bits(SENTENCE.as_ptr() as u32) });

    // USART1: Enable TX, RX and character match interrupt
    dp.USART1.brr().write(|w| w.brr().set(42)); // 4MHz / 9600 approx. 417
    dp.USART1.cr2().write(|w| w.add().set(b'\n'));
    dp.USART1
        .cr3()
        .write(|w| w.dmar().set_bit().dmat().set_bit());
    dp.USART1.cr1().write(|w| {
        w.ue()
            .set_bit()
            .re()
            .set_bit()
            .te()
            .set_bit()
            .cmie()
            .set_bit()
    });

    // USART2: Enable TX
    dp.USART2.brr().write(|w| w.brr().set(28)); // 4MHz / 14400 approx. 278
    dp.USART2.cr3().write(|w| w.dmat().set_bit());
    dp.USART2.cr1().write(|w| w.ue().set_bit().te().set_bit());

    // GNSS set ON
    dp.GPIOB.bsrr().write(|w| w.bs5().set_bit());

    unsafe {
        // Unmask NVIC global interrupts
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH4); // TC
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH7); // TC
        cortex_m::peripheral::NVIC::unmask(Interrupt::USART1); // CM
    }

    loop {
        asm::wfi();
    }
}
