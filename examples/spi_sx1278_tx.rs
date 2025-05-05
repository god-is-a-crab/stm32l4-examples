#![no_std]
#![no_main]

use cortex_m::{Peripherals as CorePeripherals, asm};
use cortex_m_rt::entry;
use heapless::spsc::Queue;
use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use stm32l4::stm32l4x2::{Interrupt, Peripherals as DevicePeripherals, interrupt};

const USART2_TDR: u32 = 0x4000_4428;
const SPI1_DR: u32 = 0x4001_300C;

static mut CORE_PERIPHERALS: Option<CorePeripherals> = None;
static mut DEVICE_PERIPHERALS: Option<DevicePeripherals> = None;

// Commands
const DIO_MAPPING: [u8; 2] = [0xc0, 0x40];
const SLEEP_MODE: [u8; 2] = [0x81, 0x88]; // Set SLEEP and LoRA mode
const W_OP_MODE: [u8; 2] = [0x81, 0x89]; // Set STBY mode
const TX_MODE: [u8; 2] = [0x81, 0x8b];
const R_OP_MODE: [u8; 2] = [0x01, 0x00];
const W_PAYLOAD_LENGTH: [u8; 2] = [0xA2, 0x26]; // Set payload length to 38
const R_PAYLOAD_LENGTH: [u8; 2] = [0x22, 0];
const R_PA_CONFIG: [u8; 2] = [0x09, 0x00];
const W_PA_CONFIG: [u8; 2] = [0x80 | 0x09, 0xcf]; // PA output PA_BOOST
const R_FIFO: [u8; 33] = [0; 33];
const R_FIFO_ADDR_PTR: [u8; 2] = [0x0D, 0x00];
const W_FIFO_ADDR_PTR: [u8; 2] = [0b1000_0000 | 0x0D, 0x80];
const R_IRQ_FLAGS: [u8; 2] = [0x12, 0x00];
const CLEAR_TXDONE: [u8; 2] = [0x92, 0x08];
const W_FRF_H: [u8; 2] = [0x06 | 0x80, 0x7B];
const W_FRF_M: [u8; 2] = [0x07 | 0x80, 0xC0];
const W_FRF_L: [u8; 2] = [0x08 | 0x80, 0x14];
const R_FRF_H: [u8; 2] = [0x06, 0];
const R_FRF_M: [u8; 2] = [0x07, 0];
const R_FRF_L: [u8; 2] = [0x08, 0];
const R_PREAMBLE_H: [u8; 2] = [0x20, 0];
const R_PREAMBLE_L: [u8; 2] = [0x21, 0];
const R_SYNC_WORD: [u8; 2] = [0x39, 0];
const R_CONFIG1: [u8; 2] = [0x1d, 0];
const R_CONFIG2: [u8; 2] = [0x1e, 0];
const W_TX_PAYLOAD: [u8; 39] = [
    0b1000_0000,
    b't',
    b'h',
    b'e',
    b' ',
    b'l',
    b'a',
    b'z',
    b'y',
    b' ',
    b'f',
    b'o',
    b'x',
    b' ',
    b'j',
    b'u',
    b'm',
    b'p',
    b'e',
    b'd',
    b' ',
    b'o',
    b'v',
    b'e',
    b'r',
    b' ',
    b't',
    b'h',
    b'e',
    b' ',
    b'b',
    b'r',
    b'o',
    b'w',
    b'n',
    b' ',
    b'f',
    b'o',
    b'x',
];

static mut SPI1_RX_BUFFER: [u8; 33] = [0; 33];
static mut COMMANDS: Queue<&[u8], 64> = Queue::new();

#[inline]
fn send_command(command: &[u8], dp: &mut DevicePeripherals) {
    // Write memory address for SPI1 TX
    dp.DMA1
        .ch3()
        .mar()
        .write(|w| unsafe { w.bits(command.as_ptr() as u32) });
    let transfer_size = command.len() as u32;
    // Set DMA transfer size for SPI1 RX, TX, USART2 TX
    dp.DMA1
        .ch2()
        .ndtr()
        .write(|w| unsafe { w.bits(transfer_size) });
    dp.DMA1
        .ch3()
        .ndtr()
        .write(|w| unsafe { w.bits(transfer_size) });
    dp.DMA1
        .ch7()
        .ndtr()
        .write(|w| unsafe { w.bits(transfer_size) });

    // Enable DMA for SPI1 RX, TX
    dp.DMA1
        .ch2()
        .cr()
        .write(|w| w.minc().set_bit().tcie().set_bit().en().set_bit());
    dp.DMA1.ch3().cr().write(|w| {
        w.minc()
            .set_bit()
            .dir()
            .set_bit()
            .tcie()
            .set_bit()
            .en()
            .set_bit()
    });
    // Enable SPI1
    dp.SPI1.cr1().write(|w| w.mstr().set_bit().spe().enabled());
}

#[interrupt]
fn USART2() {
    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    if dp.USART2.isr().read().rxne().bit_is_set() {
        let received_byte = dp.USART2.rdr().read().rdr().bits();

        match received_byte {
            97 => {
                // a
                send_command(&R_FIFO_ADDR_PTR, dp);
            }
            98 => {
                // b
                send_command(&W_FIFO_ADDR_PTR, dp);
            }
            99 => {
                // c
                send_command(&R_FIFO, dp);
            }
            100 => {
                // d
                send_command(&W_TX_PAYLOAD, dp);
            }
            101 => {
                // e
                send_command(&R_OP_MODE, dp);
            }
            102 => {
                // f
                send_command(&TX_MODE, dp);
            }
            103 => {
                // g
                send_command(&R_IRQ_FLAGS, dp);
            }
            104 => {
                // h
                send_command(&CLEAR_TXDONE, dp);
            }
            _ => (),
        }
    }
    if dp.USART2.isr().read().ore().bit_is_set() {
        dp.USART2.icr().write(|w| w.orecf().clear_bit_by_one());
    }
}

#[interrupt]
fn DMA1_CH7() {
    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();
    #[allow(static_mut_refs)]
    let commands = unsafe { &mut COMMANDS };

    if dp.DMA1.isr().read().tcif7().bit_is_set() {
        // Disable DMA Ch7
        dp.DMA1.ch7().cr().write(|w| {
            w.minc()
                .set_bit()
                .dir()
                .set_bit()
                .tcie()
                .set_bit()
                .en()
                .clear_bit()
        });
        // Send next command
        if let Some(command) = commands.dequeue() {
            send_command(command, dp);
        }
        dp.DMA1.ifcr().write(|w| w.ctcif7().set_bit());
    }
}

/// SPI1 RX DMA - nRF24L01 command complete
#[interrupt]
fn DMA1_CH2() {
    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    if dp.DMA1.isr().read().tcif2().bit_is_set() {
        // Disable DMA Ch2
        dp.DMA1
            .ch2()
            .cr()
            .write(|w| w.minc().set_bit().tcie().set_bit().en().clear_bit());

        // Disable SPI1
        dp.SPI1
            .cr1()
            .write(|w| w.mstr().set_bit().spe().clear_bit());

        // Enable USART2 TX DMA
        dp.DMA1.ch7().cr().write(|w| {
            w.minc()
                .set_bit()
                .dir()
                .set_bit()
                .tcie()
                .set_bit()
                .en()
                .set_bit()
        });
        dp.DMA1.ifcr().write(|w| w.ctcif2().set_bit());
    }
}

/// SPI1 TX DMA - Send nRF24L01 command
#[interrupt]
fn DMA1_CH3() {
    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();
    if dp.DMA1.isr().read().tcif3().bit_is_set() {
        // Disable DMA Ch3
        dp.DMA1.ch3().cr().write(|w| {
            w.minc()
                .set_bit()
                .dir()
                .set_bit()
                .tcie()
                .set_bit()
                .en()
                .clear_bit()
        });
        dp.DMA1.ifcr().write(|w| w.ctcif3().set_bit());
    }
}

/// CE 10us pulse
#[interrupt]
fn TIM6_DACUNDER() {
    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    if dp.TIM6.sr().read().uif().bit_is_set() {
        dp.GPIOA.bsrr().write(|w| w.br0().set_bit());
        dp.TIM6.sr().write(|w| w.uif().clear_bit());
    }
}

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let cp = CorePeripherals::take().unwrap();
    let dp = DevicePeripherals::take().unwrap();

    // Set system clock speed to 200 khz
    dp.RCC
        .cr()
        .write(|w| w.msirange().range200k().msirgsel().set_bit());

    // Enable peripheral clocks
    dp.RCC.ahb1enr().write(|w| w.dma1en().set_bit());
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());
    dp.RCC
        .apb1enr1()
        .write(|w| w.usart2en().enabled().tim6en().set_bit());
    dp.RCC.apb2enr().write(|w| w.spi1en().set_bit());

    // USART2: A2 (TX), A3 (RX) as AF 7
    // SPI1: A4 (NSS), A5 (SCK), A6 (MISO), A7 (MOSI) as AF 5
    // GPIO: A0 (CE)
    dp.GPIOA.moder().write(|w| {
        w.moder0()
            .output()
            .moder1()
            .input()
            .moder2()
            .alternate()
            .moder3()
            .alternate()
            .moder4()
            .alternate()
            .moder5()
            .alternate()
            .moder6()
            .alternate()
            .moder7()
            .alternate()
    });
    dp.GPIOA.otyper().write(|w| w.ot0().push_pull());
    // NSS output is active low
    dp.GPIOA.pupdr().write(|w| w.pupdr4().pull_up());
    dp.GPIOA.ospeedr().write(|w| {
        w.ospeedr5()
            .medium_speed()
            .ospeedr6()
            .medium_speed()
            .ospeedr7()
            .medium_speed()
    });
    dp.GPIOA.afrl().write(|w| {
        w.afrl2()
            .af7()
            .afrl3()
            .af7()
            .afrl4()
            .af5()
            .afrl5()
            .af5()
            .afrl6()
            .af5()
            .afrl7()
            .af5()
    });

    // DMA channel selection
    dp.DMA1
        .cselr()
        .write(|w| w.c2s().map1().c3s().map1().c7s().map2());

    // DMA channel 7 USART2 TX
    dp.DMA1
        .ch7()
        .par()
        .write(|w| unsafe { w.pa().bits(USART2_TDR) });
    #[allow(static_mut_refs)]
    dp.DMA1
        .ch7()
        .mar()
        .write(|w| unsafe { w.ma().bits(SPI1_RX_BUFFER.as_ptr() as u32) });

    // DMA channel 2 SPI1 RX
    dp.DMA1
        .ch2()
        .par()
        .write(|w| unsafe { w.pa().bits(SPI1_DR) });
    #[allow(static_mut_refs)]
    dp.DMA1
        .ch2()
        .mar()
        .write(|w| unsafe { w.ma().bits(SPI1_RX_BUFFER.as_ptr() as u32) });

    // DMA channel 3 SPI1 TX
    dp.DMA1
        .ch3()
        .par()
        .write(|w| unsafe { w.pa().bits(SPI1_DR) });

    // Set 11us interval
    dp.TIM6.arr().write(|w| unsafe { w.arr().bits(3) }); // 200khz * 11us approx. 3

    // Enable TIM6 update interrupt
    dp.TIM6.dier().write(|w| w.uie().set_bit());

    // USART2: Configure baud rate 9600
    dp.USART2.brr().write(|w| unsafe { w.bits(21) }); // 200khz / 9600 approx. 21

    // USART2: enable DMA TX
    dp.USART2.cr3().write(|w| w.dmat().set_bit());
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

    // SPI1: Set FIFO reception threshold to 1/4, enable slave select output, enable DMA
    dp.SPI1.cr2().write(|w| {
        w.frxth()
            .set_bit()
            .ssoe()
            .enabled()
            .txdmaen()
            .set_bit()
            .rxdmaen()
            .set_bit()
    });

    // Initialization commands
    #[allow(static_mut_refs)]
    let commands = unsafe { &mut COMMANDS };

    unsafe {
        commands.enqueue_unchecked(&SLEEP_MODE);
        commands.enqueue_unchecked(&W_OP_MODE);
        commands.enqueue_unchecked(&R_OP_MODE);
        commands.enqueue_unchecked(&W_PAYLOAD_LENGTH);
        commands.enqueue_unchecked(&R_PAYLOAD_LENGTH);
        commands.enqueue_unchecked(&W_PA_CONFIG);
        commands.enqueue_unchecked(&R_PA_CONFIG);
        commands.enqueue_unchecked(&W_FIFO_ADDR_PTR);
        commands.enqueue_unchecked(&W_TX_PAYLOAD);
        commands.enqueue_unchecked(&W_FRF_H);
        commands.enqueue_unchecked(&W_FRF_M);
        commands.enqueue_unchecked(&W_FRF_L);
        commands.enqueue_unchecked(&R_FRF_H);
        commands.enqueue_unchecked(&R_FRF_M);
        commands.enqueue_unchecked(&R_FRF_L);
        commands.enqueue_unchecked(&R_CONFIG1);
        commands.enqueue_unchecked(&R_CONFIG2);
        commands.enqueue_unchecked(&R_PREAMBLE_H);
        commands.enqueue_unchecked(&R_PREAMBLE_L);
        commands.enqueue_unchecked(&R_SYNC_WORD);

        // Unmask NVIC global interrupts
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH2);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH3);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH7);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM6_DACUNDER);
        cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
        CORE_PERIPHERALS = Some(cp);
        DEVICE_PERIPHERALS = Some(dp);
    }

    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    send_command(&DIO_MAPPING, dp);

    loop {
        asm::wfi();
    }
}
