#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use cortex_m::{asm, Peripherals as CorePeripherals};
use cortex_m_rt::entry;
use heapless::spsc::Queue;
use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use stm32l4::stm32l4x2::{interrupt, Interrupt, Peripherals as DevicePeripherals};
use sx127x_commands::{
    commands::{ReadFifo, ReadSingle, WriteFifo, WriteSingle},
    registers,
};

const USART2_TDR: u32 = 0x4000_4428;
const SPI1_DR: u32 = 0x4001_300C;

static mut CORE_PERIPHERALS: Option<CorePeripherals> = None;
static mut DEVICE_PERIPHERALS: Option<DevicePeripherals> = None;

// Commands
const DIO0_TXDONE: [u8; 2] =
    WriteSingle(registers::RegDioMapping1::new().with_dio0_mapping(0b01)).bytes();
const DIO0_RXDONE: [u8; 2] =
    WriteSingle(registers::RegDioMapping1::new().with_dio0_mapping(0b00)).bytes();
const SET_LORA_MODE: [u8; 2] = WriteSingle(
    // LoRA mode can only be set in SLEEP mode
    registers::RegOpMode::new()
        .with_long_range_mode(registers::LongRangeMode::Lora)
        .with_mode(registers::Mode::Sleep),
)
.bytes();
const SET_STDBY_MODE: [u8; 2] = WriteSingle(
    registers::RegOpMode::new()
        .with_long_range_mode(registers::LongRangeMode::Lora)
        .with_mode(registers::Mode::Stdby),
)
.bytes();
const TX_MODE: [u8; 2] = WriteSingle(
    registers::RegOpMode::new()
        .with_long_range_mode(registers::LongRangeMode::Lora)
        .with_mode(registers::Mode::Tx),
)
.bytes();
const RX_MODE: [u8; 2] = WriteSingle(
    registers::RegOpMode::new()
        .with_long_range_mode(registers::LongRangeMode::Lora)
        .with_mode(registers::Mode::RxSingle),
)
.bytes();
const W_PAYLOAD_LENGTH: [u8; 2] =
    WriteSingle(registers::lora::RegPayloadLength::new().with_payload_length(3)).bytes();
const W_PA_CONFIG: [u8; 2] = WriteSingle(
    registers::RegPaConfig::new()
        .with_pa_select(registers::PaSelect::PaBoost)
        .with_output_power(15),
)
.bytes();
// const W_FRF_H: [u8; 2] = [0x06 | 0x80, 0x7B];
// const W_FRF_M: [u8; 2] = [0x07 | 0x80, 0xC0];
// const W_FRF_L: [u8; 2] = [0x08 | 0x80, 0x14];
const W_SYMB_TIMEOUT_LSB: [u8; 2] =
    WriteSingle(registers::lora::RegSymbTimeoutLsb::new().with_symb_timeout(0x16)).bytes();
const R_FIFO: [u8; 4] = ReadFifo::<3>::bytes();
const W_FIFO_ADDR_PTR_RX: [u8; 2] =
    WriteSingle(registers::lora::RegFifoAddrPtr::new().with_fifo_addr_ptr(0)).bytes();
const W_FIFO_ADDR_PTR_TX: [u8; 2] =
    WriteSingle(registers::lora::RegFifoAddrPtr::new().with_fifo_addr_ptr(0x80)).bytes();
const W_IRQ_FLAGS_MASK: [u8; 2] = WriteSingle(
    registers::lora::RegIrqFlagsMask::from_bits(0xFF)
        .with_rx_done_mask(false)
        .with_tx_done_mask(false),
)
.bytes();
const R_IRQ_FLAGS: [u8; 2] = ReadSingle::<registers::lora::RegIrqFlags>::bytes();
const CLEAR_IRQ: [u8; 2] = WriteSingle(
    registers::lora::RegIrqFlags::new()
        .with_rx_done(true)
        .with_tx_done(true),
)
.bytes();
static mut W_FIFO: [u8; 4] = WriteFifo([0x63, 0x10, 0xC0]).bytes();

static mut STATE: State = State::Start;
static mut SPI1_RX_BUFFER: [u8; 33] = [0; 33];
static mut COMMANDS: Queue<&[u8], 64> = Queue::new();

enum State {
    Start,
    ReadIrq,
    Rx,
}

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
            113 => {
                // q
                send_command(&W_FIFO_ADDR_PTR_TX, dp);
            }
            119 => {
                // w
                #[allow(static_mut_refs)]
                let payload = unsafe { W_FIFO.as_mut() };
                payload[3] = payload[3].wrapping_add(1);
                send_command(payload, dp);
            }
            101 => {
                // e
                send_command(&TX_MODE, dp);
            }
            114 => {
                // r
                send_command(&CLEAR_IRQ, dp);
            }
            _ => (),
        }
    }
    if dp.USART2.isr().read().ore().bit_is_set() {
        dp.USART2.icr().write(|w| w.orecf().set_bit());
    }
}

#[interrupt]
fn DMA1_CH7() {
    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

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
        dp.DMA1.ifcr().write(|w| w.ctcif7().set_bit());
    }
}

/// SPI1 RX DMA - nRF24L01 command complete
#[interrupt]
fn DMA1_CH2() {
    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();
    #[allow(static_mut_refs)]
    let rx_buf = unsafe { &SPI1_RX_BUFFER };
    #[allow(static_mut_refs)]
    let commands = unsafe { &mut COMMANDS };
    #[allow(static_mut_refs)]
    let state = unsafe { &mut STATE };

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

        // Send next command
        if let Some(command) = commands.dequeue() {
            send_command(command, dp);
        } else {
            match *state {
                State::ReadIrq => {
                    let irq_flags = rx_buf[1];
                    if irq_flags >> 3 & 1 == 1 {
                        // tx done
                        unsafe { commands.enqueue_unchecked(&DIO0_RXDONE) };
                        unsafe { commands.enqueue_unchecked(&W_FIFO_ADDR_PTR_RX) };
                        unsafe { commands.enqueue_unchecked(&RX_MODE) };
                        send_command(&CLEAR_IRQ, dp);
                        *state = State::Start;
                    } else if irq_flags >> 6 & 1 == 1 {
                        // rx done
                        unsafe { commands.enqueue_unchecked(&DIO0_TXDONE) };
                        unsafe { commands.enqueue_unchecked(&W_FIFO_ADDR_PTR_RX) };
                        unsafe { commands.enqueue_unchecked(&R_FIFO) };
                        send_command(&CLEAR_IRQ, dp);
                        *state = State::Rx;
                    }
                }
                State::Rx => {
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
                    *state = State::Start;
                }
                _ => {}
            }
        }

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

#[interrupt]
fn EXTI1() {
    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();
    #[allow(static_mut_refs)]
    let state = unsafe { &mut STATE };

    if dp.EXTI.pr1().read().pr1().bit_is_set() {
        // Get IRQ status
        // TXDONE and ack required -> Set RX
        // RXDONE -> read rx buffer
        send_command(&R_IRQ_FLAGS, dp);
        *state = State::ReadIrq;
        dp.EXTI.pr1().write(|w| w.pr1().clear_bit_by_one());
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
    dp.RCC
        .ahb2enr()
        .write(|w| w.gpioaen().set_bit().gpioben().set_bit());
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

    dp.EXTI.rtsr1().write(|w| w.tr1().set_bit());
    dp.EXTI.imr1().write(|w| w.mr1().set_bit());

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
        commands.enqueue_unchecked(&SET_LORA_MODE);
        commands.enqueue_unchecked(&SET_STDBY_MODE);
        commands.enqueue_unchecked(&W_PAYLOAD_LENGTH);
        commands.enqueue_unchecked(&W_PA_CONFIG);
        commands.enqueue_unchecked(&W_SYMB_TIMEOUT_LSB);
        commands.enqueue_unchecked(&W_IRQ_FLAGS_MASK);
        commands.enqueue_unchecked(&W_FIFO_ADDR_PTR_TX);
        #[allow(static_mut_refs)]
        commands.enqueue_unchecked(&W_FIFO);
        // commands.enqueue_unchecked(&W_FRF_H);
        // commands.enqueue_unchecked(&W_FRF_M);
        // commands.enqueue_unchecked(&W_FRF_L);

        // Unmask NVIC global interrupts
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH2);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH3);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH7);
        cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI1);
        cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
        CORE_PERIPHERALS = Some(cp);
        DEVICE_PERIPHERALS = Some(dp);
    }

    #[allow(static_mut_refs)]
    let dp = unsafe { DEVICE_PERIPHERALS.as_mut() }.unwrap();

    send_command(&DIO0_TXDONE, dp);

    loop {
        asm::wfi();
    }
}
