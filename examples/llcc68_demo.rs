#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![allow(static_mut_refs)]

use core::{arch::asm, mem::MaybeUninit, panic::PanicInfo};
use cortex_m::{Peripherals as CorePeripherals, asm};
use cortex_m_rt::{entry, pre_init};
use panic_semihosting as _;
use static_fifo_queue::Queue;
use stm32l4::stm32l4x2::{Interrupt, Peripherals as DevicePeripherals, interrupt};
use sx126x_spi_buffers::commands::{
    Bw, Cr, GetDeviceErrors, GetPacketType, GetStatus, HeaderType, InvertIq, Irq, PacketType,
    RampTime, SetBufferBaseAddress, SetDio2AsRfSwitchCtrl, SetDio3AsTcxoCtrl, SetDioIrqParams,
    SetModulationParamsLora, SetPaConfig, SetPacketParams, SetPacketType, SetRfFrequency,
    SetStandby, SetTx, SetTxParams, Sf, SpiDescriptor, StdbyConfig, TcxoVoltage, WriteBuffer,
};

// Peripherals
static mut CORE_PERIPHERALS: MaybeUninit<CorePeripherals> = MaybeUninit::uninit();
static mut DEVICE_PERIPHERALS: MaybeUninit<DevicePeripherals> = MaybeUninit::uninit();

/// Device address
const ADDRESS: u32 = 0x6310C;

// Constants - peripheral register addresses for DMA
const SPI1_DR: u32 = 0x4001_300C;

/// The queue of SPI commands to run
static mut COMMANDS: Queue<&SpiDescriptor, 32> = Queue::new();

// SX126X buffers
static mut SET_STANDBY_XOSC_BUFS: SetStandby = SetStandby::new(StdbyConfig::StdbyXosc);
static mut SET_STANDBY_XOSC: SpiDescriptor = unsafe { SET_STANDBY_XOSC_BUFS.descriptor() };

static mut GET_PACKET_TYPE_BUFS: GetPacketType = GetPacketType::new();
static mut GET_PACKET_TYPE: SpiDescriptor = unsafe { GET_PACKET_TYPE_BUFS.descriptor() };

static mut SET_PACKET_TYPE_BUFS: SetPacketType = SetPacketType::new(PacketType::Lora);
static mut SET_PACKET_TYPE: SpiDescriptor = unsafe { SET_PACKET_TYPE_BUFS.descriptor() };

static mut SET_RF_FREQUENCY_BUFS: SetRfFrequency = SetRfFrequency::new(455081984);
static mut SET_RF_FREQUENCY: SpiDescriptor = unsafe { SET_RF_FREQUENCY_BUFS.descriptor() };

static mut SET_PA_CONFIG_BUFS: SetPaConfig = SetPaConfig::new(0x04, 0x07);
static mut SET_PA_CONFIG: SpiDescriptor = unsafe { SET_PA_CONFIG_BUFS.descriptor() };

static mut SET_TX_PARAMS_BUFS: SetTxParams = SetTxParams::new(22, RampTime::Ramp200U);
static mut SET_TX_PARAMS: SpiDescriptor = unsafe { SET_TX_PARAMS_BUFS.descriptor() };

static mut SET_BUFFER_ADDRESS_BUFS: SetBufferBaseAddress = SetBufferBaseAddress::new(0x00, 0x80);
static mut SET_BUFFER_ADDRESS: SpiDescriptor = unsafe { SET_BUFFER_ADDRESS_BUFS.descriptor() };

static mut SET_MOD_PARAMS_BUFS: SetModulationParamsLora =
    SetModulationParamsLora::new(Sf::Sf10, Bw::Bw125, Cr::Cr4_5, false);
static mut SET_MOD_PARAMS: SpiDescriptor = unsafe { SET_MOD_PARAMS_BUFS.descriptor() };

static mut PACKET: WriteBuffer<5> = WriteBuffer::new(0x00, [0x63, 0x10, 0xC0]);
static mut WRITE_PACKET: SpiDescriptor = unsafe { PACKET.descriptor() };

static mut SET_PACKET_PARAMS_BUFS: SetPacketParams =
    SetPacketParams::new(8, HeaderType::VariableLength, 3, false, InvertIq::Standard);
static mut SET_PACKET_PARAMS: SpiDescriptor = unsafe { SET_PACKET_PARAMS_BUFS.descriptor() };

static mut SET_DIO_IRQ_PARAMS_BUFS: SetDioIrqParams = SetDioIrqParams::new(
    Irq::new().with_tx_done(true),
    Irq::new().with_tx_done(true),
    Irq::new(),
    Irq::new(),
);
static mut SET_DIO_IRQ_PARAMS: SpiDescriptor = unsafe { SET_DIO_IRQ_PARAMS_BUFS.descriptor() };

static mut SET_DIO2_AS_RF_SWITCH_CTRL_BUFS: SetDio2AsRfSwitchCtrl =
    SetDio2AsRfSwitchCtrl::new(true);
static mut SET_DIO2_AS_RF_SWITCH_CTRL: SpiDescriptor =
    unsafe { SET_DIO2_AS_RF_SWITCH_CTRL_BUFS.descriptor() };

// static mut SET_DIO3_AS_TCXO_CTRL_BUFS: SetDio3AsTcxoCtrl =
//     SetDio3AsTcxoCtrl::new(TcxoVoltage::V1_8, 0x000140);
// static mut SET_DIO3_AS_TCXO_CTRL: SpiDescriptor =
//     unsafe { SET_DIO3_AS_TCXO_CTRL_BUFS.descriptor() };

static mut SET_TX_BUFS: SetTx = SetTx::new(0);
static mut SET_TX: SpiDescriptor = unsafe { SET_TX_BUFS.descriptor() };

static WRITE_SYNC_WORD_TX: [u8; 5] = [0x0D, 0x07, 0x40, 0x14, 0x24];
static mut WRITE_SYNC_WORD_RX: [u8; 5] = [0; 5];
static mut WRITE_SYNC_WORD: SpiDescriptor = SpiDescriptor {
    tx_buf_ptr: WRITE_SYNC_WORD_TX.as_ptr(),
    rx_buf_ptr: unsafe { WRITE_SYNC_WORD_RX.as_mut_ptr() },
    transfer_length: 5,
};

static mut GET_DEVICE_ERRORS_BUFS: GetDeviceErrors = GetDeviceErrors::new();
static mut GET_DEVICE_ERRORS: SpiDescriptor = unsafe { GET_DEVICE_ERRORS_BUFS.descriptor() };

static mut GET_STATUS_BUFS: GetStatus = GetStatus::new();
static mut GET_STATUS: SpiDescriptor = unsafe { GET_STATUS_BUFS.descriptor() };

/// Initiates a sequence of SPI1 transfers over DMA1_CH2(RX) and DMA1_CH3(TX).
/// The sequence of commands starts with the command given to this function
/// and continues with commands from [`COMMANDS`] until no commands remain.
#[inline(always)]
fn send_command(command: &SpiDescriptor, dp: &mut DevicePeripherals) {
    // Write memory address for SPI1 RX DMA channel
    dp.DMA1
        .ch2()
        .mar()
        .write(|w| unsafe { w.ma().bits(command.rx_buf_ptr as u32) });
    // Write memory address for SPI1 TX DMA channel
    dp.DMA1
        .ch3()
        .mar()
        .write(|w| unsafe { w.ma().bits(command.tx_buf_ptr as u32) });

    // Set DMA transfer size for SPI1 RX/TX
    dp.DMA1
        .ch2()
        .ndtr()
        .write(|w| w.ndt().set(command.transfer_length));
    dp.DMA1
        .ch3()
        .ndtr()
        .write(|w| w.ndt().set(command.transfer_length));

    // Enable DMA for SPI1 RX, TX
    dp.DMA1
        .ch2()
        .cr()
        .write(|w| w.minc().set_bit().tcie().set_bit().en().set_bit());
    dp.DMA1.ch3().cr().write(|w| {
        w.minc()
            .set_bit()
            .dir()
            .from_memory()
            .tcie()
            .set_bit()
            .en()
            .set_bit()
    });
    // Enable SPI1
    dp.SPI1.cr1().write(|w| w.mstr().set_bit().spe().enabled());
}

/// DIO0 handler (rising edge). Either RXDONE or TXDONE will be mapped at one time.
#[interrupt]
fn EXTI1() {
    let dp = unsafe { &mut *DEVICE_PERIPHERALS.as_mut_ptr() };

    if dp.EXTI.pr1().read().pr1().bit_is_set() {
        dp.EXTI.pr1().write(|w| w.pr1().clear_bit_by_one());
    }
}

/// SPI1 RX DMA TC interrupt - SX1278 command complete
#[interrupt]
fn DMA1_CH2() {
    let dp = unsafe { &mut *DEVICE_PERIPHERALS.as_mut_ptr() };
    let commands = unsafe { &mut COMMANDS };

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
        }
        dp.DMA1.ifcr().write(|w| w.ctcif2().set_bit());
    }
}

/// SPI1 TX DMA TC interrupt - Send SX1278 command
#[interrupt]
fn DMA1_CH3() {
    let dp = unsafe { &mut *DEVICE_PERIPHERALS.as_mut_ptr() };
    if dp.DMA1.isr().read().tcif3().bit_is_set() {
        // Disable DMA Ch3
        dp.DMA1.ch3().cr().write(|w| {
            w.minc()
                .set_bit()
                .dir()
                .from_memory()
                .tcie()
                .set_bit()
                .en()
                .clear_bit()
        });
        dp.DMA1.ifcr().write(|w| w.ctcif3().set_bit());
    }
}

/// USART1(GNSS) TX DMA TC interrupt
#[interrupt]
fn DMA1_CH4() {
    let dp = unsafe { &mut *DEVICE_PERIPHERALS.as_mut_ptr() };

    if dp.DMA1.isr().read().tcif4().bit_is_set() {
        dp.DMA1.ch4().cr().write(|w| w.en().clear_bit());
        // Poll transmission complete
        while dp.USART1.isr().read().tc().bit_is_clear() {}
        // Disable transmitter
        dp.USART1
            .cr1()
            .write(|w| w.ue().clear_bit().te().clear_bit());
        // Wait for GNSS module to initialize

        dp.DMA1.ifcr().write(|w| w.ctcif4().set_bit());
    }
}

/// Wait for GNSS initialization -> continue remaining initialization
#[interrupt]
fn TIM1_BRK_TIM15() {
    let dp: &mut DevicePeripherals = unsafe { &mut *DEVICE_PERIPHERALS.as_mut_ptr() };

    if dp.TIM15.sr().read().uif().bit_is_set() {
        // Transmit initial handshake
        unsafe {
            // send_command(&sx126x::SET_PACKET_TYPE, dp);
        }

        // Disable TIM15 clock
        cortex_m::peripheral::NVIC::mask(Interrupt::TIM1_BRK_TIM15);
        dp.RCC.apb2enr().write(|w| w.spi1en().set_bit());
        dp.TIM15.sr().write(|w| w.uif().clear_bit());
    }
}

#[entry]
fn main() -> ! {
    let dp = unsafe { DEVICE_PERIPHERALS.write(DevicePeripherals::take().unwrap()) };

    // region: ==== RCC configuration ====
    // Set system clock speed to 200 kHz
    dp.RCC
        .cr()
        .write(|w| w.msirange().range200k().msirgsel().set_bit());

    // Enable peripheral clocks
    dp.RCC.ahb1enr().write(|w| w.dma1en().set_bit());
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());
    dp.RCC.apb1enr1().write(|w| w.tim6en().set_bit());
    dp.RCC.apb2enr().write(|w| w.spi1en().set_bit());

    // region: ==== TIM6 configuration ====
    // For ACK timeouts
    dp.TIM6.psc().write(|w| w.psc().set(1999)); // 200kHz / (1999 + 1) = 100 Hz (10ms per tick)
    dp.TIM6.arr().write(|w| w.arr().set(23)); // Works at 230ms, +10ms for sanity
    dp.TIM6.cr1().write(|w| w.urs().counter_only());
    dp.TIM6.egr().write(|w| w.ug().set_bit());
    dp.TIM6.dier().write(|w| w.uie().set_bit());

    // region: ==== TIM15 configuration ====
    // Wait for GNSS to initialize
    dp.TIM15.psc().write(|w| w.psc().set(1999)); // 200kHz / (1999 + 1) = 100 Hz (10ms per tick)
    dp.TIM15.arr().write(|w| w.arr().set(1)); // Works at 2 ticks
    dp.TIM15.cr1().write(|w| w.urs().counter_only());
    dp.TIM15.egr().write(|w| w.ug().set_bit());
    dp.TIM15.dier().write(|w| w.uie().set_bit());

    // region: ==== GPIOA configuration ====
    // DIO0: A1
    // SPI1(SX1278): A4(NSS), A5(SCK), A6(MISO), A7(MOSI)
    dp.GPIOA.moder().write(|w| {
        w.moder0()
            .analog()
            .moder1()
            .input()
            .moder4()
            .alternate()
            .moder5()
            .alternate()
            .moder6()
            .alternate()
            .moder7()
            .alternate()
    });
    dp.GPIOA.pupdr().write(|w| w.pupdr4().pull_up());
    dp.GPIOA.ospeedr().write(|w| {
        w.ospeedr5()
            .medium_speed()
            .ospeedr6()
            .medium_speed()
            .ospeedr7()
            .medium_speed()
    });
    dp.GPIOA
        .afrl()
        .write(|w| w.afrl4().af5().afrl5().af5().afrl6().af5().afrl7().af5());

    // region: ==== DMA1 C2,C3 + SPI1 configuration ====
    // DMA channel selection p.299
    // C2: SPI1 RX
    // C3: SPI1 TX
    dp.DMA1.cselr().write(|w| w.c2s().map1().c3s().map1());

    // DMA C2 SPI1 RX
    dp.DMA1
        .ch2()
        .par()
        .write(|w| unsafe { w.pa().bits(SPI1_DR) });

    // DMA C3 SPI1 TX
    dp.DMA1
        .ch3()
        .par()
        .write(|w| unsafe { w.pa().bits(SPI1_DR) });

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

    // region: ==== EXTI configuration ====
    dp.EXTI.rtsr1().write(|w| w.tr1().set_bit());
    dp.EXTI.imr1().write(|w| w.mr1().set_bit());

    unsafe {
        COMMANDS.enqueue(&SET_STANDBY_XOSC);
        COMMANDS.enqueue(&GET_PACKET_TYPE);
        COMMANDS.enqueue(&SET_RF_FREQUENCY);
        COMMANDS.enqueue(&SET_PA_CONFIG);
        COMMANDS.enqueue(&SET_TX_PARAMS);
        COMMANDS.enqueue(&SET_BUFFER_ADDRESS);
        COMMANDS.enqueue(&SET_MOD_PARAMS);
        COMMANDS.enqueue(&GET_DEVICE_ERRORS);
        COMMANDS.enqueue(&WRITE_PACKET);
        COMMANDS.enqueue(&SET_PACKET_PARAMS);
        COMMANDS.enqueue(&SET_DIO_IRQ_PARAMS);
        COMMANDS.enqueue(&WRITE_SYNC_WORD);
        COMMANDS.enqueue(&SET_DIO2_AS_RF_SWITCH_CTRL);
        // COMMANDS.enqueue(&sx126x::SET_DIO3_AS_TCXO_CTRL);
        COMMANDS.enqueue(&SET_TX);

        // Unmask NVIC global interrupts
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM6_DACUNDER); // UIF - ACK timeout
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH2); // TC - SPI1 RX
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH3); // TC - SPI1 TX
        cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI1); // RT1, RT20
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM1_BRK_TIM15); // UIF - GNSS initialize
    }

    // Transmit initial handshake
    // dp.TIM15.cr1().write(|w| w.urs().counter_only().opm().set_bit().cen().set_bit());
    unsafe {
        send_command(&SET_PACKET_TYPE, dp);
    }

    loop {
        asm::wfi();
    }
}
