#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, i2c, peripherals, Config, spi, i2s};
use embassy_stm32::dcmi::{self, *};
use embassy_stm32::gpio::{Level, Output, Speed, Pull, Input};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::i2c::I2c;
use embassy_stm32::i2s::I2S;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::{khz, mhz};
use embassy_time::{Duration, Timer};
use fmt::info;

bind_interrupts!(struct Irqs {
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>; // I2C2 event
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>; // I2C2 error
    DCMI_PSSI => dcmi::InterruptHandler<peripherals::DCMI>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV8), 
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; 
        config.rcc.ahb_pre = AHBPrescaler::DIV2; 
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV2; 
        config.rcc.apb3_pre = APBPrescaler::DIV2;
        config.rcc.apb4_pre = APBPrescaler::DIV2;
        config.rcc.voltage_scale = VoltageScale::Scale1;
    }
    let p = embassy_stm32::init(config);

    defmt::info!("Booting...");

    // let mco = Mco::new(p.MCO1, p.PA8, Mco1Source::HSI, McoPrescaler::DIV3);

    let mut led_r = Output::new(p.PD8, Level::Low, Speed::Low);
    let mut led_g = Output::new(p.PD9, Level::Low, Speed::Low);
    let mut led_b = Output::new(p.PD10, Level::Low, Speed::Low);

    let i2c_bus = I2c::new(
        p.I2C2, // peri
        p.PB10, // scl
        p.PB11, // sda
        Irqs,   // int binding
        p.DMA1_CH1,
        p.DMA1_CH2,
        khz(100),
        Default::default(),
    );

    let mut config = spi::Config::default();
    config.frequency = mhz(1); // TODO
    let dac_spi = Spi::new_txonly(
        p.SPI3,
        p.PC10, // sck
        p.PC12, // mosi
        p.DMA1_CH3, // txdma
        config,
    );

    let mut nrec_int = ExtiInput::new(p.PB15, p.EXTI15, Pull::None);
    let mut hdmi_int = ExtiInput::new(p.PB14, p.EXTI14, Pull::None);

    let config = dcmi::Config::default();
    let mut cam = Dcmi::new_es_8bit(
        p.DCMI, p.DMA1_CH0, Irqs, p.PC6, p.PA10, p.PB13, p.PC9, p.PE4, p.PD3, p.PE5, p.PE6, p.PA6, config, 
    );

    let mut config = i2s::Config::default();
    config.master_clock = false;
    config.mode = i2s::Mode::Slave;
    let mut mic_buffer = [0x00_u16; 128];

    let mut mic = I2S::new_rxonly( // PA3 is unused and not mapped to MCLK in HW
        p.SPI6, p.PB4, p.PA0, p.PA5, p.PA3, p.BDMA2_CH0, &mut mic_buffer, khz(44), config,
    );

    let mut led = Output::new(p.PB7, Level::High, Speed::Low);

    loop {
        info!("Hello, World!");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}