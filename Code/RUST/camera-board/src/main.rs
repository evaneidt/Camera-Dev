#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, peripherals, Config};
use embassy_stm32::dcmi::{self, *};
use embassy_stm32::gpio::{Level, Output, Speed, Pull, Input};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::i2s::{self, I2S};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::ospi::{self, Ospi};
use embassy_stm32::time::{hz, khz, mhz};
use embassy_time::{Duration, Timer};
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use fmt::info;

bind_interrupts!(struct Irqs {
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>; // I2C2 event
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>; // I2C2 error
    DCMI_PSSI => dcmi::InterruptHandler<peripherals::DCMI>;
});

static OSD_MUTEX: Mutex<ThreadModeRawMutex, u32> = Mutex::new(0);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some( Hse {
            freq: mhz(50), 
            mode: HseMode::Bypass,
        });
        config.rcc.ls = LsConfig{
            rtc: RtcClockSource::LSE,
            lsi: false,
            lse: Some( LseConfig{frequency: hz(32_768), mode: LseMode::Bypass}),
        };
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
    let p: embassy_stm32::Peripherals = embassy_stm32::init(config);

    defmt::info!("Booting...");

    let mut led_r = Output::new(p.PD8, Level::Low, Speed::Low);
    let mut led_g = Output::new(p.PD9, Level::Low, Speed::Low);
    let mut led_b = Output::new(p.PD10, Level::Low, Speed::Low);

    let mut osd_en: Output<'_> = Output::new(p.PB5, Level::Low, Speed::Low);
    let mut osd_a0: Output<'_> = Output::new(p.PB6, Level::Low, Speed::Low);
    let mut osd_a1: Output<'_> = Output::new(p.PB7, Level::Low, Speed::Low);
    let mut osd_a2: Output<'_> = Output::new(p.PB8, Level::Low, Speed::Low);

    let mut i2s_mclk_ctrl: Output<'_> = Output::new(p.PB12, Level::Low, Speed::Low);
    let mut sr_nclr: Output<'_> = Output::new(p.PD2, Level::High, Speed::Low);

    let mut eeprom_nwc: Output<'_> = Output::new(p.PE10, Level::High, Speed::Low);

    let mut cam_pd: Output<'_> = Output::new(p.PD1, Level::Low, Speed::Low);
    let mut ram_npd: Output<'_> = Output::new(p.PD0, Level::High, Speed::Low);
    let mut ram_ndqm: Output<'_> = Output::new(p.PD4, Level::High, Speed::Low);
    let mut ram_nwe: Output<'_> = Output::new(p.PD5, Level::High, Speed::Low);
    let mut ram_ncas: Output<'_> = Output::new(p.PD6, Level::High, Speed::Low);
    let mut ram_nras: Output<'_> = Output::new(p.PD7, Level::High, Speed::Low);
    let mut addr_noe: Output<'_> = Output::new(p.PD11, Level::High, Speed::Low);
    let mut data_noe: Output<'_> = Output::new(p.PD12, Level::High, Speed::Low);

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

    let mut config = ospi::Config::default();
    config.memory_type = ospi::MemoryType::Standard;
    config.device_size = ospi::MemorySize::_16MiB;
    config.wrap_size = ospi::WrapSize::_64Bytes;
    config.chip_select_high_time = ospi::ChipSelectHighTime::_1Cycle;
    let mut flash_qspi = Ospi::new_blocking_quadspi(
        p.OCTOSPI2, p.PB2, p.PB1, p.PB0, p.PE2, p.PA1, p.PE11, config
    );

    loop {
        info!("Hello, World!");
        set_osd(osd_map::CENTER, &mut osd_en, &mut osd_a0, &mut osd_a1, &mut osd_a2).await;
    }
}

pub mod osd_map {
    pub const CENTER: u8 = 0;
    pub const UP: u8     = 1;
    pub const LEFT: u8   = 2;
    pub const DOWN: u8   = 3;
    pub const RIGHT: u8  = 4;
}

async fn set_osd(
    input: u8, 
    osd_en: &mut Output<'_>, 
    osd_a0: &mut Output<'_>, 
    osd_a1: &mut Output<'_>, 
    osd_a2: &mut Output<'_>,
    ) {

        // Async lock this function to prevent multiple threads configuring osd at once
        let lock = OSD_MUTEX.lock().await;

        if osd_en.is_set_high() { // Ensure the OSD controller enable is not set
            osd_en.set_low();
        }

        // Extract bits from the input and set the appropriate gpio outputs
        osd_a0.set_level(Level::from(((input >> 0) & 1) != 0));
        osd_a1.set_level(Level::from(((input >> 1) & 1) != 0));
        osd_a2.set_level(Level::from(((input >> 2) & 1) != 0));

        // Enable the osd output and sleep 200 ms then turn off ("button press")
        osd_en.set_high();
        Timer::after(Duration::from_millis(200)).await;
        osd_en.set_low();

        drop(lock);
}