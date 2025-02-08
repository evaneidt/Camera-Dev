#![no_std]
#![no_main]

mod fmt;

use cortex_m::{delay, prelude::_embedded_hal_blocking_delay_DelayMs};
use defmt::warn;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, peripherals, mode::Async, Config};
use embassy_stm32::dcmi::{self, *};
use embassy_stm32::gpio::{Level, Output, Speed, Pull, Input};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::i2s::{self, I2S};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::ospi::{self, Ospi};
use embassy_stm32::usb::{self, Driver, Instance};
use embassy_stm32::time::{hz, khz, mhz};
use embassy_time::{self, Delay, Duration, Timer};
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_usb::class::hid;
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use fmt::info;

bind_interrupts!(struct Irqs {
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>; // I2C2 event
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>; // I2C2 error
    DCMI_PSSI => dcmi::InterruptHandler<peripherals::DCMI>;
    OTG_HS => usb::InterruptHandler<peripherals::USB_OTG_HS>;
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
        config.rcc.hsi = Some(HSIPrescaler::DIV2);
        config.rcc.csi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL32,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV8), 
            divr: None,
        });
        config.rcc.sys = Sysclk::HSI; 
        config.rcc.ahb_pre = AHBPrescaler::DIV2; 
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV2; 
        config.rcc.apb3_pre = APBPrescaler::DIV2;
        config.rcc.apb4_pre = APBPrescaler::DIV2;
        config.rcc.voltage_scale = VoltageScale::Scale0;
        config.rcc.supply_config = SupplyConfig::LDO;
    }
    let p: embassy_stm32::Peripherals = embassy_stm32::init(config);

    defmt::info!("Booting...");

    let mut led_r = Output::new(p.PD8, Level::Low, Speed::Low);
    let mut led_g = Output::new(p.PD10, Level::Low, Speed::Low);
    let mut led_b = Output::new(p.PD9, Level::Low, Speed::Low);

    let mut osd_en: Output<'_> = Output::new(p.PB5, Level::Low, Speed::Low);
    let mut osd_a0: Output<'_> = Output::new(p.PB6, Level::Low, Speed::Low);
    let mut osd_a1: Output<'_> = Output::new(p.PB7, Level::Low, Speed::Low);
    let mut osd_a2: Output<'_> = Output::new(p.PB8, Level::Low, Speed::Low);

    let mut i2s_mclk_ctrl: Output<'_> = Output::new(p.PB12, Level::Low, Speed::Low);
    let mut sr_nclr: Output<'_> = Output::new(p.PD2, Level::High, Speed::Low);

    let mut eeprom_nwc: Output<'_> = Output::new(p.PE10, Level::High, Speed::Low);

    let mut cam_pd: Output<'_> = Output::new(p.PD1, Level::High, Speed::Low); // Off by default
    let mut ram_npd: Output<'_> = Output::new(p.PD0, Level::High, Speed::Low);
    let mut ram_ndqm: Output<'_> = Output::new(p.PD4, Level::High, Speed::Low);
    let mut ram_nwe: Output<'_> = Output::new(p.PD5, Level::High, Speed::Low);
    let mut ram_ncas: Output<'_> = Output::new(p.PD6, Level::High, Speed::Low);
    let mut ram_nras: Output<'_> = Output::new(p.PD7, Level::High, Speed::Low);
    let mut addr_noe: Output<'_> = Output::new(p.PD11, Level::High, Speed::Low);
    let mut data_noe: Output<'_> = Output::new(p.PD12, Level::High, Speed::Low);

    let mut i2c_bus = I2c::new(
        p.I2C2, // peri
        p.PB10, // scl
        p.PB11, // sda
        Irqs,   // int binding
        p.DMA1_CH1,
        p.DMA1_CH2,
        khz(100),
        Default::default(),
    );

    let mut dac_ncs: Output<'_> = Output::new(p.PA4, Level::High, Speed::Low);

    let mut config = spi::Config::default();
    config.frequency = mhz(1); // TODO
    let mut dac_spi = Spi::new_txonly(
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

    // let mut config = i2s::Config::default();
    // config.master_clock = false;
    // config.mode = i2s::Mode::Slave;
    // let mut mic_buffer = [0x00_u16; 128];

    // let mut mic = I2S::new_rxonly( // PA3 is unused and not mapped to MCLK in HW
    //     p.SPI6, p.PB4, p.PA0, p.PA5, p.PA3, p.BDMA2_CH0, &mut mic_buffer, mhz(1), config,
    // );

    let mut config = ospi::Config::default();
    config.memory_type = ospi::MemoryType::Standard;
    config.device_size = ospi::MemorySize::_16MiB;
    config.wrap_size = ospi::WrapSize::_64Bytes;
    config.chip_select_high_time = ospi::ChipSelectHighTime::_1Cycle;
    let mut flash_qspi = Ospi::new_blocking_quadspi(
        p.OCTOSPI2, p.PB2, p.PB1, p.PB0, p.PE2, p.PA1, p.PE11, config
    );

    let mut config = usb::Config::default();
    config.vbus_detection = true;
    let mut usb_buf = [0_u8; 256];
    let mut usb_impl = Driver::new_fs(
        p.USB_OTG_HS, Irqs, p.PA12, p.PA11, &mut usb_buf, config
    );

    config_dac(&mut dac_spi, &mut dac_ncs).await;

    i2s_mclk_ctrl.set_high();

    loop {
        info!("Loop");
        led_b.set_low();
        
        Timer::after_millis(1000).await;
        
        led_b.set_high();
        
        Timer::after_millis(1000).await;
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

        if input == osd_map::CENTER {
            info!("CENTER");
        } else if input == osd_map::UP {
            info!("UP");
        } else if input == osd_map::LEFT {
            info!("LEFT");
        } else if input == osd_map::DOWN {
            info!("DOWN");
        } else if input == osd_map::RIGHT {
            info!("RIGHT");
        } else {
            info!("Invalid OSD option")
        }

        // Async lock this function to prevent multiple threads configuring osd at once
        let lock = OSD_MUTEX.lock().await;

        if osd_en.is_set_high() { // Ensure the OSD controller enable is not set
            osd_en.set_low();
        }

        // Extract bits from the input and set the appropriate gpio outputs
            // != 0 is equivalent to casting to a bool (0 --> false, 1 --> true)
        osd_a0.set_level(Level::from(((input >> 0) & 1) != 0)); 
        osd_a1.set_level(Level::from(((input >> 1) & 1) != 0));
        osd_a2.set_level(Level::from(((input >> 2) & 1) != 0));

        // Enable the osd output and sleep 200 ms then turn off ("button press")
        osd_en.set_high();
        Timer::after(Duration::from_millis(200)).await;
        osd_en.set_low();
        Timer::after(Duration::from_millis(500)).await;

        drop(lock);
}

async fn config_dac(spi: &mut Spi<'_, Async>, cs: &mut Output<'_>){
    let mut data = [0x00_u8; 2];

    let atten_dB = 0; // - dB
    let inv_atten_width = (1.0 / 1.0) as u8; // bit/dB

    assert!(inv_atten_width == 1 || inv_atten_width == 2);

    cs.set_low();

    data[0] = 0x12; // Register 18
    data[1] = 1 << 7; // Software Reset
    let _ = spi.write(&data).await.ok();
    
    Timer::after(Duration::from_millis(50)).await;

    data[0] = 0x10; // Register 16
    data[1] = 255 - (atten_dB * inv_atten_width); // Attenuation
    let _ = spi.write(&data).await.ok();
    data[0] = 0x11; // Register 17
    data[1] = 255 - (atten_dB * inv_atten_width); // Attenuation
    let _ = spi.write(&data).await.ok();
    data[0] = 0x12; // Register 18
    data[1] = 0 << 6; // Oversampling on
    let _ = spi.write(&data).await.ok();
    data[0] = 0x13; // Register 19
    data[1] = 0 << 4; // Set Deemphasis
    let _ = spi.write(&data).await.ok();
    data[0] = 0x14; // Register 20
    data[1] = 0b100; // Set to I2S mode
    let _ = spi.write(&data).await.ok();
    data[0] = 0x15; // Register 21
    data[1] = if inv_atten_width == 1 {1} else {0}; // Set digital attenuation width
    let _ = spi.write(&data).await.ok();
    data[0] = 0x16; // Register 22
    data[1] = 0; // Phase inversion
    let _ = spi.write(&data).await.ok();
    cs.set_high();
}