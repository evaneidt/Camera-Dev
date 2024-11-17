#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{AnyPin, Pin, Output, Level, Speed};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _}; // global logger

#[embassy_executor::task]
async fn blinker(p: AnyPin, interval: Duration) {
    
    let mut led = Output::new(p, Level::Low, Speed::Low);
 
    loop {
        led.set_high();
        info!("High");
        Timer::after(interval).await;
        led.set_low();
        info!("Low");
        Timer::after(interval).await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Booting...");
unwrap!(_spawner.spawn(blinker(p.PA5.degrade(), Duration::from_millis(1000))));
}
