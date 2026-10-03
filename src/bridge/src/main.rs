//! Dukki servo bridge firmware for the WeAct STM32G474 core board (QFN48,
//! STM32G474CEU6).
//!
//! For now this only blinks the board's blue LED on PC6 (active high), one
//! second on and one second off, to check the toolchain and the USB DFU
//! flashing path.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;
use panic_halt as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PC6, Level::Low, Speed::Low);

    loop {
        led.set_high();
        Timer::after_millis(1000).await;
        led.set_low();
        Timer::after_millis(1000).await;
    }
}
