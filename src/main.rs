#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
};
use embassy_stm32::{
    peripherals,
    usart::{self, Config, Uart},
};
use embassy_time::{Duration, Timer};
use fmt::info;
use fmt::unwrap;

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PB7, Level::High, Speed::Low);

    let config = Config::default();
    let mut usart = Uart::new(
        p.USART3, p.PC5, p.PB10, Irqs, p.DMA1_CH3, p.DMA1_CH4, config,
    )
    .unwrap();

    loop {
        info!("Hello, World!");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;

        unwrap!(usart.write("Hello DMA!".as_bytes()).await);
    }
}
