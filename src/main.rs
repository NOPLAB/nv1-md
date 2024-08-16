#![no_std]
#![no_main]

mod fmt;
mod motor;

use motor::{MotorGroup, MotorGroupComplementary, MotorGroupSimple, Motors};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, OutputType, Speed},
    time::khz,
    timer::{
        complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin},
        simple_pwm::{PwmPin, SimplePwm},
        Channel,
    },
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

    let mut config = Config::default();
    config.baudrate = 250000;
    let mut usart = Uart::new(
        p.USART3, p.PC5, p.PB10, Irqs, p.DMA1_CH3, p.DMA1_CH1, config,
    )
    .unwrap();

    let pwm1_ch1 = PwmPin::new_ch1(p.PA8, OutputType::PushPull);
    let pwm1_ch4 = PwmPin::new_ch4(p.PA11, OutputType::PushPull);

    let pwm1_ch2n = ComplementaryPwmPin::new_ch2(p.PB14, OutputType::PushPull);
    let pwm1_ch3n = ComplementaryPwmPin::new_ch3(p.PB15, OutputType::PushPull);

    let mut pwm1 = ComplementaryPwm::new(
        p.TIM1,
        Some(pwm1_ch1),
        None,
        None,
        Some(pwm1_ch2n),
        None,
        Some(pwm1_ch3n),
        Some(pwm1_ch4),
        None,
        khz(1),
        Default::default(),
    );

    pwm1.enable(Channel::Ch1);
    pwm1.enable(Channel::Ch2);
    pwm1.enable(Channel::Ch3);
    pwm1.enable(Channel::Ch4);

    let motor_group1 = MotorGroupComplementary::new(
        pwm1,
        Channel::Ch1,
        Channel::Ch4,
        Channel::Ch2,
        Channel::Ch3,
        128,
    );

    let pwm8_ch1 = PwmPin::new_ch1(p.PC6, OutputType::PushPull);
    let pwm8_ch2 = PwmPin::new_ch2(p.PC7, OutputType::PushPull);
    let pwm8_ch3 = PwmPin::new_ch3(p.PC8, OutputType::PushPull);
    let pwm8_ch4 = PwmPin::new_ch4(p.PC9, OutputType::PushPull);

    let mut pwm8 = SimplePwm::new(
        p.TIM8,
        Some(pwm8_ch1),
        Some(pwm8_ch2),
        Some(pwm8_ch3),
        Some(pwm8_ch4),
        khz(1),
        Default::default(),
    );

    pwm8.enable(Channel::Ch1);
    pwm8.enable(Channel::Ch2);
    pwm8.enable(Channel::Ch3);
    pwm8.enable(Channel::Ch4);

    let motor_group2 = MotorGroupSimple::new(
        pwm8,
        Channel::Ch1,
        Channel::Ch2,
        Channel::Ch3,
        Channel::Ch4,
        128,
    );

    let mut motors = Motors::new(motor_group1, motor_group2);

    loop {
        info!("Hello, World!");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;

        motors.set_speed1(128);
        motors.set_speed2(64);
        motors.set_speed3(128);
        motors.set_speed4(64);

        unwrap!(usart.write("Hello DMA!".as_bytes()).await);
    }
}
