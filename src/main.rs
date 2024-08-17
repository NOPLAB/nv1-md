#![no_std]
#![no_main]

mod fmt;
mod motor;

use embassy_time::Timer;
use motor::{MotorGroupComplementary, MotorGroupSimple, Motors};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::timer::low_level::GeneralPurpose32bitInstance;
use embassy_stm32::{
    bind_interrupts,
    gpio::{low_level::Pin, OutputType},
    pac::{self, timer::vals::Sms},
    time::khz,
    timer::{
        complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin},
        simple_pwm::{PwmPin, SimplePwm},
        Channel,
    },
    usart::Uart,
};
use embassy_stm32::{pac::timer::vals::CcmrInputCcs, rcc::low_level::RccPeripheral};
use embassy_stm32::{
    peripherals,
    usart::{self, Config},
};
use fmt::info;

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct MotorSpeed {
    motor1: f32,
    motor2: f32,
    motor3: f32,
    motor4: f32,
}

#[repr(C)]
union MotorSpeedData {
    motor_speed: MotorSpeed,
    buffer: [u8; 16],
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

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
        Channel::Ch3,
        Channel::Ch2,
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

    let mut config = Config::default();
    config.baudrate = 115200;
    let usart = Uart::new(
        p.USART3, p.PC5, p.PB10, Irqs, p.DMA1_CH3, p.DMA1_CH1, config,
    )
    .unwrap();

    spawner.must_spawn(uart_task(usart));

    // encoder mode tim5 motor1

    // initialize gpio
    pac::RCC.ahb1enr().modify(|r| r.set_gpioaen(true));
    p.PA0
        .block()
        .moder()
        .modify(|r| r.set_moder(0, pac::gpio::vals::Moder::ALTERNATE));
    p.PA0.block().afr(0).modify(|r| r.set_afr(0, 2));
    p.PA1
        .block()
        .moder()
        .modify(|r| r.set_moder(1, pac::gpio::vals::Moder::ALTERNATE));
    p.PA1.block().afr(0).modify(|r| r.set_afr(1, 2));

    // initialize tim
    peripherals::TIM5::enable_and_reset();

    let tim5 = peripherals::TIM5::regs_gp32();

    tim5.psc().write(|w| w.set_psc(0));
    tim5.arr().write(|w| w.set_arr(0xFF));
    tim5.cnt().write(|w| w.set_cnt(128));
    tim5.ccmr_input(0)
        .modify(|w| w.set_ccs(0, CcmrInputCcs::from_bits(0b01)));
    tim5.ccmr_input(0)
        .modify(|w| w.set_ccs(1, CcmrInputCcs::from_bits(0b01)));
    tim5.ccer().modify(|w| {
        w.0 &= !(0x1 << 1) | !(0x1 << 5);
    });
    tim5.smcr().modify(|w| w.set_sms(Sms::ENCODER_MODE_3));
    tim5.cr1().modify(|r| r.set_cen(true));

    loop {
        let speed = 0;
        motors.set_speed1(speed);
        motors.set_speed2(speed);
        motors.set_speed3(speed);
        motors.set_speed4(speed);

        Timer::after_millis(10).await;

        info!("{}", tim5.cnt().read().cnt());
    }
}

#[embassy_executor::task]
async fn uart_task(
    mut usart: Uart<'static, peripherals::USART3, peripherals::DMA1_CH3, peripherals::DMA1_CH1>,
) {
    // read until 0
    let mut buffer = [0; 1];
    loop {
        usart.read(&mut buffer).await.unwrap();
        if buffer[0] == 0 {
            break;
        }
    }

    loop {
        const DECODE_DATA_SIZE: usize = 16;
        const RECEIVE_DATA_SIZE: usize = DECODE_DATA_SIZE + 2;

        let mut buffer = [0; RECEIVE_DATA_SIZE];
        usart.read(&mut buffer).await.unwrap();

        // info!("Data: {:?}", stack);

        let mut decoded_buf = [0; 32];
        match corncobs::decode_buf(&buffer, &mut decoded_buf) {
            Ok(size) => {
                if size != DECODE_DATA_SIZE {
                    info!("Invalid data size: {}", size);
                    continue;
                }

                let motor_speed_data = MotorSpeedData {
                    buffer: decoded_buf[0..DECODE_DATA_SIZE].try_into().unwrap(),
                };

                let motor_speed = unsafe { motor_speed_data.motor_speed };
            }
            Err(err) => {
                info!("Failed to decode data");
                info!("Data: {:?}", buffer);
                match err {
                    corncobs::CobsError::Truncated => info!("Truncated"),
                    corncobs::CobsError::Corrupt => info!("Corrupt"),
                }

                // read until 0
                let mut buffer = [0; 1];
                loop {
                    usart.read(&mut buffer).await.unwrap();
                    if buffer[0] == 0 {
                        break;
                    }
                }
            }
        }
    }
}
