#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Debug};

use defmt::error;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use heapless::Vec;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

mod fmt;
mod motor;
mod music;

use motor::{MotorGroupComplementary, MotorGroupSimple, Motors};

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{low_level::Pin, OutputType},
    pac::{self, timer::vals::Sms},
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
use embassy_stm32::{
    time::Hertz,
    timer::low_level::{GeneralPurpose16bitInstance, GeneralPurpose32bitInstance},
};
use embassy_time::{with_timeout, Duration, Timer};

use fmt::info;

use pid::Pid;

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

const MOTOR_ENCODER_PLUS: usize = 3 * 4;
const MOTOR_GEAR_RATIO: f32 = 1.0 / 19.225;

static G_HUB_MSG: Mutex<ThreadModeRawMutex, RefCell<nv1_msg::md::HubMsgPackRx>> =
    Mutex::new(RefCell::new(nv1_msg::md::HubMsgPackRx {
        enable: false,
        m1: 0.0,
        m2: 0.0,
        m3: 0.0,
        m4: 0.0,
    }));

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
        Hertz(470),
        Default::default(),
    );

    pwm1.enable(Channel::Ch1);
    pwm1.enable(Channel::Ch2);
    pwm1.enable(Channel::Ch3);
    pwm1.enable(Channel::Ch4);

    let motor_group1 = MotorGroupComplementary::new(
        pwm1,
        Channel::Ch4,
        Channel::Ch1,
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
        Hertz(470),
        Default::default(),
    );

    pwm8.enable(Channel::Ch1);
    pwm8.enable(Channel::Ch2);
    pwm8.enable(Channel::Ch3);
    pwm8.enable(Channel::Ch4);

    let motor_group2 = MotorGroupSimple::new(
        pwm8,
        Channel::Ch2,
        Channel::Ch1,
        Channel::Ch4,
        Channel::Ch3,
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

    const ENCODER_TIM_MAX_VALUE: u16 = 0xFF;
    const ENCODER_TIM_HALF_VALUE: u16 = ENCODER_TIM_MAX_VALUE / 2;

    // initialize gpio
    pac::RCC.ahb1enr().modify(|r| r.set_gpioaen(true));
    pac::RCC.ahb1enr().modify(|r| r.set_gpioben(true));

    // encoder mode tim5 motor1

    // initialize gpio
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
    tim5.arr()
        .write(|w| w.set_arr(ENCODER_TIM_MAX_VALUE as u32));
    tim5.cnt()
        .write(|w| w.set_cnt(ENCODER_TIM_HALF_VALUE as u32));
    tim5.ccmr_input(0)
        .modify(|w| w.set_ccs(0, CcmrInputCcs::from_bits(0b01)));
    tim5.ccmr_input(0)
        .modify(|w| w.set_ccs(1, CcmrInputCcs::from_bits(0b01)));
    tim5.ccer().modify(|w| {
        w.0 &= !(0x1 << 1) | !(0x1 << 5);
    });
    tim5.smcr().modify(|w| w.set_sms(Sms::ENCODER_MODE_3));
    tim5.cr1().modify(|r| r.set_cen(true));

    // encoder mode tim3 motor2

    // initialize gpio
    p.PB4
        .block()
        .moder()
        .modify(|r| r.set_moder(4, pac::gpio::vals::Moder::ALTERNATE));
    p.PB4.block().afr(0).modify(|r| r.set_afr(4, 2));
    p.PB5
        .block()
        .moder()
        .modify(|r| r.set_moder(5, pac::gpio::vals::Moder::ALTERNATE));
    p.PB5.block().afr(0).modify(|r| r.set_afr(5, 2));

    // initialize tim
    peripherals::TIM3::enable_and_reset();

    let tim3 = peripherals::TIM3::regs_gp16();

    tim3.psc().write(|w| w.set_psc(0));
    tim3.arr().write(|w| w.set_arr(ENCODER_TIM_MAX_VALUE));
    tim3.cnt().write(|w| w.set_cnt(ENCODER_TIM_HALF_VALUE));
    tim3.ccmr_input(0)
        .modify(|w| w.set_ccs(0, CcmrInputCcs::from_bits(0b01)));
    tim3.ccmr_input(0)
        .modify(|w| w.set_ccs(1, CcmrInputCcs::from_bits(0b01)));
    tim3.ccer().modify(|w| {
        w.0 &= !(0x1 << 1) | !(0x1 << 5);
    });
    tim3.smcr().modify(|w| w.set_sms(Sms::ENCODER_MODE_3));
    tim3.cr1().modify(|r| r.set_cen(true));

    // encoder mod tim4 motor3

    // initialize gpio
    p.PB6
        .block()
        .moder()
        .modify(|r| r.set_moder(6, pac::gpio::vals::Moder::ALTERNATE));
    p.PB6.block().afr(0).modify(|r| r.set_afr(6, 2));
    p.PB7
        .block()
        .moder()
        .modify(|r| r.set_moder(7, pac::gpio::vals::Moder::ALTERNATE));
    p.PB7.block().afr(0).modify(|r| r.set_afr(7, 2));

    // initialize tim
    peripherals::TIM4::enable_and_reset();

    let tim4 = peripherals::TIM4::regs_gp16();

    tim4.psc().write(|w| w.set_psc(0));
    tim4.arr().write(|w| w.set_arr(ENCODER_TIM_MAX_VALUE));
    tim4.cnt().write(|w| w.set_cnt(ENCODER_TIM_HALF_VALUE));
    tim4.ccmr_input(0)
        .modify(|w| w.set_ccs(0, CcmrInputCcs::from_bits(0b01)));
    tim4.ccmr_input(0)
        .modify(|w| w.set_ccs(1, CcmrInputCcs::from_bits(0b01)));
    tim4.ccer().modify(|w| {
        w.0 &= !(0x1 << 1) | !(0x1 << 5);
    });
    tim4.smcr().modify(|w| w.set_sms(Sms::ENCODER_MODE_3));
    tim4.cr1().modify(|r| r.set_cen(true));

    // encoder mode tim2 motor4

    // initialize gpio
    p.PA15
        .block()
        .moder()
        .modify(|r| r.set_moder(15, pac::gpio::vals::Moder::ALTERNATE));
    p.PA15.block().afr(1).modify(|r| r.set_afr(7, 1));
    p.PB9
        .block()
        .moder()
        .modify(|r| r.set_moder(9, pac::gpio::vals::Moder::ALTERNATE));
    p.PB9.block().afr(1).modify(|r| r.set_afr(1, 1));

    // initialize tim
    peripherals::TIM2::enable_and_reset();

    let tim2 = peripherals::TIM2::regs_gp32();

    tim2.psc().write(|w| w.set_psc(0));
    tim2.arr()
        .write(|w| w.set_arr(ENCODER_TIM_MAX_VALUE as u32));
    tim2.cnt()
        .write(|w| w.set_cnt(ENCODER_TIM_HALF_VALUE as u32));
    tim2.ccmr_input(0)
        .modify(|w| w.set_ccs(0, CcmrInputCcs::from_bits(0b01)));
    tim2.ccmr_input(0)
        .modify(|w| w.set_ccs(1, CcmrInputCcs::from_bits(0b01)));
    tim2.ccer().modify(|w| {
        w.0 &= !(0x1 << 1) | !(0x1 << 5);
    });
    tim2.smcr().modify(|w| w.set_sms(Sms::ENCODER_MODE_3));
    tim2.cr1().modify(|r| r.set_cen(true));

    let read_encoder1 = || {
        let tmp = (tim5.cnt().read().cnt() as i16 - ENCODER_TIM_HALF_VALUE as i16) as f32;
        tim5.cnt()
            .write(|w| w.set_cnt(ENCODER_TIM_HALF_VALUE as u32));
        return tmp;
    };
    let read_encoder2 = || {
        let tmp = (tim3.cnt().read().cnt() as i16 - ENCODER_TIM_HALF_VALUE as i16) as f32;
        tim3.cnt()
            .write(|w| w.set_cnt(ENCODER_TIM_HALF_VALUE as u16));
        return tmp;
    };
    let read_encoder3 = || {
        let tmp = (tim4.cnt().read().cnt() as i16 - ENCODER_TIM_HALF_VALUE as i16) as f32;
        tim4.cnt()
            .write(|w| w.set_cnt(ENCODER_TIM_HALF_VALUE as u16));
        return tmp;
    };
    let read_encoder4 = || {
        let tmp = -(tim2.cnt().read().cnt() as i16 - ENCODER_TIM_HALF_VALUE as i16) as f32;
        tim2.cnt()
            .write(|w| w.set_cnt(ENCODER_TIM_HALF_VALUE as u32));
        return tmp;
    };

    let mut pid1: Pid<f32> = pid::Pid::new(10.0, 100.0);
    let mut pid2: Pid<f32> = pid::Pid::new(0.0, 100.0);
    let mut pid3: Pid<f32> = pid::Pid::new(0.0, 100.0);
    let mut pid4: Pid<f32> = pid::Pid::new(0.0, 100.0);

    pid1.p(5.0, 100.0).i(3.0, 100.0).d(0.0, 0.0);
    pid2.p(5.0, 100.0).i(3.0, 100.0).d(0.0, 0.0);
    pid3.p(5.0, 100.0).i(3.0, 100.0).d(0.0, 0.0);
    pid4.p(5.0, 100.0).i(3.0, 100.0).d(0.0, 0.0);

    let music = music::MUSIC_DOREMI;
    let mut play_time: u32 = 0;
    let mut next_music: u32 = music::MUSIC_DOREMI[0].1;
    let mut music_index: usize = 0;

    info!("MD initialized");

    loop {
        let msg = G_HUB_MSG.lock().await.borrow().clone();

        if msg.enable {
            pid1.setpoint(msg.m1);
            pid2.setpoint(msg.m2);
            pid3.setpoint(msg.m3);
            pid4.setpoint(msg.m4);

            let motor1_rps = read_encoder1() / MOTOR_ENCODER_PLUS as f32 * MOTOR_GEAR_RATIO / 0.01;
            let motor2_rps = read_encoder2() / MOTOR_ENCODER_PLUS as f32 * MOTOR_GEAR_RATIO / 0.01;
            let motor3_rps = read_encoder3() / MOTOR_ENCODER_PLUS as f32 * MOTOR_GEAR_RATIO / 0.01;
            let motor4_rps = read_encoder4() / MOTOR_ENCODER_PLUS as f32 * MOTOR_GEAR_RATIO / 0.01;

            if motor1_rps.is_nan() {
                continue;
            }

            let motor1_output = pid1.next_control_output(motor1_rps);
            let motor2_output = pid2.next_control_output(motor2_rps);
            let motor3_output = pid3.next_control_output(motor3_rps);
            let motor4_output = pid4.next_control_output(motor4_rps);

            motors.set_speed1(motor1_output.output as i16);
            motors.set_speed2(motor2_output.output as i16);
            motors.set_speed3(motor3_output.output as i16);
            motors.set_speed4(motor4_output.output as i16);
        } else {
            pid1.setpoint(0.0);
            pid2.setpoint(0.0);
            pid3.setpoint(0.0);
            pid4.setpoint(0.0);

            motors.stop1();
            motors.stop2();
            motors.stop3();
            motors.stop4();
        }

        // info!(
        //     "rps: {}, {}, {}, {}",
        //     motor1_rps, motor2_rps, motor3_rps, motor4_rps
        // );

        // info!(
        //     "output: {}, {}, {}",
        //     motor1_output.p, motor1_output.i, motor1_output.d
        // );

        // info!(
        //     "{}, {}, {}, {}",
        //     tim5.cnt().read().cnt(),
        //     tim3.cnt().read().cnt(),
        //     tim4.cnt().read().cnt(),
        //     tim2.cnt().read().cnt()
        // );

        Timer::after_millis(10).await;

        // play_time += 10;
        // if play_time > next_music {
        //     play_time = 0;
        //     music_index += 1;
        //     if music_index >= music::MUSIC_DOREMI.len() {
        //         music_index = 0;
        //     }
        //     next_music = music::MUSIC_DOREMI[music_index].1;
        //     motors.set_frequency(music[music_index].0);
        // }
    }
}

#[embassy_executor::task]
async fn uart_task(
    mut usart: Uart<'static, peripherals::USART3, peripherals::DMA1_CH3, peripherals::DMA1_CH1>,
) {
    let initial_msg = nv1_msg::md::HubMsgPackRx {
        enable: false,
        m1: 0.0,
        m2: 0.0,
        m3: 0.0,
        m4: 0.0,
    };
    let decoded = postcard::to_vec_cobs::<nv1_msg::md::HubMsgPackRx, 64>(&initial_msg).unwrap();
    let receive_data_size = decoded.len();

    // let mut original_msg: Vec<u8, 64> = Vec::new();
    // original_msg.resize(receive_data_size, 0).unwrap();
    let mut original_msg = [0u8; 19];

    let mut timeout_count = 0;
    loop {
        let timeout_res =
            with_timeout(Duration::from_millis(5), usart.read(&mut original_msg)).await;
        match timeout_res {
            Ok(rx) => match rx {
                Ok(_) => {
                    // info!("[UART] received data");
                    match postcard::from_bytes_cobs::<nv1_msg::md::HubMsgPackRx>(&mut original_msg)
                    {
                        Ok(msg) => {
                            info!("[UART] received msg: {:?}", msg.m1);
                            G_HUB_MSG.lock().await.replace(msg);
                        }
                        Err(_) => {
                            info!("[UART] postcard decode error");
                            continue;
                        }
                    };
                    timeout_count = 0;
                }
                Err(err) => {
                    error!("[UART] read error: {:?}", err);
                    continue;
                }
            },
            Err(_) => {
                timeout_count += 1;

                if timeout_count > 100 {
                    error!("[UART] timeout");
                    G_HUB_MSG.lock().await.get_mut().enable = false;
                    timeout_count = 0;
                }
                continue;
            }
        }
    }
}
