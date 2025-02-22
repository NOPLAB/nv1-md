#![no_std]
#![no_main]

mod fmt;
mod motor;

extern crate alloc;

use alloc::vec::Vec;
use embedded_alloc::LlffHeap as Heap;
use midly::num::u7;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use core::cell::RefCell;

use defmt::error;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use motor::{MotorGroupComplementary, MotorGroupSimple, Motors};

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{low_level::Pin, OutputType},
    pac::{self, common::W, timer::vals::Sms},
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

static G_HUB_MSG: Mutex<ThreadModeRawMutex, RefCell<nv1_msg::md::ToMD>> =
    Mutex::new(RefCell::new(nv1_msg::md::ToMD {
        enable: false,
        m1: 0.0,
        m2: 0.0,
        m3: 0.0,
        m4: 0.0,
    }));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // initialize static heap
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc;

        config.rcc.hse = Some(rcc::Hse {
            freq: Hertz(20_000_000),
            mode: rcc::HseMode::Oscillator,
        });

        config.rcc.pll_src = rcc::PllSource::HSE;
        config.rcc.pll = Some(rcc::Pll {
            prediv: rcc::PllPreDiv::DIV16,
            mul: rcc::PllMul::MUL288,
            divp: Some(rcc::PllPDiv::DIV2),
            divq: None,
            divr: Some(rcc::PllRDiv::DIV2),
        });

        config.rcc.sys = rcc::Sysclk::PLL1_P;

        config.rcc.apb1_pre = rcc::APBPrescaler::DIV4;
        config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;
    }

    let p = embassy_stm32::init(config);

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
    config.baudrate = 2_000_000;
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

    let mut pid1: Pid<f32> = pid::Pid::new(0.0, 100.0);
    let mut pid2: Pid<f32> = pid::Pid::new(0.0, 100.0);
    let mut pid3: Pid<f32> = pid::Pid::new(0.0, 100.0);
    let mut pid4: Pid<f32> = pid::Pid::new(0.0, 100.0);

    pid1.p(8.0, 50.0).i(4.0, 50.0).d(0.0, 0.0);
    pid2.p(8.0, 50.0).i(4.0, 50.0).d(0.0, 0.0);
    pid3.p(8.0, 50.0).i(4.0, 50.0).d(0.0, 0.0);
    pid4.p(8.0, 50.0).i(4.0, 50.0).d(0.0, 0.0);

    // let midi = midly::parse(include_bytes!("../midi.mid")).unwrap();

    info!("[MD] initialized");

    loop {
        let msg = G_HUB_MSG.lock().await.borrow().clone();

        if msg.enable {
            pid1.setpoint(msg.m1);
            pid2.setpoint(msg.m2);
            pid3.setpoint(msg.m3);
            pid4.setpoint(msg.m4);

            // PID Tune
            // pid1.setpoint(5.0);
            // pid2.setpoint(5.0);
            // pid3.setpoint(5.0);
            // pid4.setpoint(5.0);

            let motor1_rps = read_encoder1() / MOTOR_ENCODER_PLUS as f32 * MOTOR_GEAR_RATIO / 0.01;
            let motor2_rps = read_encoder2() / MOTOR_ENCODER_PLUS as f32 * MOTOR_GEAR_RATIO / 0.01;
            let motor3_rps = read_encoder3() / MOTOR_ENCODER_PLUS as f32 * MOTOR_GEAR_RATIO / 0.01;
            let motor4_rps = read_encoder4() / MOTOR_ENCODER_PLUS as f32 * MOTOR_GEAR_RATIO / 0.01;

            // info!(
            //     "rps: {}, {}, {}, {}",
            //     motor1_rps, motor2_rps, motor3_rps, motor4_rps
            // );

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

        // MIDI
        // motors.set_speed1(10);
        // motors.set_speed2(10);
        // motors.set_speed3(10);
        // motors.set_speed4(10);

        // let mut running_notes: Vec<(u7, u7)> = Vec::new();

        // for (i, track) in midi.clone().1.enumerate() {
        //     if i != 1 {
        //         continue;
        //     }

        //     let track = track.unwrap();
        //     for event in track {
        //         let event = event.unwrap();

        //         match event.kind {
        //             midly::TrackEventKind::Midi { channel, message } => {
        //                 if channel == 0 {
        //                     match message {
        //                         midly::MidiMessage::NoteOn { key, vel } => {
        //                             running_notes.push((key, vel));
        //                         }
        //                         midly::MidiMessage::NoteOff { key, vel: _ } => {
        //                             running_notes.retain(|&x| x.0 != key.as_int());
        //                         }
        //                         _ => (),
        //                     }

        //                     let max_note = running_notes.iter().max_by_key(|x| x.0.as_int());
        //                     let second_max_note = running_notes
        //                         .iter()
        //                         .filter(|x| x.0 != max_note.unwrap().0)
        //                         .max_by_key(|x| x.0.as_int());
        //                     if let Some(note) = max_note {
        //                         let key = note.0.as_int() as f32;
        //                         let hz = 440.0 * libm::powf(2.0_f32, (key - 69.0) / 12.0);

        //                         motors.group1.set_frequency(Hertz(hz as u32));
        //                     }
        //                     if let Some(note) = second_max_note {
        //                         let key = note.0.as_int() as f32;
        //                         let hz = 440.0 * libm::powf(2.0_f32, (key - 69.0) / 12.0);

        //                         motors.group2.set_frequency(Hertz(hz as u32));
        //                     }
        //                 }
        //             }
        //             _ => (),
        //         }

        //         if event.delta.as_int() == 0 {
        //             continue;
        //         } else {
        //             Timer::after_millis(event.delta.as_int() as u64).await;
        //         }
        //     }
        // }
    }
}

#[embassy_executor::task]
async fn uart_task(
    usart: Uart<'static, peripherals::USART3, peripherals::DMA1_CH3, peripherals::DMA1_CH1>,
) {
    let (_, uart_rx) = usart.split();

    let mut dma_buf = [0u8; 128];
    let mut uart_rx = uart_rx.into_ring_buffered(&mut dma_buf);

    let _ = uart_rx.start();

    loop {
        let mut byte = [0u8; 1];
        let mut msg_with_cobs = [0u8; 64];
        let mut c = 0;
        loop {
            let timeout_res =
                with_timeout(Duration::from_millis(50), uart_rx.read(&mut byte)).await;
            match timeout_res {
                Ok(receive_res) => match receive_res {
                    Ok(_size) => {
                        msg_with_cobs[c] = byte[0];
                        c += 1;

                        if byte[0] == 0 {
                            break;
                        }
                    }
                    Err(_err) => {
                        // error!("[UART] read error: {:?}, {}", err, c);

                        let _ = uart_rx.start();
                    }
                },
                Err(_) => {
                    error!("[UART] timeout");

                    G_HUB_MSG.lock().await.replace(nv1_msg::md::ToMD {
                        enable: false,
                        m1: 0.0,
                        m2: 0.0,
                        m3: 0.0,
                        m4: 0.0,
                    });

                    break;
                }
            }
        }

        match postcard::from_bytes_cobs::<nv1_msg::md::ToMD>(&mut msg_with_cobs) {
            Ok(msg) => {
                // info!("[UART] received msg: {:?}", msg.m1);
                G_HUB_MSG.lock().await.replace(msg);
            }
            Err(_) => {
                // info!("[UART] postcard decode error");
                continue;
            }
        };

        Timer::after_millis(5).await;
    }
}
