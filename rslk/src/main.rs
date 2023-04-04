#![no_std]
#![no_main]
#![feature(abi_msp430_interrupt)]
#![feature(core_panic)]

mod encoder;
mod line_sensor;
mod motor;
mod queuebuf;
mod serial_utils;

use core::{
    panic::PanicInfo,
    option::Option::Some,
};
use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use embedded_hal::PwmPin;
use msp430::asm;
use msp430fr2x5x_hal::{
    fram::Fram,
    gpio::Batch,
    serial::*,
    watchdog::Wdt,
};
use msp430_rt::entry;
use msp430fr2x5x_hal::clock::{
    ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv
};
use msp430fr2x5x_hal::pmm::Pmm;
use msp430fr2x5x_hal::pwm::{PwmParts3, PwmParts7, TimerConfig};
use serial_utils::{
    init_serial,
    print_bytes
};
use crate::line_sensor::{LineSensorBusy, LineSensorReady};
use crate::motor::{LeftMotor, Motor, MotorPair, RightMotor};
use crate::serial_utils::{byte_to_dec, byte_to_hex};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    print_bytes(b"Panic handler was called.\n");
    loop {
        // Prevent optimizations that can remove this loop.
        msp430::asm::barrier();
    }
}


#[entry]
fn main() -> ! {
    if let Some(periph) = msp430fr2355::Peripherals::take() {
        let mut fram = Fram::new(periph.FRCTL);
        let _wdt = Wdt::constrain(periph.WDT_A);
        let (smclk, _aclk, mut delay) = ClockConfig::new(periph.CS)
            .mclk_dcoclk(DcoclkFreqSel::_2MHz, MclkDiv::_2)
            .smclk_on(SmclkDiv::_1)
            .aclk_refoclk()
            .freeze(&mut fram);
        let pmm = Pmm::new(periph.PMM);

        let p2 = Batch::new(periph.P2).split(&pmm);
        let p3 = Batch::new(periph.P3).split(&pmm);
        let p4 = Batch::new(periph.P4).split(&pmm);
        let p6 = Batch::new(periph.P6).split(&pmm);

        let (tx, mut rx) = SerialConfig::new(
            periph.E_USCI_A1,
            BitOrder::LsbFirst,
            BitCount::EightBits,
            StopBits::OneStopBit,
            Parity::NoParity,
            Loopback::NoLoop,
            115200,
        )
        .use_smclk(&smclk)
        .split(p4.pin3.to_alternate1(), p4.pin2.to_alternate1());
        init_serial(rx, tx);
        print_bytes(b"Serial started\n");

        let pwm_tb1 = PwmParts3::new(periph.TB1,
                                     TimerConfig::smclk(&smclk),
                                     5000);
        let pwm_tb3 = PwmParts7::new(periph.TB3,
                                     TimerConfig::smclk(&smclk),
                                     5000);
        let mut left_pwm = pwm_tb1.pwm2.init(p2.pin1.to_output().to_alternate1());
        left_pwm.enable();
        left_pwm.set_duty(0);
        let mut right_pwm = pwm_tb3.pwm1.init(p6.pin0.to_output().to_alternate1());
        right_pwm.enable();
        right_pwm.set_duty(0);

        let mut left_motor: Motor<LeftMotor> = Motor::new(p3.pin5.to_output(),
                                                          left_pwm,
                                                          p3.pin2.to_output());
        let mut right_motor: Motor<RightMotor> = Motor::new(p3.pin1.to_output(),
                                                          right_pwm,
                                                          p3.pin0.to_output());
        // let mut motor_pair = MotorPair::new(left_motor, right_motor);


        let mut line_sensor_r : LineSensorReady = LineSensorReady::new(
            p6.pin1.to_output(),
            p4.pin0.to_output(),
            p4.pin7.to_output(),
            p6.pin3.to_output(),
            p4.pin6.to_output(),
            p6.pin2.to_output(),
            p6.pin4.to_output(),
            p2.pin4.to_output(),
            p3.pin7.to_output(),
            p2.pin2.to_output()
        );



        print_bytes(b"Forward\n");

        // motor_pair.forward();
        // motor_pair.set_speed(2000u16);
        // motor_pair.enable();
        //
        // delay_ms(3000u16);
        // print_bytes(b"Back\n");
        //
        // motor_pair.reverse();
        // motor_pair.set_speed(1000u16);
        //
        // delay_ms(3000u16);
        // print_bytes(b"Stop\n");
        //
        // motor_pair.disable();
        left_motor.enable();
        right_motor.enable();
        left_motor.forward();
        right_motor.forward();
        loop {
            let mut line_sensor_b : LineSensorBusy = line_sensor_r.start_read();
            for _ in 0 .. 60{
                asm::nop();
            }
            let res = line_sensor_b.grab_result();
            let val = res.0;
            line_sensor_r = res.1;

            print_bytes(&byte_to_hex(val));
            print_bytes(b"\n");

            line_response(&mut left_motor, &mut right_motor, val);
            delay_ms(100u16);
            asm::barrier();
        }
    }
    loop{}
}


fn count_set_bits(mut val: u8) -> u8{
    let mut count = 0;
    for _ in 0..8{
        count += val & 0x1;
        val >>= 1;
    }
    count
}

const MAX_SPEED : u16 = 500u16;
const MAX_SPEED_90 : u16 = (MAX_SPEED as f32 * 0.90) as u16;
const MAX_SPEED_75 : u16 = (MAX_SPEED as f32 * 0.75) as u16;
const MAX_SPEED_50 : u16 = (MAX_SPEED as f32 * 0.5) as u16;
const MAX_SPEED_25 : u16 = (MAX_SPEED as f32 * 0.25) as u16;
const MAX_SPEED_13_5 : u16 = (MAX_SPEED as f32 * 0.135) as u16;

fn line_response(left: &mut Motor<LeftMotor>, right: &mut Motor<RightMotor>, line_val: u8){
    let bits = count_set_bits(line_val);
    match bits{
        0 =>{}
        1 =>{
            match line_val {
                0x80 =>{
                    left.set_speed(MAX_SPEED);
                    right.set_speed(MAX_SPEED_13_5);
                }
                _ =>{
                    left.set_speed(MAX_SPEED_13_5);
                    right.set_speed(MAX_SPEED);
                }
            }
        }
        2 =>{
            match line_val {
                0b1100_0000 =>{
                    left.set_speed(MAX_SPEED_25);
                    right.set_speed(MAX_SPEED);
                }
                0b0110_0000 =>{
                    left.set_speed(MAX_SPEED_25);
                    right.set_speed(MAX_SPEED);
                }
                0b0011_0000 =>{
                    left.set_speed(MAX_SPEED_50);
                    right.set_speed(MAX_SPEED);
                }
                0b0001_1000 =>{
                    left.set_speed(MAX_SPEED);
                    right.set_speed(MAX_SPEED);
                }
                0b0000_1100 =>{
                    left.set_speed(MAX_SPEED);
                    right.set_speed(MAX_SPEED_50);
                }
                0b0000_0110 =>{
                    left.set_speed(MAX_SPEED);
                    right.set_speed(MAX_SPEED_25);
                }
                0b0000_0011 =>{
                    left.set_speed(MAX_SPEED);
                    right.set_speed(MAX_SPEED_25);
                }
                _ =>{
                    left.set_speed(MAX_SPEED);
                    right.set_speed(MAX_SPEED);
                }
            }
        }
        3 =>{
            match line_val {
                0b1110_0000 =>{
                    left.set_speed(MAX_SPEED_50);
                    right.set_speed(MAX_SPEED);
                }
                0b0111_0000 =>{
                    left.set_speed(MAX_SPEED_75);
                    right.set_speed(MAX_SPEED);
                }
                0b0011_1000 =>{
                    left.set_speed(MAX_SPEED_90);
                    right.set_speed(MAX_SPEED);
                }
                0b0001_1100 =>{
                    left.set_speed(MAX_SPEED_90);
                    right.set_speed(MAX_SPEED);
                }
                0b0000_1110 =>{
                    left.set_speed(MAX_SPEED);
                    right.set_speed(MAX_SPEED_75);
                }
                0b0000_0111 =>{
                    left.set_speed(MAX_SPEED);
                    right.set_speed(MAX_SPEED_50);
                }
                0b1001_1000 =>{
                    left.set_speed(0u16);
                    right.set_speed(MAX_SPEED);
                }
                0b0001_1001 =>{
                    left.set_speed(MAX_SPEED);
                    right.set_speed(0u16);
                }
                _ =>{
                    left.set_speed(MAX_SPEED);
                    right.set_speed(MAX_SPEED);
                }
            }
        }
        4 =>{
            match line_val {
                0b1111_0000 => {
                    left.set_speed(0u16);
                    right.set_speed(MAX_SPEED);
                }
                0b0000_1111 => {
                    left.set_speed(MAX_SPEED);
                    right.set_speed(0u16);
                }
                0b1100_1100 | 0b1101_1000 => {
                    left.set_speed(0u16);
                    right.set_speed(MAX_SPEED);
                }
                0b0011_0011 | 0b0001_1011 => {
                    left.set_speed(MAX_SPEED);
                    right.set_speed(0u16);
                }
                _ =>{
                    left.set_speed(MAX_SPEED);
                    right.set_speed(MAX_SPEED);
                }
            }
        }
        5 =>{
            match line_val {
                0b1111_1000 => {
                    left.set_speed(0u16);
                    right.set_speed(MAX_SPEED);
                }
                0b0001_1111 => {
                    left.set_speed(MAX_SPEED);
                    right.set_speed(0u16);
                }
                0b1101_1100 => {
                    left.set_speed(MAX_SPEED_13_5);
                    right.set_speed(MAX_SPEED);
                }
                0b0011_1011 => {
                    left.set_speed(MAX_SPEED);
                    right.set_speed(MAX_SPEED_13_5);
                }
                _ =>{
                    left.set_speed(MAX_SPEED);
                    right.set_speed(MAX_SPEED);
                }
            }
        }
        6 =>{
            match line_val {
                0b1111_1100 => {
                    left.set_speed(0u16);
                    right.set_speed(MAX_SPEED);
                }
                0b0011_1111 => {
                    left.set_speed(MAX_SPEED);
                    right.set_speed(0u16);
                }
                _ =>{
                    left.set_speed(MAX_SPEED);
                    right.set_speed(MAX_SPEED);
                }
            }
        }
        _ =>{
            left.set_speed(MAX_SPEED);
            right.set_speed(MAX_SPEED_90);
        }
    }
}

fn delay_ms(ms: u16){
    for _ in 0 .. ms{
        for _ in 0 .. 60{
            asm::barrier();
            asm::nop();
        }
    }
}

#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}