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
use crate::motor::{LeftMotor, Motor, MotorPair, RightMotor};

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
        let p4 = Batch::new(periph.P4).split(&pmm);
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

        let p2 = Batch::new(periph.P2).split(&pmm);
        let p3 = Batch::new(periph.P3).split(&pmm);
        let p6 = Batch::new(periph.P6).split(&pmm);
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
        let mut motor_pair = MotorPair::new(left_motor, right_motor);

        print_bytes(b"Forward\n");

        motor_pair.forward();
        motor_pair.set_speed(2000u16);
        motor_pair.enable();

        delay_ms(3000u16);
        print_bytes(b"Back\n");

        motor_pair.reverse();
        motor_pair.set_speed(1000u16);

        delay_ms(3000u16);
        print_bytes(b"Stop\n");

        motor_pair.disable();

        loop {
            asm::barrier();
        }
    }
    loop{}
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