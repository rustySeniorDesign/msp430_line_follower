use core::marker::PhantomData;
use msp430fr2355::{P3, TB1, TB3};
use msp430fr2x5x_hal::gpio::*;
use msp430fr2x5x_hal::pwm::{CCR1, CCR2, Pwm};

// DIRL: P3.5
// DIRR: P3.1
// PWML: P2.1 / TB1.2
// PWMR: P6.0 / TB3.1
// ENL: P3.2
// ENR: P3.0

pub trait MotorPins{
    type Direction;
    type PWM;
    type Enable;
}

pub struct LeftMotor;
impl MotorPins for LeftMotor {
    type Direction = Pin<P3, Pin5, Output>;
    type PWM = Pwm<TB1, CCR2>;
    type Enable = Pin<P3, Pin2, Output>;
}

pub struct RightMotor;
impl MotorPins for RightMotor {
    type Direction = Pin<P3, Pin1, Output>;
    type PWM = Pwm<TB3, CCR1>;
    type Enable = Pin<P3, Pin0, Output>;
}

#[derive(Copy, Clone)]
pub enum MotorDirection{
    Forward = 0,
    Reverse = 1,
}

pub struct Motor<PINS: MotorPins> {
    dir: PINS::Direction,
    pwm: PINS::PWM,
    en: PINS::Enable,
}

impl<PINS: MotorPins> Motor<PINS>{

    pub fn new<DIR: Into<PINS::Direction>, PWM: Into<PINS::PWM>, EN: Into<PINS::Enable>>
        (dir:DIR, pwm:PWM, en:EN) -> Self{
        // TODO
        Motor{
            dir: dir.into(),
            pwm: pwm.into(),
            en: en.into()
        }
    }

    pub fn set_speed(&mut self, speed: u8){
        // TODO
    }

    pub fn set_direction(&mut self, direction: MotorDirection){
        // TODO
    }

    pub fn enable(&mut self){
        // TODO
    }

    pub fn disable(&mut self){
        // TODO
    }
}
