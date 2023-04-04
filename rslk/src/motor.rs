use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
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
    type Direction: OutputPin;
    type PWM: PwmPin;
    type Enable: OutputPin;
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

pub struct Motor<PINS: MotorPins> {
    dir: PINS::Direction,
    pwm: PINS::PWM,
    en: PINS::Enable,
}

impl<PINS: MotorPins> Motor<PINS>{

    pub fn new<DIR: Into<PINS::Direction>, PWM: Into<PINS::PWM>, EN: Into<PINS::Enable>>
        (dir:DIR, pwm:PWM, en:EN) -> Self{
        Motor{
            dir: dir.into(),
            pwm: pwm.into(),
            en: en.into()
        }
    }

    #[inline]
    pub fn set_speed<DUTY: Into<<<PINS as MotorPins>::PWM as PwmPin>::Duty>>(&mut self, speed: DUTY){
        self.pwm.set_duty(speed.into());
    }

    #[inline]
    pub fn forward(&mut self){
        self.dir.set_low().ok();
    }

    #[inline]
    pub fn reverse(&mut self){
        self.dir.set_high().ok();
    }

    #[inline]
    pub fn enable(&mut self){
        self.en.set_high().ok();
    }

    #[inline]
    pub fn disable(&mut self){
        self.en.set_low().ok();
    }
}


pub struct MotorPair{
    left: Motor<LeftMotor>,
    right: Motor<RightMotor>,
}

impl MotorPair {
    pub fn new(left: Motor<LeftMotor>, right: Motor<RightMotor>) -> Self{
        MotorPair{left, right}
    }

    #[inline]
    pub fn split(self) -> (Motor<LeftMotor>, Motor<RightMotor>){
        (self.left, self.right)
    }

    #[inline]
    pub fn set_speed(&mut self, speed: u16){
        self.left.set_speed(speed);
        self.right.set_speed(speed);
    }

    #[inline]
    pub fn forward(&mut self){
        self.left.forward();
        self.right.forward();
    }

    #[inline]
    pub fn reverse(&mut self){
        self.left.reverse();
        self.right.reverse();
    }

    #[inline]
    pub fn enable(&mut self){
        self.left.enable();
        self.right.enable();
    }

    #[inline]
    pub fn disable(&mut self){
        self.left.disable();
        self.right.disable();
    }
}