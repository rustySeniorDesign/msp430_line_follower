use msp430fr2355::P2;
use msp430fr2x5x_hal::gpio::*;

pub trait EncoderPin{
    type Pin;
}

// pinL : P2.5
// pinR : P4.4

pub struct LeftEncoder;
impl EncoderPin for LeftEncoder{
    type Pin = Pin<P2, Pin5, Input<Floating>>;
}

pub struct RightEncoder;
impl EncoderPin for RightEncoder{
    type Pin = Pin<P4, Pin4, Input<Floating>>;
}

pub struct Encoder<PIN: EncoderPin>{
    pin: PIN::Pin,
}

impl<PIN: EncoderPin> Encoder<PIN>{
    pub fn new<EPIN: Into<PIN::Pin>>(pin: EPIN) -> Self{
        Encoder{
            pin: pin.into()
        }
    }

    pub fn get_speed(&mut self) -> u16{
        // TODO
        0
    }

}

