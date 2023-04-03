
use embedded_hal::digital::v2::{InputPin, OutputPin, PinState};
use core::convert::Infallible;

pub struct LineSensorArray<OUT0, OUT1, IN0, IN1, IN2, IN3, IN4, IN5, IN6, IN7> 
where OUT0: OutputPin,
      OUT1: OutputPin,
      IN0: InputPin,
      IN1: InputPin,
      IN2: InputPin,
      IN3: InputPin,
      IN4: InputPin,
      IN5: InputPin,
      IN6: InputPin,
      IN7: InputPin,
{
    out0: OUT0,
    out1: OUT1,
    in0: IN0,
    in1: IN1,
    in2: IN2,
    in3: IN3,
    in4: IN4,
    in5: IN5,
    in6: IN6,
    in7: IN7,
}

impl<OUT0, OUT1, IN0, IN1, IN2, IN3, IN4, IN5, IN6, IN7> LineSensorArray<OUT0, OUT1, IN0, IN1, IN2, IN3, IN4, IN5, IN6, IN7>
where OUT0: OutputPin,
      OUT1: OutputPin,
      IN0: InputPin,
      IN1: InputPin,
      IN2: InputPin,
      IN3: InputPin,
      IN4: InputPin,
      IN5: InputPin,
      IN6: InputPin,
      IN7: InputPin,
{
    pub fn new(out0: OUT0, out1: OUT1, in0: IN0, in1: IN1, in2: IN2, in3: IN3, in4: IN4, in5: IN5, in6: IN6, in7: IN7) -> Self {
        LineSensorArray {
            out0,
            out1,
            in0,
            in1,
            in2,
            in3,
            in4,
            in5,
            in6,
            in7,
        }
    }

    pub fn set_leds(&mut self, led_state: bool) {
        if led_state {
            self.out0.set_high().ok().unwrap();
            self.out1.set_high().ok().unwrap();
        } else {
            self.out0.set_low().ok().unwrap();
            self.out1.set_low().ok().unwrap();
        }
    }

    pub fn read(&mut self) -> [bool; 8] {
        let mut result = [false; 8];

        self.set_leds(true);
        result[0] = self.in0.is_high().ok().unwrap();
        result[1] = self.in1.is_high().ok().unwrap();
        result[2] = self.in2.is_high().ok().unwrap();
        result[3] = self.in3.is_high().ok().unwrap();
        result[4] = self.in4.is_high().ok().unwrap();
        result[5] = self.in5.is_high().ok().unwrap();
        result[6] = self.in6.is_high().ok().unwrap();
        result[7] = self.in7.is_high().ok().unwrap();
        self.set_leds(false);

        return result
    }
}