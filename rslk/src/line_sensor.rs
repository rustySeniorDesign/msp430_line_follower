use core::arch::asm;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use msp430::asm;
use msp430fr2x5x_hal::delay::Delay;
use msp430fr2x5x_hal::gpio::*;


// ctrl_even: P2.4
// ctrl_odd: P3.3
// out0: P2.2
// out1: P4.0
// out2: P4.6
// out3: P4.7
// out4: P6.1
// out5: P6.2
// out6: P6.3
// out7: P6.4

pub struct LineSensorReady{
    ctrl_odd: Pin<P2, Pin4, Output>,
    ctrl_even: Pin<P3, Pin3, Output>,
    out0: Pin<P2, Pin2, Output>, // leftmost
    out1: Pin<P4, Pin0, Output>,
    out2: Pin<P4, Pin6, Output>,
    out3: Pin<P4, Pin7, Output>,
    out4: Pin<P6, Pin1, Output>,
    out5: Pin<P6, Pin2, Output>,
    out6: Pin<P6, Pin3, Output>,
    out7: Pin<P6, Pin4, Output>, // rightmost
}

/// Line sensor which is currently measuring sensor outputs
pub struct LineSensorBusy{
    ctrl_odd: Pin<P2, Pin4, Output>,
    ctrl_even: Pin<P3, Pin3, Output>,
    out0: Pin<P2, Pin2, Input<Floating>>, // leftmost
    out1: Pin<P4, Pin0, Input<Floating>>,
    out2: Pin<P4, Pin6, Input<Floating>>,
    out3: Pin<P4, Pin7, Input<Floating>>,
    out4: Pin<P6, Pin1, Input<Floating>>,
    out5: Pin<P6, Pin2, Input<Floating>>,
    out6: Pin<P6, Pin3, Input<Floating>>,
    out7: Pin<P6, Pin4, Input<Floating>>, // rightmost
}

impl LineSensorReady{
    pub fn new(
        ctrl_odd: Pin<P2, Pin4, Output>,
        ctrl_even: Pin<P3, Pin3, Output>,
        out0: Pin<P2, Pin2, Output>,
        out1: Pin<P4, Pin0, Output>,
        out2: Pin<P4, Pin6, Output>,
        out3: Pin<P4, Pin7, Output>,
        out4: Pin<P6, Pin1, Output>,
        out5: Pin<P6, Pin2, Output>,
        out6: Pin<P6, Pin3, Output>,
        out7: Pin<P6, Pin4, Output>,
    ) -> Self{
        // TODO
        LineSensorReady{
            ctrl_odd,
            ctrl_even,
            out0,
            out1,
            out2,
            out3,
            out4,
            out5,
            out6,
            out7,
        }
    }

    pub fn start_read(mut self) -> LineSensorBusy{
        //turn on leds
        self.ctrl_odd.set_high().ok();
        self.ctrl_even.set_high().ok();

        // drive sensor array high
        self.out0.set_high().ok();
        self.out1.set_high().ok();
        self.out2.set_high().ok();
        self.out3.set_high().ok();
        self.out4.set_high().ok();
        self.out5.set_high().ok();
        self.out6.set_high().ok();
        self.out7.set_high().ok();
        for _ in 21u8{ // wait ~10us for sensor output to rise
            asm::nop();
        }

        // leave sensors floating to measure decay time
        LineSensorBusy::new(
            ctrl_odd,
            ctrl_even,
            self.out0.to_input_floating(),
            self.out1.to_input_floating(),
            self.out2.to_input_floating(),
            self.out3.to_input_floating(),
            self.out4.to_input_floating(),
            self.out5.to_input_floating(),
            self.out6.to_input_floating(),
            self.out7.to_input_floating(),
        )
    }
}

impl LineSensorBusy {
    pub fn new(
        ctrl_odd: Pin<P2, Pin4, Output>,
        ctrl_even: Pin<P3, Pin3, Output>,
        out0: Pin<P2, Pin2, Input<Floating>>,
        out1: Pin<P4, Pin0, Input<Floating>>,
        out2: Pin<P4, Pin6, Input<Floating>>,
        out3: Pin<P4, Pin7, Input<Floating>>,
        out4: Pin<P6, Pin1, Input<Floating>>,
        out5: Pin<P6, Pin2, Input<Floating>>,
        out6: Pin<P6, Pin3, Input<Floating>>,
        out7: Pin<P6, Pin4, Input<Floating>>,
    ) -> Self{
        LineSensorBusy{
            ctrl_odd,
            ctrl_even,
            out0,
            out1,
            out2,
            out3,
            out4,
            out5,
            out6,
            out7,
        }
    }

    /// Grabs the current state of the IR sensors and packs them into a single byte value.
    ///
    /// Rightmost sensor is bit 0, leftmost is bit 7
    pub fn peek_result(&self) -> u8{
        unsafe {
            let res = ((self.out0.is_low().ok().unwrap_unchecked() & 0x1u8) << 7u8)
                    | ((self.out1.is_low().ok().unwrap_unchecked() & 0x1u8) << 6u8)
                    | ((self.out2.is_low().ok().unwrap_unchecked() & 0x1u8) << 5u8)
                    | ((self.out3.is_low().ok().unwrap_unchecked() & 0x1u8) << 4u8)
                    | ((self.out4.is_low().ok().unwrap_unchecked() & 0x1u8) << 3u8)
                    | ((self.out5.is_low().ok().unwrap_unchecked() & 0x1u8) << 2u8)
                    | ((self.out6.is_low().ok().unwrap_unchecked() & 0x1u8) << 1u8)
                    | (self.out7.is_low().ok().unwrap_unchecked() & 0x1u8);
            res
        }
    }

    pub fn grab_result(mut self) -> (u8, LineSensorReady) {
        let res = (&mut self).peek_result();
        (res, LineSensorReady::new(
            ctrl_odd,
            ctrl_even,
            self.out0.to_output(),
            self.out1.to_output(),
            self.out2.to_output(),
            self.out3.to_output(),
            self.out4.to_output(),
            self.out5.to_output(),
            self.out6.to_output(),
            self.out7.to_output(),
        ))
    }
}
