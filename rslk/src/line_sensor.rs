use embedded_hal::digital::v2::{InputPin, OutputPin};
use msp430::asm;
use msp430fr2x5x_hal::gpio::*;


// ctrl_even: P4.0
// ctrl_odd: P6.1
// out0: P4.7
// out1: P6.3
// out2: P4.6
// out3: P6.2
// out4: P6.4
// out5: P2.4
// out6: P3.7
// out7: P2.2

pub struct LineSensorReady{
    ctrl_odd: Pin<P6, Pin1, Output>,
    ctrl_even: Pin<P4, Pin0, Output>,
    out0: Pin<P4, Pin7, Output>, // leftmost
    out1: Pin<P6, Pin3, Output>,
    out2: Pin<P4, Pin6, Output>,
    out3: Pin<P6, Pin2, Output>,
    out4: Pin<P6, Pin4, Output>,
    out5: Pin<P2, Pin4, Output>,
    out6: Pin<P3, Pin7, Output>,
    out7: Pin<P2, Pin2, Output>, // rightmost
}

/// Line sensor which is currently measuring sensor outputs
pub struct LineSensorBusy{
    ctrl_odd: Pin<P6, Pin1, Output>,
    ctrl_even: Pin<P4, Pin0, Output>,
    out0: Pin<P4, Pin7, Input<Floating>>, // leftmost
    out1: Pin<P6, Pin3, Input<Floating>>,
    out2: Pin<P4, Pin6, Input<Floating>>,
    out3: Pin<P6, Pin2, Input<Floating>>,
    out4: Pin<P6, Pin4, Input<Floating>>,
    out5: Pin<P2, Pin4, Input<Floating>>,
    out6: Pin<P3, Pin7, Input<Floating>>,
    out7: Pin<P2, Pin2, Input<Floating>>, // rightmost
}

impl LineSensorReady{
    pub fn new(
        mut ctrl_odd: Pin<P6, Pin1, Output>,
        mut ctrl_even: Pin<P4, Pin0, Output>,
        out0: Pin<P4, Pin7, Output>,
        out1: Pin<P6, Pin3, Output>,
        out2: Pin<P4, Pin6, Output>,
        out3: Pin<P6, Pin2, Output>,
        out4: Pin<P6, Pin4, Output>,
        out5: Pin<P2, Pin4, Output>,
        out6: Pin<P3, Pin7, Output>,
        out7: Pin<P2, Pin2, Output>,
    ) -> Self{
        // TODO
        ctrl_odd.set_high().ok();
        ctrl_even.set_high().ok();
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
        // drive sensor array high
        self.out0.set_high().ok();
        self.out1.set_high().ok();
        self.out2.set_high().ok();
        self.out3.set_high().ok();
        self.out4.set_high().ok();
        self.out5.set_high().ok();
        self.out6.set_high().ok();
        self.out7.set_high().ok();
        for _ in 0..21u8{ // wait ~10us for sensor output to rise
            asm::nop();
        }

        // leave sensors floating to measure decay time
        LineSensorBusy::new(
            self.ctrl_odd,
            self.ctrl_even,
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
        ctrl_odd: Pin<P6, Pin1, Output>,
        ctrl_even: Pin<P4, Pin0, Output>,
        out0: Pin<P4, Pin7, Input<Floating>>,
        out1: Pin<P6, Pin3, Input<Floating>>,
        out2: Pin<P4, Pin6, Input<Floating>>,
        out3: Pin<P6, Pin2, Input<Floating>>,
        out4: Pin<P6, Pin4, Input<Floating>>,
        out5: Pin<P2, Pin4, Input<Floating>>,
        out6: Pin<P3, Pin7, Input<Floating>>,
        out7: Pin<P2, Pin2, Input<Floating>>,
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

    /// Grabs the current logic state of the IR sensors and packs them into a single byte value.
    ///
    /// Keeps the pins floating.
    ///
    /// Rightmost sensor is bit 0, leftmost is bit 7
    pub fn peek_result(&self) -> u8{
        unsafe {
            let res = ((self.out0.is_high().ok().unwrap_unchecked() as u8) << 7u8)
                    | ((self.out1.is_high().ok().unwrap_unchecked() as u8) << 6u8)
                    | ((self.out2.is_high().ok().unwrap_unchecked() as u8) << 5u8)
                    | ((self.out3.is_high().ok().unwrap_unchecked() as u8) << 4u8)
                    | ((self.out4.is_high().ok().unwrap_unchecked() as u8) << 3u8)
                    | ((self.out5.is_high().ok().unwrap_unchecked() as u8) << 2u8)
                    | ((self.out6.is_high().ok().unwrap_unchecked() as u8) << 1u8)
                    | (self.out7.is_high().ok().unwrap_unchecked() as u8);
            res
        }
    }

    /// Grabs the IR state and reverts pins back to output mode in preparation for a new read.
    ///
    /// Rightmost sensor is bit 0, leftmost is bit 7
    pub fn grab_result(mut self) -> (u8, LineSensorReady) {
        let res = (&mut self).peek_result();
        // self.ctrl_odd.set_low().ok();
        // self.ctrl_even.set_low().ok();
        (res, LineSensorReady::new(
            self.ctrl_odd,
            self.ctrl_even,
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
