use msp430fr2x5x_hal::gpio::*;


pub struct LineSensor{
    // TODO decide which pins to use
    ctrl_odd: Pin<P1, Pin1, Output>,
    ctrl_even: Pin<P1, Pin1, Output>,
    out0: Pin<P1, Pin1, Output>,
    out1: Pin<P1, Pin1, Output>,
    out2: Pin<P1, Pin1, Output>,
    out3: Pin<P1, Pin1, Output>,
    out4: Pin<P1, Pin1, Output>,
    out5: Pin<P1, Pin1, Output>,
    out6: Pin<P1, Pin1, Output>,
    out7: Pin<P1, Pin1, Output>,
}

impl LineSensor{
    pub fn new(
        ctrl_odd: Pin<P1, Pin1, Output>,
        ctrl_even: Pin<P1, Pin1, Output>,
        out0: Pin<P1, Pin1, Output>,
        out1: Pin<P1, Pin1, Output>,
        out2: Pin<P1, Pin1, Output>,
        out3: Pin<P1, Pin1, Output>,
        out4: Pin<P1, Pin1, Output>,
        out5: Pin<P1, Pin1, Output>,
        out6: Pin<P1, Pin1, Output>,
        out7: Pin<P1, Pin1, Output>,
    ) -> Self{
        // TODO
        LineSensor{
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

    pub fn start_read(&mut self){
        // TODO
    }

    pub fn poll_result(&mut self) -> Option<u8>{
        // TODO
        None
    }
}
