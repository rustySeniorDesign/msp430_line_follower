#![no_std]
#![no_main]
#![feature(abi_msp430_interrupt)]
#![feature(core_panic)]

mod line_sensor;
mod motor;
mod queuebuf;
mod serial_utils;

use core::{
    panic::PanicInfo,
    option::Option::Some,
};
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
use serial_utils::{
    init_serial,
    print_bytes
};

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
            .mclk_dcoclk(DcoclkFreqSel::_16MHz, MclkDiv::_2)
            .smclk_on(SmclkDiv::_1)
            .aclk_refoclk()
            .freeze(&mut fram);
        let pmm = Pmm::new(periph.PMM);

        let p4 = Batch::new(periph.P4).split(&pmm);
        let p5 = Batch::new(periph.P5).split(&pmm);
        let p7 = Batch::new(periph.P7).split(&pmm);
        let p9 = Batch::new(periph.P9).split(&pmm);

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

        print_bytes(b"Serial started");

        let mut line_sensor = line_sensor::LineSensorArray::new(
            p5.pin3.to_output(),
            p9.pin2.to_output(),
            p7.pin0.pulldown(),
            p7.pin1.pulldown(),
            p7.pin2.pulldown(),
            p7.pin3.pulldown(),
            p7.pin4.pulldown(),
            p7.pin5.pulldown(),
            p7.pin6.pulldown(),
            p7.pin7.pulldown(),
        );

        loop {
            asm::barrier();
        }
    }
    loop{}
}

#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}