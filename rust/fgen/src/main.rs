#![no_std]
#![no_main]

extern crate panic_halt;

use litex_pac;
use riscv_rt::entry;

mod fgen;
mod leds;
mod print;
mod timer;

use crate::fgen::{FunctionGenerator, NUMBER_OF_LUT_SAMPLES};
use leds::Leds;
use timer::Timer;

const SYSTEM_CLOCK_FREQUENCY: u32 = 18_000_000;

const KHZ: u32 = 233;

// This is the entry point for the application.
// It is not allowed to return.
#[entry]
fn main() -> ! {
    let peripherals = litex_pac::Peripherals::take().unwrap();

    print::print_hardware::set_hardware(peripherals.UART);
    let mut timer = Timer::new(peripherals.TIMER0);
    let mut leds = Leds::new(peripherals.LEDS);
    leds.set_single(true, false, true, false, true, false, true);
    let gpiob = peripherals.GPIOB;

    let mut fgen = FunctionGenerator::new(peripherals.FGEN, peripherals.FGEN_LUT);

    fgen.set_prescaler(32);
    fgen.set_fcw(1 * KHZ);

    for i in 0..NUMBER_OF_LUT_SAMPLES {
        fgen.write_lut_sample(i, i);
    }

    fgen.enable();

    loop {
        if gpiob.in_.read().userbtn1().bit() {
            //print!("a");
            fgen.set_fcw(4 * KHZ);
        } else if gpiob.in_.read().userbtn2().bit() {
            //print!("b");
            fgen.set_fcw(3 * KHZ);
        } else if gpiob.in_.read().userbtn3().bit() {
            //print!("c");
            fgen.set_fcw(2 * KHZ);
        } else {
            //print!("d");
            fgen.set_fcw(1 * KHZ);
        }
        leds.toggle();
        msleep(&mut timer, 160);
    }
}

fn msleep(timer: &mut Timer, ms: u32) {
    timer.disable();

    timer.reload(0);
    timer.load(SYSTEM_CLOCK_FREQUENCY / 1_000 * ms);

    timer.enable();

    // Wait until the time has elapsed
    while timer.value() > 0 {}
    timer.disable();
}
