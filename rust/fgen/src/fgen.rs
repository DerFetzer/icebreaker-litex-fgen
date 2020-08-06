use litex_pac::{FGEN, FGEN_LUT};

pub const NUMBER_OF_LUT_SAMPLES: u32 = 2048;
pub const NUMBER_OF_BANK_SAMPLES: u32 = 512;
pub const FRACTION_BITS: u32 = 6;

pub struct FunctionGenerator {
    registers: FGEN,
    lut: FGEN_LUT,
}

#[allow(dead_code)]
impl FunctionGenerator {
    pub fn new(registers: FGEN, lut: FGEN_LUT) -> Self {
        Self { registers, lut }
    }

    pub fn enable(&mut self) {
        unsafe {
            self.registers.en.write(|w| w.bits(1));
        }
    }

    pub fn disable(&mut self) {
        unsafe {
            self.registers.en.write(|w| w.bits(0));
        }
    }

    pub fn get_prescaler(&mut self) -> u32 {
        self.registers.prescaler.read().bits()
    }

    pub fn set_prescaler(&mut self, prescaler: u32) {
        unsafe {
            self.registers.prescaler.write(|w| w.bits(prescaler));
        }
    }

    pub fn get_fcw(&mut self) -> u32 {
        self.registers.fcw.read().bits()
    }

    pub fn set_fcw(&mut self, fcw: u32) {
        unsafe {
            self.registers.fcw.write(|w| w.bits(fcw));
        }
    }

    pub fn read_lut(&mut self) -> [u32; NUMBER_OF_LUT_SAMPLES as usize] {
        let mut lut = [0; NUMBER_OF_LUT_SAMPLES as usize];

        for b in 0..NUMBER_OF_LUT_SAMPLES / NUMBER_OF_BANK_SAMPLES {
            self.registers.lut_page.write(|w| unsafe { w.bits(b) });

            for (i, elem) in self.lut.fgen_lut.iter().enumerate() {
                lut[i + (b * NUMBER_OF_BANK_SAMPLES) as usize] = elem.read().bits();
            }
        }
        lut
    }

    pub fn read_lut_sample(&mut self, index: u32) {
        self.registers
            .lut_page
            .write(|w| unsafe { w.bits(index / NUMBER_OF_BANK_SAMPLES) });
        self.lut.fgen_lut[(index % NUMBER_OF_BANK_SAMPLES) as usize]
            .read()
            .bits();
    }

    pub fn write_lut(&mut self, lut: [u32; NUMBER_OF_LUT_SAMPLES as usize]) {
        for b in 0..NUMBER_OF_LUT_SAMPLES / NUMBER_OF_BANK_SAMPLES {
            self.registers.lut_page.write(|w| unsafe { w.bits(b) });

            for (i, elem) in self.lut.fgen_lut.iter().enumerate() {
                unsafe {
                    elem.write(|w| w.bits(lut[i + (b * NUMBER_OF_BANK_SAMPLES) as usize]));
                }
            }
        }
    }

    pub fn write_lut_sample(&mut self, index: u32, sample: u32) {
        self.registers
            .lut_page
            .write(|w| unsafe { w.bits(index / NUMBER_OF_BANK_SAMPLES) });
        self.lut.fgen_lut[(index % NUMBER_OF_BANK_SAMPLES) as usize]
            .write(|w| unsafe { w.bits(sample) });
    }

    pub fn get_frequency(&mut self, sys_clk: u32) -> f32 {
        sys_clk as f32
            / self.get_prescaler() as f32
            / ((NUMBER_OF_LUT_SAMPLES << FRACTION_BITS) as f32 / self.get_fcw() as f32)
    }
}
