#![no_std]

#[macro_use]
extern crate nb;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::serial::Read;
use embedded_hal::digital::v2::OutputPin;

const PMS5003_DATA_START: [u8;2] = [0x42, 0x4d];
const PMS5003_DATA_LENGTH: usize = 26;

macro_rules! extract_u16 {
    ($data:expr) => {
        (($data[0] as u16) << 8) | ($data[1] as u16)
    };
    ($data:expr, $offset:expr) => {
        (($data[$offset] as u16) << 8) | ($data[$offset + 1] as u16)
    };
}

macro_rules! checksum {
    ($data:ident) => {{
        let mut checksum = 0u16;
        for i in 0..$data.len() {
            checksum += $data[i] as u16;
        }
        checksum
    }};
}

macro_rules! read {
    ($self:expr, $len:expr) => {{
        let mut data: [u8; $len] = [0; $len];
        for i in 0..$len {
            data[i] = block!($self.tty.read()).map_err(Error::Serial)?;
        }
        data
    }};
}

/// PMS5003 errors
#[derive(Debug)]
pub enum Error<E> {
    /// Checksum failed
    InvalidData(u16, u16),
    /// Length does no match expectation
    InvalidLength(u16),
    /// Serial read error
    Serial(E),
}

#[derive(Debug)]
pub struct Concentrations {
    pub pm1p0: u16,
    pub pm2p5: u16,
    pub pm10p0: u16,
}

#[derive(Debug)]
pub struct Absolutes {
    pub pm0p3: u16,
    pub pm0p5: u16,
    pub pm1p0: u16,
    pub pm2p5: u16,
    pub pm5p0: u16,
    pub pm10p0: u16,
}

#[derive(Debug)]
pub struct Measurement {
    pub ug_per_m3: Concentrations,
    pub ug_per_m3_atmospheric: Concentrations,
    pub per_0p1l: Absolutes,
}

impl Measurement {
    pub fn parse(data: [u8;PMS5003_DATA_LENGTH]) -> Self {
        Measurement {
            ug_per_m3: Concentrations {
                pm1p0: extract_u16!(data, 0),
                pm2p5: extract_u16!(data, 2),
                pm10p0: extract_u16!(data, 4),
            },
            ug_per_m3_atmospheric: Concentrations {
                pm1p0: extract_u16!(data, 6),
                pm2p5: extract_u16!(data, 8),
                pm10p0: extract_u16!(data, 10),
            },
            per_0p1l: Absolutes {
                pm0p3: extract_u16!(data, 12),
                pm0p5: extract_u16!(data, 14),
                pm1p0: extract_u16!(data, 16),
                pm2p5: extract_u16!(data, 18),
                pm5p0: extract_u16!(data, 20),
                pm10p0: extract_u16!(data, 22),
            },
        }
    }
}

pub struct PMS5003<TTY, P, D> {
    /// Serial port
    tty: TTY,

    /// Data/command pin
    dc: P,

    /// Reset pin
    rst: P,

    /// Timer
    delay: D,
}

impl<TTY, P, D, E, PE> PMS5003<TTY, P, D>
where
    TTY: Read<u8, Error = E>,
    P: OutputPin<Error = PE>,
    D: DelayMs<u8>,
{
    pub fn new(tty: TTY, dc: P, rst: P, delay: D) -> Self {
        PMS5003 { tty, dc, rst, delay }
    }

    pub fn init(&mut self) -> Result<(), PE> {
        self.dc.set_high()?;
        self.rst.set_high()?;
        self.reset()
    }

    pub fn reset(&mut self) -> Result<(), PE> {
        self.delay.delay_ms(100);
        self.rst.set_low()?;
        self.delay.delay_ms(100);
        self.rst.set_high()?;
        Ok(())
    }

    pub fn measure(&mut self) -> Result<Measurement, Error<E>> {
        let mut expect = 0;
        loop {
            let byte = block!(self.tty.read()).map_err(Error::Serial)?;
            expect = if byte == PMS5003_DATA_START[expect] { expect + 1 } else { 0 };
            if expect >= PMS5003_DATA_START.len() {
                break;
            }
        }

        let raw_length = read!(self, 2);
        let length = extract_u16!(raw_length);
        if length != (PMS5003_DATA_LENGTH + 2) as u16 {
            return Err(Error::InvalidLength(length));
        }
        let data = read!(self, PMS5003_DATA_LENGTH);
        let raw_expected = read!(self, 2);
        let expected = extract_u16!(raw_expected);
        let received = checksum!(PMS5003_DATA_START) + checksum!(raw_length) + checksum!(data);
        if received != expected {
            return Err(Error::InvalidData(received, expected));
        }
        Ok(Measurement::parse(data))
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }

}
