//! nrf52 blink example
//!
//! This is for the nrf52840-dk.
//!
//! LED1 will blink SOS in morse code
//! LED2 will blink HELLO. in morse code
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m::{
    peripheral::SCB,
    asm::delay,
};
use embedded_hal::blocking::delay::DelayMs;
use nrf52840_hal::{
    self as hal,
    gpio::{
        p0::{Parts as P0Parts, P0_13, P0_14},
        Level, Output, PushPull,
    },
    spim::{Spim, Pins, Frequency, MODE_0},
    Timer,
    Rng,
};
use panic_reset as _;
use rtt_target::{rprintln, rtt_init_print};
use core::sync::atomic::{compiler_fence, Ordering};
use libm::{fminf, fmaxf};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let board = hal::pac::Peripherals::take().unwrap();

    // if !board.UICR.regout0.read().vout().is_3v3() {
    //     // Enable erase
    //     board.NVMC.config.write(|w| {
    //         w.wen().een()
    //     });
    //     while board.NVMC.ready.read().ready().is_busy() {}

    //     // Erase regout0 page
    //     board.NVMC.erasepage().write(|w| unsafe {
    //         w.erasepage().bits(&board.UICR.regout0 as *const _ as u32)
    //     });
    //     while board.NVMC.ready.read().ready().is_busy() {}

    //     // enable write
    //     board.NVMC.config.write(|w| {
    //         w.wen().wen()
    //     });
    //     while board.NVMC.ready.read().ready().is_busy() {}

    //     // Set 3v3 setting
    //     board.UICR.regout0.write(|w| {
    //         w.vout()._3v3()
    //     });
    //     while board.NVMC.ready.read().ready().is_busy() {}

    //     // Return UCIR to read only
    //     board.NVMC.config.write(|w| {
    //         w.wen().ren()
    //     });
    //     while board.NVMC.ready.read().ready().is_busy() {}

    //     // system reset
    //     SCB::sys_reset();
    // }

    let mut timer = Timer::new(board.TIMER0);
    let gpios = P0Parts::new(board.P0);

    let _led1 = gpios.p0_13.into_push_pull_output(Level::High);
    let _led2 = gpios.p0_14.into_push_pull_output(Level::High);

    use embedded_hal::digital::v2::OutputPin;

    let sdout = gpios.p0_30.into_push_pull_output(Level::Low);
    let sck = gpios.p0_08.into_push_pull_output(Level::Low);
    let mut lrck = gpios.p0_09.into_push_pull_output(Level::Low);
    let _mck = gpios.p0_10.into_push_pull_output(Level::Low);

    let mut rng = Rng::new(board.RNG);

    // board.I2S.config.rxen.modify(|_r, w| w.rxen().disabled());
    // board.I2S.config.txen.modify(|_r, w| w.txen().enabled());
    // board.I2S.config.mcken.modify(|_r, w| w.mcken().enabled());
    // board.I2S.config.mckfreq.modify(|_r, w| w.mckfreq()._32mdiv10());
    // board.I2S.config.ratio.modify(|_r, w| w.ratio()._32x());
    // board.I2S.config.swidth.modify(|_r, w| w.swidth()._16bit());
    // board.I2S.config.align.modify(|_r, w| w.align().left());
    // board.I2S.config.format.modify(|_r, w| w.format().i2s());
    // board.I2S.config.channels.modify(|_r, w| w.channels().stereo());

    // // ?
    // board.I2S.config.mode.modify(|_r, w| w.mode().master());

    // // "optional"
    // // board.I2S.psel.mck.modify(|_r, w| {
    // //     w.port().clear_bit();
    // //     unsafe { w.pin().bits(10) };
    // //     w.connect().connected()
    // // });

    // // "required"
    // board.I2S.psel.lrck.modify(|_r, w| {
    //     w.port().clear_bit();
    //     unsafe { w.pin().bits(9) };
    //     w.connect().connected()
    // });

    // // "required"
    // board.I2S.psel.sck.modify(|_r, w| {
    //     w.port().clear_bit();
    //     unsafe { w.pin().bits(8) };
    //     w.connect().connected()
    // });

    // board.I2S.psel.sdout.modify(|_r, w| {
    //     w.port().clear_bit();
    //     unsafe { w.pin().bits(30) };
    //     w.connect().connected()
    // });

    // let mut buffy = [0u32; 9];
    // let mut buffy2 = [0u32; 9];

    // buffy.copy_from_slice(&[
    //     0x88888888,
    //     0xCCCCCCCC,
    //     0xCCCCCCCC,
    //     0x00000000,
    //     0x00000000,
    //     0x00000000,
    //     0x00000000,
    //     0x00000000,
    //     0x00000000,
    // ]);
    // buffy2.copy_from_slice(&[
    //     0xCCCCCCCC,
    //     0x88888888,
    //     0xCCCCCCCC,
    //     0x00000000,
    //     0x00000000,
    //     0x00000000,
    //     0x00000000,
    //     0x00000000,
    //     0x00000000,
    // ]);

    // let addr = buffy.as_ptr() as u32;
    // let addr2 = buffy2.as_ptr() as u32;

    // assert!((addr % 4) == 0);
    // assert!((addr2 % 4) == 0);
    // rprintln!("0x{:08X} - {:X?}", addr, buffy);
    // rprintln!("0x{:08X} - {:X?}", addr2, buffy2);

    // loop {
    //     rprintln!("Sleep...");
    //     timer.delay_ms(1000u32);
    //     board.I2S.rxtxd.maxcnt.write(|w| unsafe {
    //         w.maxcnt().bits(buffy.len() as u16)
    //     });
    //     board.I2S.txd.ptr.write(|w| unsafe {
    //         w.ptr().bits(addr)
    //     });
    //     board.I2S.config.txen.write(|w| w.txen().enabled());
    //     compiler_fence(Ordering::SeqCst);
    //     board.I2S.enable.write(|w| w.enable().enabled());

    //     board.I2S.events_txptrupd.write(|w| w.events_txptrupd().clear_bit());
    //     board.I2S.events_stopped.write(|w| w.events_stopped().clear_bit());

    //     assert!(board.I2S.events_txptrupd.read().events_txptrupd().bit_is_clear());
    //     board.I2S.tasks_start.write(|w| w.tasks_start().set_bit());

    //     compiler_fence(Ordering::SeqCst);

    //     rprintln!("A");
    //     while board.I2S.events_txptrupd.read().events_txptrupd().bit_is_clear() {}
    //     rprintln!("B");
    //     // board.I2S.events_txptrupd.write(|w| w);
    //     // rprintln!("C");
    //     // board.I2S.rxtxd.maxcnt.write(|w| unsafe {
    //     //     w.maxcnt().bits(buffy2.len() as u16)
    //     // });
    //     // board.I2S.txd.ptr.write(|w| unsafe {
    //     //     w.ptr().bits(addr2)
    //     // });
    //     rprintln!("D");
    //     // while !(board.I2S.events_txptrupd.read().events_txptrupd().bit_is_set() || board.I2S.events_stopped.read().events_stopped().bit_is_set()) {}
    //     // rprintln!("E");
    //     timer.delay_ms(250u32);

    //     board.I2S.tasks_stop.write(|w| w.tasks_stop().set_bit());
    //     unsafe {
    //         core::ptr::write_volatile(0x40025038 as *mut u32, 1);
    //         core::ptr::write_volatile(0x4002503C as *mut u32, 1);
    //     }

    //     compiler_fence(Ordering::SeqCst);

    //     while board.I2S.events_stopped.read().events_stopped().bit_is_clear() {}
    //     board.I2S.events_stopped.write(|w| w.events_stopped().clear_bit());

    //     // timer.delay_ms(1u32);
    //     board.I2S.enable.write(|w| w.enable().disabled());
    //     compiler_fence(Ordering::SeqCst);
    // }

    // loop {
    //     for i in 0..3 {



    //         for _ in 0..8 {
    //             sdout.set_high().ok();
    //             delay(19);
    //             sdout.set_low().ok();
    //             delay(58);
    //         }

    //         for _ in 0..8 {
    //             sdout.set_high().ok();
    //             delay(19);
    //             sdout.set_low().ok();
    //             delay(58);
    //         }
    //         for _ in 0..8 {
    //             sdout.set_high().ok();
    //             delay(38);
    //             sdout.set_low().ok();
    //             delay(38);
    //         }
    //         // for _ in 0..8 {
    //         //     sdout.set_high().ok();
    //         //     delay(38);
    //         //     sdout.set_low().ok();
    //         //     delay(38);
    //         // }
    //     }

    //     timer.delay_ms(1000u32);

    //     for i in 0..3 {
    //         for _ in 0..8 {
    //             sdout.set_high().ok();
    //             delay(19);
    //             sdout.set_low().ok();
    //             delay(58);
    //         }
    //     }

    //     timer.delay_ms(1000u32);
    // }

    // let mut spim = Spim::new(
    //     board.SPIM0,
    //     Pins {
    //         sck: sck.degrade(),
    //         mosi: Some(sdout.degrade()),
    //         miso: None,
    //     },
    //     Frequency::M8,
    //     MODE_0,
    //     0x00
    // );

    // let mut buf = [0u8; 72];


    // let mut lrck = lrck.degrade();

    // loop {
    //     rprintln!("Start");
    //     buf.copy_from_slice(&[
    //         0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
    //         0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
    //         0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
    //         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     ]);
    //     spim.write(&mut lrck, &buf).ok();
    //     timer.delay_ms(1000u32);

    //     // rprintln!("Two");
    //     // buf.copy_from_slice(&[
    //     //     0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
    //     //     0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
    //     //     0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     // ]);
    //     // spim.write(&mut lrck, &buf).ok();
    //     // timer.delay_ms(1000u32);

    //     // rprintln!("Three");
    //     // buf.copy_from_slice(&[
    //     //     0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
    //     //     0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
    //     //     0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     // ]);
    //     // spim.write(&mut lrck, &buf).ok();
    //     // timer.delay_ms(1000u32);

    //     // rprintln!("Four");
    //     // buf.copy_from_slice(&[
    //     //     0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
    //     //     0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
    //     //     0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     // ]);
    //     // spim.write(&mut lrck, &buf).ok();
    //     // timer.delay_ms(1000u32);

    //     rprintln!("Five");
    //     buf.copy_from_slice(&[
    //         0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
    //         0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
    //         0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
    //         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     ]);
    //     spim.write(&mut lrck, &buf).ok();
    //     timer.delay_ms(1000u32);

    // }

    board.PWM0.psel.out[0].write(|w| {
        w.port().clear_bit();
        unsafe { w.pin().bits(30) };
        w.connect().connected()
    });
    board.PWM0.enable.write(|w| w.enable().enabled());
    board.PWM0.mode.write(|w| w.updown().up());
    board.PWM0.prescaler.write(|w| w.prescaler().div_1());
    board.PWM0.countertop.write(|w| unsafe { w.countertop().bits(20) });
    board.PWM0.loop_.write(|w| w.cnt().disabled());
    board.PWM0.decoder.write(|w| {
        w.load().common();
        w.mode().refresh_count()
    });

    let mut buf = [0x8000u16; 45 + (24 * 1) + 1];

    let mut old_red: f32 = 0f32;
    let mut old_green: f32 = 0f32;
    let mut old_blue: f32 = 0f32;

    loop {
        let new_red = rng.random_u8() as f32;
        let new_green = rng.random_u8() as f32;
        let new_blue = rng.random_u8() as f32;

        let delta_red = new_red - old_red;
        let delta_green = new_green - old_green;
        let delta_blue = new_blue - old_blue;

        rprintln!("Start!");

        rprintln!("Red: {} => {}", old_red, new_red);
        rprintln!("Green: {} => {}", old_green, new_green);
        rprintln!("Blue: {} => {}", old_blue, new_blue);

        for i in 1..=64 {

            // // rgbr_gbrg
            // let red: u8 = (i & 0x80) | ((i & 0x10) << 2) | ((i & 0x02) << 4);
            // let green: u8 = ((i & 0x40) << 1) | ((i & 0x08) << 3) | ((i & 0x01) << 5);
            // let blue: u8 = ((i & 0x20) << 2) | ((i & 0x04) << 4);

            let red = fminf(255f32, fmaxf(0f32, (old_red + ((i as f32 / 64f32) * delta_red)))) as u8;
            let green = fminf(255f32, fmaxf(0f32, (old_green + ((i as f32 / 64f32) * delta_green)))) as u8;
            let blue = fminf(255f32, fmaxf(0f32, (old_blue + ((i as f32 / 64f32) * delta_blue)))) as u8;

            // rprintln!("r: {}, g: {}, b: {}", red, green, blue);

            let red = GAMMA8[red as usize];
            let green = GAMMA8[green as usize];
            let blue = GAMMA8[blue as usize];

            let red = red.reverse_bits();
            let green = green.reverse_bits();
            let blue = blue.reverse_bits();

            for g in 0..8 {
                if ((green >> g) & 0b1) == 0b1 {
                    buf[45 + g] = 0x8000 | 13;
                } else {
                    buf[45 + g] = 0x8000 | 5;
                }
            }

            for r in 0..8 {
                if ((red >> r) & 0b1) == 0b1 {
                    buf[45 + 8 + r] = 0x8000 | 13;
                } else {
                    buf[45 + 8 + r] = 0x8000 | 5;
                }
            }

            for b in 0..8 {
                if ((blue >> b) & 0b1) == 0b1 {
                    buf[45 + 16 + b] = 0x8000 | 13;
                } else {
                    buf[45 + 16 + b] = 0x8000 | 5;
                }
            }

            unsafe {
                board.PWM0.seq0.refresh.write(|w| w.bits(0));
                board.PWM0.seq0.enddelay.write(|w| w.bits(0));
                board.PWM0.seq0.ptr.write(|w| w.bits(buf.as_ptr() as u32));
                board.PWM0.seq0.cnt.write(|w| w.bits(buf.len() as u32))
            }

            compiler_fence(Ordering::SeqCst);

            board.PWM0.tasks_seqstart[0].write(|w| w.tasks_seqstart().set_bit());

            timer.delay_ms(15u32);
        }

        old_red = new_red;
        old_green = new_green;
        old_blue = new_blue;

        timer.delay_ms(3000u32);

    }

}

const GAMMA8: [u8; 256] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4,
    4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11,
    12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22,
    22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36, 37,
    38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50, 51, 52, 54, 55, 56, 57, 58,
    59, 60, 61, 62, 63, 64, 66, 67, 68, 69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85,
    86, 87, 89, 90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
    115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142, 144,
    146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175, 177, 180,
    182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213, 215, 218, 220,
    223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255,
];
