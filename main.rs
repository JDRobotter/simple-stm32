#![no_main]
#![no_std]

mod i2c_wrapper;
use i2c_wrapper::I2CWrapper;

mod dgpio;
use dgpio::{DynamicGpioPin, GpioPinConfiguration, IntoDynamicPin};

// Setup startup code and minimal runtime for uC
// (check https://docs.rs/cortex-m-rt/latest/cortex_m_rt/)
use cortex_m_rt::entry;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use stm32f0xx_hal::{
    gpio, i2c, pac,
    prelude::*,
    serial,
    time::{Bps, Hertz},
    timers::Timer,
};

use embedded_graphics::{
    mono_font::{
        ascii::{FONT_4X6, FONT_8X13, FONT_9X18},
        MonoTextStyleBuilder,
    },
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{rectangle::Rectangle, PrimitiveStyleBuilder},
    text::{Baseline, Text},
};
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

use static_cell::StaticCell;
type SERIAL = serial::Serial<
    pac::USART1,
    gpio::gpioa::PA9<gpio::Alternate<gpio::AF1>>,
    gpio::gpioa::PA10<gpio::Alternate<gpio::AF1>>,
>;

use embedded_io::Write;
struct SerialWrapper {
    s: SERIAL,
}

impl embedded_io::ErrorType for SerialWrapper {
    type Error = core::convert::Infallible;
}

impl embedded_io::Write for SerialWrapper {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for c in buf {
            nb::block!(self.s.write(*c)).ok();
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.s.flush().ok();
        Ok(())
    }
}

static SERIALCELL: StaticCell<SerialWrapper> = StaticCell::new(); //SerialDevice = SerialDevice::new();

use irq::{handler, scope, scoped_interrupts};
use stm32f0xx_hal::pac::interrupt;
scoped_interrupts! {
    enum Interrupt {
        EXTI0_1,
    }

    use #[interrupt];
}

use core::panic::PanicInfo;
#[panic_handler]
fn panic(pi: &PanicInfo) -> ! {
    for _ in 1..100 {
        defmt::error!("-- PANIC --");
        if let Some(l) = pi.location() {
            defmt::error!("{}:{}", l.file(), l.line());
        }
        defmt::error!("{}", pi.message().as_str());
        cortex_m::asm::delay(10_000_000);
    }
    // reboot in bootloader mode
    cortex_m::peripheral::SCB::sys_reset();
}

#[entry]
fn main() -> ! {
    let mut p = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // configure clock frequency
    let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);

    let gpioa = p.GPIOA.split(&mut rcc);
    let gpiob = p.GPIOB.split(&mut rcc);
    let gpiof = p.GPIOF.split(&mut rcc);

    // -- DEBUG SERIAL LINK --
    // configure debug serial link
    cortex_m::interrupt::free(|cs| {
        let tx = gpioa.pa9.into_alternate_af1(&cs);
        let rx = gpioa.pa10.into_alternate_af1(&cs);
        let s = serial::Serial::usart1(p.USART1, (tx, rx), Bps(115200), &mut rcc);
        let _ = defmt_serial::defmt_serial(SERIALCELL.init(SerialWrapper { s }));
        defmt::info!("SERIAL SETUP");
    });
    let mut ax = cortex_m::interrupt::free(|cs| {
        (
            gpioa.pa15.into_dynamic(),
            gpioa.pa13.into_dynamic(),
            gpiob.pb4.into_dynamic(),
            gpioa.pa1.into_dynamic(),
            gpiob.pb6.into_dynamic(),
            gpiof.pf0.into_dynamic(),
        )
    });

    use cortex_m::interrupt::CriticalSection;
    let set_ax_mode = |n: usize, mode: GpioPinConfiguration| {
        cortex_m::interrupt::free(|cs| match n {
            0 => ax.0.configure(mode, &cs),
            1 => ax.1.configure(mode, &cs),
            2 => ax.2.configure(mode, &cs),
            3 => ax.3.configure(mode, &cs),
            4 => ax.4.configure(mode, &cs),
            5 => ax.5.configure(mode, &cs),
            _ => panic!(),
        })
    };
    let get_ax_state = |n: usize| -> bool {
        match n {
            0 => ax.0.get_state(),
            1 => ax.1.get_state(),
            2 => ax.2.get_state(),
            3 => ax.3.get_state(),
            4 => ax.4.get_state(),
            5 => ax.5.get_state(),
            _ => panic!(),
        }
    };
    let set_ax_state = |n: usize, b: bool| match n {
        0 => ax.0.set_state(b),
        1 => ax.1.set_state(b),
        2 => ax.2.set_state(b),
        3 => ax.3.set_state(b),
        4 => ax.4.set_state(b),
        5 => ax.5.set_state(b),
        _ => panic!(),
    };

    /*
    let mut ay = cortex_m::interrupt::free(|cs| {
        [
            gpioa.pa14.into_floating_input(&cs).downgrade(),
            gpioa.pa8.into_floating_input(&cs).downgrade(),
            gpiob.pb3.into_floating_input(&cs).downgrade(),
            gpioa.pa0.into_floating_input(&cs).downgrade(),
            gpiob.pb5.into_floating_input(&cs).downgrade(),
            gpiob.pb7.into_floating_input(&cs).downgrade(),
        ]
    });*/

    // -- DEEP SLEEP --
    // configure uC DEEP SLEEP mode according to RM0091 p.89
    cp.SCB.set_sleepdeep();
    // configure stop mode on deep sleep and voltage regulator in low-power during stop mode
    p.PWR
        .cr
        .modify(|_, w| w.pdds().stop_mode().lpds().set_bit());

    // -- EXTERNAL EVENT --
    // wire EXTI0 line on PA0 GPIO
    p.SYSCFG.exticr1.modify(|_, w| w.exti0().pa0());
    // configure EXTI interrupt mask for EXTI0
    p.EXTI.imr.modify(|_, w| w.mr0().unmasked());
    // setup falling-edge trigger on EXTI0
    p.EXTI.ftsr.modify(|_, w| w.tr0().enabled());
    p.EXTI.rtsr.modify(|_, w| w.tr0().disabled());

    // wrap EXTI in crirtical section mutex to share it
    let exti = Mutex::new(p.EXTI);

    // setup SSD1306 OLED screen
    defmt::info!("I2C SETUP");
    let (scl, sda) = cortex_m::interrupt::free(|cs| {
        (
            gpioa.pa11.into_alternate_af5(&cs),
            gpioa.pa12.into_alternate_af5(&cs),
        )
    });
    let mut i2c = i2c::I2c::i2c1(p.I2C1, (scl, sda), 100.khz(), &mut rcc);
    let i2c = I2CWrapper::wrap(i2c);
    // setup display interface
    defmt::info!("DISPLAY SETUP 2");
    let i2c = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(i2c, DisplaySize128x64, DisplayRotation::Rotate180)
        .into_buffered_graphics_mode();
    defmt::info!("DISPLAY INIT");
    display.init().unwrap();
    defmt::info!("DONE");
    display.clear(BinaryColor::Off).ok();
    display.flush().ok();

    handler!(
        exti01_handler = || {
            cortex_m::interrupt::free(|cs| {});
        }
    );

    let ts = MonoTextStyleBuilder::new()
        .font(&FONT_9X18)
        .text_color(BinaryColor::On)
        .build();
    let style = PrimitiveStyleBuilder::new()
        .stroke_color(BinaryColor::On)
        .stroke_width(1)
        .fill_color(BinaryColor::Off)
        .build();
    let filled = PrimitiveStyleBuilder::new()
        .stroke_color(BinaryColor::On)
        .stroke_width(1)
        .fill_color(BinaryColor::On)
        .build();

    scope(|scope| {
        scope.register(Interrupt::EXTI0_1, exti01_handler);

        // unmask EXTI0 interrupt
        unsafe {
            use cortex_m::peripheral::NVIC;
            //NVIC::unmask(pac::interrupt::EXTI0_1);
        }

        let mut t = 0;
        loop {
            // CHECK CONNECTIVITY
            defmt::info!("----------");
            let mut matrix = [false; 6 * 6];
            for j in 0..6 {
                // set one line high
                set_ax_mode(j, GpioPinConfiguration::OutputPushPull);
                set_ax_state(j, true);

                cortex_m::asm::delay(300_000);
                // read all lines
                for i in 0..6 {
                    let b = get_ax_state(i);
                    matrix[i + j * 6] = b;
                }

                set_ax_state(j, false);
                cortex_m::asm::delay(300_000);
                set_ax_mode(j, GpioPinConfiguration::InputHighImpedance);
            }

            // UPDATE DISPLAY
            display.clear(BinaryColor::Off).ok();
            for j in 0..6 {
                for i in 0..6 {
                    let x = 10 * i as i32;
                    let y = 10 * j as i32;
                    let b = matrix[i + j * 6];
                    Rectangle::new(Point::new(x, y), Size::new(10, 10))
                        .into_styled(if b { filled } else { style })
                        .draw(&mut display)
                        .ok();
                }
            }
            /*
                        for i in 0..6 {
                            let x = 20 * i;
                            let y = 24;
                            Rectangle::new(Point::new(x, y), Size::new(20, 20))
                                .into_styled(style)
                                .draw(&mut display)
                                .ok();
                            /*
                            Text::with_baseline(txt, Point::zero(), ts, Baseline::Top)
                                .draw(&mut display)
                                .ok();*/
                        }
                        for i in 0..6 {
                            let x = 20 * i;
                            let y = 44;
                            Rectangle::new(Point::new(x, y), Size::new(20, 20))
                                .into_styled(style)
                                .draw(&mut display)
                                .ok();
                            /*
                            Text::with_baseline(txt, Point::zero(), ts, Baseline::Top)
                                .draw(&mut display)
                                .ok();*/
                        }
            */
            /*t += 1;
                        Rectangle::new(Point::new(t % (128 - 5), 1), Size::new(5, 15))
                            .into_styled(style)
                            .draw(&mut display)
                            .ok();
            */
            display.flush().ok();
            cortex_m::asm::delay(100_000);
        }
    })
}
