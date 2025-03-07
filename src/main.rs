#![no_std]
#![no_main]
use core::cell::RefCell;
use core::iter::once;

use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi;
use embassy_rp::spi::{Blocking, Spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Delay, Duration, Instant, Timer};

{% if features contains graphics %}
use embedded_graphics::image::{Image, ImageRawLE};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyleBuilder, Rectangle, StyledDrawable};
{% endif %}


use itertools::*;
use micromath::F32Ext;
use mipidsi::dcs::DcsCommand;
use mipidsi::models::GC9A01;
use mipidsi::options::{ColorInversion, ColorOrder, RefreshOrder};
use mipidsi::Builder;
use rand_core::RngCore;
use {defmt_rtt as _, panic_probe as _};

const DISPLAY_FREQ: u32 = 64_000_000;
const LCD_X_RES: i32 = 240;
const LCD_Y_RES: i32 = 240;
const FERRIS_WIDTH: u32 = 86;
const FERRIS_HEIGHT: u32 = 64;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut rng = RoscRng;

    // let mut backlight = Output::new(p.PIN_22, Level::Low);
    // let mut data_command = Output::new(p.PIN_21, Level::Low);
    // let mut reset = Output::new(p.PIN_20, Level::Low);
    // let mut cs = Output::new(p.pin_17, Level::Low);
    // let mut clock = Output::new(p.pin_18, Level::Low);
    // let mut data_in = Output::new(p.PIN_19, Level::Low);

    let bl = p.PIN_22;
    let rst = p.PIN_20;
    let display_cs = p.PIN_9;
    let dcx = p.PIN_21;
    let mosi = p.PIN_19;
    let clk = p.PIN_18;

    // create SPI
    let mut display_config = spi::Config::default();
    display_config.frequency = DISPLAY_FREQ;
    display_config.phase = spi::Phase::CaptureOnSecondTransition;
    display_config.polarity = spi::Polarity::IdleHigh;

    let spi: Spi<'_, _, Blocking> =
        Spi::new_blocking_txonly(p.SPI0, clk, mosi, display_config.clone());
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi));

    let display_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(display_cs, Level::High),
        display_config,
    );
    let dcx = Output::new(dcx, Level::Low);
    let rst = Output::new(rst, Level::Low);
    // dcx: 0 = command, 1 = data

    // Enable LCD backlight
    let _bl = Output::new(bl, Level::High);

    // display interface abstraction from SPI and DC
    let di = SPIInterface::new(display_spi, dcx);

    // Define the display from the display interface and initialize it
    let mut display = Builder::new(GC9A01, di)
        .display_size(240, 240)
        .reset_pin(rst)
        .color_order(ColorOrder::Bgr)
        .invert_colors(ColorInversion::Inverted)
        .init(&mut Delay)
        .unwrap();
    display.clear(Rgb565::BLACK).unwrap();

    let mut circle = Circle::with_center(Point::zero(), 10);

    let mut style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::CSS_BLUE_VIOLET)
        .build();
    let mut a = 0;

    let start = Instant::now();

    loop {
        for (x, y) in (0..10).cartesian_product(0..10) {
            let point = Point::new((x) * 20 + a, (y) * 20 + a);
            circle
                .translate(point)
                .draw_styled(&style, &mut display)
                .unwrap();
        }
        a = if a == 0 { 5 } else { 0 };

        let millis = (Instant::now() - start).as_millis();
        let red = ((((millis as f32).sin() + 1.0) / 2.0) * 255.0) as u8;
        let blue = ((((millis as f32).cos() + 1.0) / 2.0) * 255.0) as u8;
        style.fill_color = Some(Rgb565::new(red, 0, blue));

        display.clear(Rgb565::BLACK).unwrap();
    }
}

// #![no_std]
// #![no_main]

// use embassy_executor::Spawner;
// use embassy_rp::gpio::{Flex, OutputOpenDrain};
// use embassy_rp::peripherals::USB;
// use embassy_rp::usb::Driver;
// use embassy_rp::usb;
// use embassy_rp::{bind_interrupts, gpio};
// use embassy_time::{Instant, Timer};
// use gc9a01::{Gc9a01, SPIDisplayInterface};
// use gpio::{Level, Output};
// use {defmt_rtt as _, panic_probe as _};

// bind_interrupts!(struct Irqs {
//     USBCTRL_IRQ => usb::InterruptHandler<USB>;
// });

// #[embassy_executor::task]
// async fn logger_task(driver: Driver<'static, USB>) {
//     embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
// }

// #[embassy_executor::main]
// async fn main(spawner: Spawner) {
//     let p = embassy_rp::init(Default::default());
//     let driver = Driver::new(p.USB, Irqs);
//     spawner.spawn(logger_task(driver)).unwrap();
//     let mut led = Output::new(p.PIN_25, Level::Low);

//     let mut backlight = Output::new(p.PIN_22, Level::Low);
//     let mut data_command = Output::new(p.PIN_21, Level::Low);
//     let mut reset = Output::new(p.PIN_20, Level::Low);
//     // let mut cs = Output::new(p.pin_17, Level::Low);
//     // let mut clock = Output::new(p.pin_18, Level::Low);
//     // let mut data_in = Output::new(p.PIN_19, Level::Low);

//     // p.SPI0
//     let mut display = SPIDisplayInterface::new(spi.as_ref().get_ref(), data_command);
//     // Gc9a01::new(interface, screen, gc9a01::prelude::DisplayRotation::Rotate0);
//     loop {

//         led.set_high();
//         Timer::after_secs(1).await;

//         led.set_low();
//         Timer::after_secs(1).await;
//     }
// }
