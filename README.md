# GDEH0154D67

## Description
Simple SPI driver for the GDEH0154D67 E-Paper display.
This crate is a `no_std` library that provides an interface compatible with [embedded-hal-1.0.0-alpha.8](https://docs.rs/embedded-hal/1.0.0-alpha.8/embedded_hal/).
It is also designed to be used together with [embedded-graphics](https://docs.rs/embedded-graphics/latest/embedded_graphics/).
It ensures a correct initialization and a consistent state at every moment by enforcing design
constrains at compile time using zero cost abstractions. 

## Usage
Simple example adapted from the [embedded-graphics](https://docs.rs/embedded-graphics/latest/embedded_graphics/) main documentation page:
```rust
use gdeh0154d67::GDEH0154D67;

use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    primitives::{
        Circle, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StrokeAlignment, Triangle,
    },
    text::{Alignment, Text},
};

fn main() {
    let peripherals = Peripherals::take().unwrap();

    let spi = peripherals.spi2;
    let sclk = peripherals.pins.gpio18.into_output().unwrap();
    let serial_out = peripherals.pins.gpio23.into_output().unwrap();
    let cs = peripherals.pins.gpio5.into_output().unwrap();
    let reset = peripherals.pins.gpio9.into_output().unwrap();
    let busy = peripherals.pins.gpio19.into_input().unwrap();
    let dc = peripherals.pins.gpio10.into_output().unwrap();
    
    let config = <spi::config::Config as Default>::default().baudrate(20.MHz().into());
    let spi = spi::Master::<spi::SPI2, _, _, _, _>::new(
        spi,
        spi::Pins {
            sclk,
            sdo: serial_out,
            sdi: Option::<GpioPin<InputOutput>>::None,
            cs: Option::<Gpio5<Output>>::Some(cs),
        },
        config,
    ).unwrap();

    let delay = delay::Ets;
    let mut display = GDEH0154D67::new(spi, dc, reset, busy, delay).unwrap().init().unwrap();
    display.full_update().unwrap();

    // Create styles used by the drawing operations.
    let thin_stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let thick_stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 3);
    let border_stroke = PrimitiveStyleBuilder::new()
                            .stroke_color(BinaryColor::On).stroke_width(3)
                            .stroke_alignment(StrokeAlignment::Inside).build();
    let fill = PrimitiveStyle::with_fill(BinaryColor::On);
    let character_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

    let yoffset = 20;

    // Draw a 3px wide outline around the display.
    display.bounding_box().into_styled(border_stroke).draw(&mut display).unwrap();

    // Draw a triangle.
    Triangle::new(
        Point::new(32, 32 + yoffset),
        Point::new(32 + 32, 32 + yoffset),
        Point::new(32 + 16, yoffset),
    ).into_styled(thin_stroke).draw(&mut display).unwrap();
    display.partial_update().unwrap();

    // Draw a filled square
    Rectangle::new(Point::new(84, yoffset), Size::new(32, 32))
        .into_styled(fill).draw(&mut display).unwrap();
    display.partial_update().unwrap();

    let thin_stroke = PrimitiveStyle::with_stroke(BinaryColor::Off, 1);
    // Erase the triangle.
    Triangle::new(
        Point::new(32, 32 + yoffset),
        Point::new(32 + 32, 32 + yoffset),
        Point::new(32 + 16, yoffset),
    ).into_styled(thin_stroke).draw(&mut display).unwrap();

    // Draw a circle with a 3px wide stroke.
    Circle::new(Point::new(32, yoffset), 34)
        .into_styled(thick_stroke).draw(&mut display).unwrap();
    display.partial_update().unwrap();

    // Draw centered text.
    let text = "23:35";
    Text::with_alignment(
        text,
        display.bounding_box().center() + Point::new(0, 0),
        character_style,
        Alignment::Center,
    ).draw(&mut display).unwrap();
    display.partial_update().unwrap();

    (spi, dc, reset, busy) = display.release().unwrap();

    Ok(())
}
```

## Support
Please open an issue if any bug is found or to discuss a new feature.


## Acknowledgment
I would like to acknowledge the embedded-rust community, in particular the author of this [blog post](https://nitschinger.at/Writing-an-embedded-display-driver-in-Rust/) because thanks to it I could start with the right foot.

## License
This project is licensed under the MIT License which you can find in this same repository.

## Project status
This is a project I do in my free time for fun and for learning Rust. I am open to include new features but I only have one ESP board to test the driver with all the connections already wired, this prevents me from including I2C communication for example. Therefore the driver will probably not include all the features of the GDEH0154D67 display unless someone else wants to contribute them. That said, this driver is enough to control the display in an efficient way through SPI protocol.
