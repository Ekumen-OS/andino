/*!
* Main Andino App
*/

#![no_std]
#![no_main]

use andino_firmware_rust::app::app::App;
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let mut app = App::new(dp);
    app.setup();
    loop {
        app.update();
    }
}
