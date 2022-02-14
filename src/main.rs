#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m::delay::Delay;
use cortex_m_rt::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;
use rp_pico::{
    // A shorter alias for the Hardware Abstraction Layer, which provides
    // higher-level drivers.
    hal,
    hal::gpio::bank0::Gpio25,
    hal::gpio::Pin,
    hal::gpio::PushPullOutput,
    // A shorter alias for the Peripheral Access Crate, which provides low-level
    // register access
    hal::pac,
    hal::Clock,
};
// USB Device support
use usb_device::{class_prelude::*, prelude::*};
// USB Communications Class Device support
use usbd_serial::SerialPort;
// Time handling traits
use embedded_time::rate::*;
// I2C HAL traits
use bno055::{BNO055OperationMode, Bno055};
use embedded_hal::digital::v2::OutputPin;

/// A debug function that blinks the onboard LED on and off again,
/// separated by the provided wait time in milliseconds
fn blink_onboard(led_pin: &mut Pin<Gpio25, PushPullOutput>, delay: &mut Delay, wait_time_ms: u32) {
    led_pin.set_high().unwrap();
    delay.delay_ms(wait_time_ms);
    led_pin.set_low().unwrap();
    delay.delay_ms(wait_time_ms);
}

#[entry]
fn main() -> ! {
    // Singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    // Set up the watchdog driver
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    // The delay object lets us wait a certain time in ms
    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().integer());
    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    // Use serial over USB
    let mut serial = SerialPort::new(&usb_bus);
    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("PI_PICO")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);
    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // Configure two pins as being I2C, not GPIO
    let sda_pin = pins.gpio2.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio3.into_mode::<hal::gpio::FunctionI2C>();
    // Set the pins up to have an I2C bus
    let i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock,
    );
    let mut onboard_led_pin = pins.led.into_push_pull_output();

    let mut imu = Bno055::new(i2c).with_alternative_address();
    // Let the BNO chip warm up
    delay.delay_ms(500);
    // Now let's configure it a bit
    imu.set_mode(BNO055OperationMode::NDOF, &mut delay)
        .expect("An error occurred while setting to NDOF mode");

    // Uncomment the below iff you can consistently and successfully
    // calibrate the BNO chip. At the time of this writing I cannot so I'll
    // deal with its not being calibrated all the time
    // while !imu.is_fully_calibrated().unwrap() {
    //     // delay.delay_ms(500);
    // }
    // let calib = imu.calibration_profile(&mut delay).unwrap();
    // imu.set_calibration_profile(calib, &mut delay).unwrap();
    blink_onboard(&mut onboard_led_pin, &mut delay, 100);
    loop {
        // Only do an meaningful tick iff
        // the serial buffer is ready for read or write
        //
        // If poll() isn't invoked after at most 10ms, then
        // this program isn't USB compliant and it won't be
        // recognized to the USB host
        if usb_dev.poll(&mut [&mut serial]) {
            match imu.quaternion() {
                Ok(quaternion) => {
                    let w = quaternion.s.to_le_bytes();
                    let x = quaternion.v.x.to_le_bytes();
                    let y = quaternion.v.y.to_le_bytes();
                    let z = quaternion.v.z.to_le_bytes();
                    let quat_bytes: [u8; 16] = [
                        w[0], w[1], w[2], w[3], x[0], x[1], x[2], x[3], y[0], y[1], y[2], y[3],
                        z[0], z[1], z[2], z[3],
                    ];
                    // Let's be optimistic about a failure in this tick
                    // and hope (lol) that it works on the next tick
                    let _ = serial.write(&quat_bytes);
                }
                Err(_) => {
                    blink_onboard(&mut onboard_led_pin, &mut delay, 100);
                }
            }
        }
    }
}
