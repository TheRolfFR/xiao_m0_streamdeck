#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use hal::{clock::GenericClockController};
use pac::{Peripherals};
use usbd_hid::hid_class::HIDClass;
use xiao_m0::hal::prelude::_atsamd_hal_embedded_hal_digital_v2_ToggleableOutputPin;
use xiao_m0::Led1;
use xiao_m0 as bsp;

use bsp::{hal, pac};

use usb_device::prelude::*;
use usbd_hid::descriptor::gen_hid_descriptor;
use usbd_hid::descriptor::generator_prelude::*;

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = KEYBOARD) = {
        (usage = 0x00, report_id=0x2,) = {
            #[item_settings data, array, absolute] payload=output // Output (Host to Device)
        };
    }
)]
pub struct StreamDeckReport {
    pub payload: [u8; 1024],
}

impl Default for StreamDeckReport {
    fn default() -> Self {
        Self {
            payload: [0u8; 1024],
        }
    }
}

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();

    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );

    let pins = bsp::pins::Pins::new(peripherals.PORT);

    let mut led_pin: Led1 = pins.led1.into_mode();

    let bus_allocator = bsp::usb_allocator(
        peripherals.USB,
        &mut clocks,
        &mut peripherals.PM,
        pins.usb_dm,
        pins.usb_dp,
    );

    let mut hid = HIDClass::new(&bus_allocator, StreamDeckReport::desc(), 60);

    let mut usb_dev = UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x0fd9, 0x0084))
        .device_class(0x00)
        .device_protocol(0x00)
        .device_sub_class(0x00)
        .strings(&[StringDescriptors::default()
            .manufacturer("Elgato")
            .product("Stream Deck Plus")
            .serial_number("0001")
        ]).unwrap()
        .build();

        let mut buf = [0u8; 1024];

        loop {
            if usb_dev.poll(&mut [&mut hid]) {
                if let Ok(count) = hid.pull_raw_output(&mut buf) {
                    led_pin.toggle().ok();
                    if count > 0 && buf[0] == 0x02 {

                        // Send the response back to the host
                        let mut report = StreamDeckReport::default();
                        report.payload[0] = count as u8;
                    }
                }
            }
        }
}
