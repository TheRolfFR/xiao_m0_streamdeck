#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use core::fmt::Write;

use cortex_m_rt::entry;
use cortex_m_rt::exception;
use panic_halt as _;

use hal::{clock::GenericClockController, delay::Delay};
use pac::{interrupt, CorePeripherals, Peripherals};
use usb_device::bus::UsbBusAllocator;
use usbd_hid::hid_class::HIDClass;
use usbd_serial::SerialPort;
use xiao_m0 as bsp;
use xiao_m0::ehal::delay::DelayNs;
use xiao_m0::ehal::digital::OutputPin;
use xiao_m0::hal::usb::UsbBus;
use xiao_m0::pac::NVIC;
use xiao_m0::Led1;

use bsp::{hal, pac};

use usb_device::prelude::*;
use usbd_hid::descriptor::generator_prelude::*;

// HID Report Descriptor for custom device that can receive 1024 bytes
const HID_REPORT_DESCRIPTOR: &[u8] = &[
    0x06, 0x00, 0xFF, // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01, // Usage (0x01)
    0xA1, 0x01, // Collection (Application)
    0x85, 0x02, //   Report ID (2)
    0x09, 0x01, //   Usage (0x01)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0xFF, //   Logical Maximum (255)
    0x75, 0x08, //   Report Size (8 bits)
    0x96, 0x00, 0x04, //   Report Count (1024)
    0x91,
    0x02, //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0, // End Collection
];

struct StreamDeckReport;

impl SerializedDescriptor for StreamDeckReport {
    fn desc() -> &'static [u8] {
        HID_REPORT_DESCRIPTOR
    }
}

/// Global sys-tick counter value
static mut SYSTICK_COUNT: usize = 0;

/// SysTick interrupt handler
#[exception]
fn SysTick() {
    unsafe {
        SYSTICK_COUNT = SYSTICK_COUNT.wrapping_add(1);
    }
}

static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_HID: Option<HIDClass<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;

/// Empty type to implement write over USB serial
struct UsbSerialWriter;

/// Log writer object provides core::fmt::Write over the USB_SERIAL object
static mut LOG_WRITER: UsbSerialWriter = UsbSerialWriter;

impl core::fmt::Write for UsbSerialWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        if s.len() == 0 {
            return Ok(());
        }

        cortex_m::interrupt::free(|_| unsafe {
            let serial = USB_SERIAL.as_mut().unwrap();

            let _r = serial.write(s.as_bytes());

            Ok(())
        })
    }
}

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();

    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );

    let pins = bsp::pins::Pins::new(peripherals.PORT);
    let mut delay = Delay::new(core.SYST, &mut clocks);

    let mut led_pin: Led1 = pins.led1.into_mode();

    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(bsp::usb_allocator(
            peripherals.USB,
            &mut clocks,
            &mut peripherals.PM,
            pins.usb_dm,
            pins.usb_dp,
        ));
        USB_ALLOCATOR.as_ref().unwrap()
    };

    unsafe {
        USB_HID = Some(HIDClass::new(&bus_allocator, StreamDeckReport::desc(), 100));
        USB_SERIAL = Some(SerialPort::new(&bus_allocator));
        USB_BUS = Some(
            UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x0fd9, 0x0084))
                .composite_with_iads()
                .strings(&[StringDescriptors::default()
                    .manufacturer("Elgato")
                    .product("Stream Deck Plus")
                    .serial_number("0001")])
                .unwrap()
                .max_packet_size_0(64)
                .unwrap()
                .build(),
        )
    }

    // Enable USB interrupts
    unsafe {
        core.NVIC.set_priority(interrupt::USB, 1);
        NVIC::unmask(interrupt::USB);
    }

    let mut i: u32 = 0;

    loop {
        // Flash LED
        if (i % 10) < 5 {
            led_pin.set_high().unwrap();
        } else {
            led_pin.set_low().unwrap();
        }

        // Increment loop counter
        i = i.wrapping_add(1);

        // Wait 100ms before next loop
        delay.delay_ms(100u32);
    }
}

fn handle_command(cmd: &[u8], size: usize) {
    unsafe {
        write!(LOG_WRITER, "Buffer ({})={:?}\r\n", size, cmd).ok();
        if cmd[0] == 0x2 {
            write!(LOG_WRITER, "Found reset_key_stream\r\n").ok();
        }
    }
}

fn poll_usb() {
    unsafe {
        USB_BUS.as_mut().map(|usb_dev| {
            // Fetch endpoints
            let hid = USB_HID.as_mut().unwrap();
            let serial = USB_SERIAL.as_mut().unwrap();

            // Poll on device (auto-updates HID as required)
            usb_dev.poll(&mut [hid, serial]);
            // usb_dev.poll(&mut [hid]);
            // usb_dev.poll(&mut [serial]);

            // Read incoming HID data
            let mut buff = [0u8; 1024];
            if let Ok(n) = hid.pull_raw_output(&mut buff) {
                handle_command(&buff[..n], n);
            }

            // Read incoming serial data
            if let Ok(n) = serial.read(&mut buff) {
                // Loopback
                serial.write(&buff[..n]).unwrap();
                if buff[..n].contains(&('\r' as u8)) || buff[..n].contains(&('\n' as u8)) {
                    serial.write("\n".as_bytes()).ok();
                }

                // TODO: something with this
                //...
            }
        });
    }
}

// Handle USB interrupts
#[interrupt]
fn USB() {
    poll_usb();
}
