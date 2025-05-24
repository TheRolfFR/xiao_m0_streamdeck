#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use core::fmt::Write;

use cortex_m_rt::entry;
use cortex_m_rt::exception;
use panic_halt as _;

use hal::{clock::GenericClockController, delay::Delay};
use pac::{interrupt, CorePeripherals, Peripherals};
use cortex_m::interrupt::free as disable_interrupt;
use usb_device::bus::UsbBusAllocator;
use usbd_hid::hid_class::HIDClass;
use usbd_hid::hid_class::ReportInfo;
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

// HID Report Descriptor for custom device that can receive 1024 bytes + feature reports
const HID_REPORT_DESCRIPTOR: &[u8] = &[
    0x06, 0x00, 0xFF,    // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01,          // Usage (0x01)
    0xA1, 0x01,          // Collection (Application)

    // Output Report (Host to Device) - 1024 bytes with Report ID 0x02
    0x85, 0x02,          //   Report ID (2)
    0x09, 0x01,          //   Usage (0x01)
    0x15, 0x00,          //   Logical Minimum (0)
    0x25, 0xFF,          //   Logical Maximum (255)
    0x75, 0x08,          //   Report Size (8 bits)
    0x96, 0x00, 0x04,    //   Report Count (1024)
    0x91, 0x02,          //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

    // Feature Report - Bidirectional with Report ID 0x03
    0x85, 0x03,          //   Report ID (3)
    0x09, 0x02,          //   Usage (0x02)
    0x15, 0x00,          //   Logical Minimum (0)
    0x25, 0xFF,          //   Logical Maximum (255)
    0x75, 0x08,          //   Report Size (8 bits)
    0x95, 0x40,          //   Report Count (64) - 64 byte feature report
    0xB1, 0x02,          //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

    // Read-only Feature Report with Report ID 0x06 (32 bytes)
    0x85, 0x06,          //   Report ID (6)
    0x09, 0x03,          //   Usage (0x03)
    0x15, 0x00,          //   Logical Minimum (0)
    0x25, 0xFF,          //   Logical Maximum (255)
    0x75, 0x08,          //   Report Size (8 bits)
    0x95, 0x20,          //   Report Count (32) - 32 byte feature report
    0xB1, 0x02,          //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

    0xC0,                // End Collection
];

struct StreamDeckReport;

#[gen_hid_descriptor(
    (report_id = 0x6, usage=0x3) = {
        #[item_settings_data,variable,absolute] buf=feature
    }
)]
struct RevisionReport {
    pub buf: [u8; 32]
}

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

    led_pin.set_high().unwrap();

    let mut serial_number = [0u8; 33];
    serial_number[0] = 0x6;
    serial_number[6..10].copy_from_slice("0001"[0..4].as_bytes());
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

        disable_interrupt(|_| unsafe {
            // write serial number (0001)
            USB_HID.as_mut().unwrap().push_raw_input(&serial_number).ok();
        });

        // Wait 100ms before next loop
        delay.delay_ms(100u32);
    }
}

fn handle_command(cmd: &[u8], size: usize) {
    unsafe {
        write!(LOG_WRITER, "Output buffer ({})={:?}\r\n", size, cmd).ok();
        if size >= 1 && cmd[0] == 0x2 {
            write!(LOG_WRITER, "Found reset_key_stream\r\n").ok();
        }
    }
}

fn handle_report(cmd: &[u8], size: usize, report_info: ReportInfo) {
    unsafe {
        write!(LOG_WRITER, "{report_info:?}").ok();
        write!(LOG_WRITER, "Report buffer ({})={:?}\r\n", size, cmd).ok();
        if size >= 2 && cmd[0] == 0x3 && cmd[1] == 0x2 {
            write!(LOG_WRITER, "Found reset command\r\n").ok();
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

            if let Ok(report_info) = hid.pull_raw_report(&mut buff) {
                let n = report_info.len;
                handle_report(&buff[..n], n, report_info);
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
