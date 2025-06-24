#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use cortex_m_rt::exception;
use panic_halt as _;

use hal::{clock::GenericClockController, delay::Delay};
use pac::{interrupt, CorePeripherals, Peripherals, TC3};
use cortex_m::interrupt::free as disable_interrupt;
use usb_device::bus::UsbBusAllocator;
use usbd_hid::hid_class::HIDClass;
use usbd_hid::hid_class::ReportInfo;
use usbd_serial::SerialPort;
use xiao_m0::hal::{fugit::ExtU32, timer::TimerCounter, prelude::InterruptDrivenTimer};
use xiao_m0 as bsp;
use xiao_m0::ehal::delay::DelayNs;
use xiao_m0::ehal::digital::OutputPin;
use xiao_m0::hal::usb::UsbBus;
use xiao_m0::pac::NVIC;
use xiao_m0::Led1;

use bsp::{hal, pac};

use usb_device::prelude::*;
use usbd_hid::descriptor::generator_prelude::*;


mod serial;
use serial::*;

mod revision;
use revision::*;

use crate::report_handler::RequestHandler;

mod report_handler;

#[gen_hid_descriptor(
    (collection = APPLICATION, usage = 0x1, usage_page = VENDOR_DEFINED_START) = {
        (report_id = 0x2, usage=0x2) = { // restart_key_stream
            #[item_settings data,variable,absolute] key=output
        };
        (report_id = 0x3, usage=0x3) = { // reset
            #[item_settings data,variable,absolute] reset=feature
        };
        (report_id = 0x6, usage=0x4) = { // revision
            #[item_settings data,variable,absolute] revision=feature
        };
        (report_id = 0x10, usage=0x5) = { // fake button input
            #[item_settings data,variable,absolute] button=input
        }
    }
)]
struct StreamDeckReport {
    pub key: [u8; 1024],
    pub reset: [u8; 64],
    pub revision: [u8; 32],
    pub button: u8
}

impl Default for StreamDeckReport {
    fn default() -> Self {
        Self {
            key: [0u8; 1024],
            reset: [0u8; 64],
            revision: Default::default(),
            button: Default::default()
        }
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
static mut USB_HID: Option<HIDClass<UsbBus, 64>> = None;

static mut REQ_HANDLER: Mutex<RefCell<StreamDeckHandler>> = Mutex::new(RefCell::new(StreamDeckHandler {}));

static mut TIMER: Option<TimerCounter<TC3>> = None;


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

    let timer_clock = clocks.gclk0();
    let tc_clock = clocks.tcc2_tc3(&timer_clock).unwrap();
    let tc3 = unsafe {
        TIMER = Some(TimerCounter::tc3_(&tc_clock, peripherals.TC3, &mut peripherals.PM));
        TIMER.as_mut().unwrap()
    };
    tc3.start(500.millis());
    tc3.enable_interrupt();

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
        let mut hid_class = HIDClass::new(&bus_allocator, StreamDeckReport::desc(), 100);
        hid_class.get_report_handler = Some(&handle_get_report);
        hid_class.logger = Some(&send_log);
        USB_HID = Some(hid_class);
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
        );

        // Process messages
        enqueue_message!("Rev: {:?}", prepare_revision()).ok();
        while process_next_message() {}

        // Enable USB interrupts
        core.NVIC.set_priority(interrupt::USB, 1);
        NVIC::unmask(interrupt::USB);
    }

    led_pin.set_high().unwrap();
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

        // Process messages
        while process_next_message() {}

        // Wait 100ms before next loop
        delay.delay_ms(100u32);
    }
}

struct StreamDeckHandler;

impl RequestHandler for StreamDeckHandler {
    fn get_report(&mut self, report_info: ReportInfo, data: &mut [u8]) -> Option<usize> {
        enqueue_message!("{:?}", report_info).ok();
        match report_info.report_id {
            6 => Some(prepare_revision_into(data)),
            _ => None
        }
    }
    fn set_report(&mut self, report_info: ReportInfo, data: &[u8]) -> Result<(), ()> {
        enqueue_message!("{report_info:?}").ok();

        let size = report_info.len;
        enqueue_message!("Report buffer ({})={:?}", size, data).ok();

        if size >= 2 && data[0] == 0x3 && data[1] == 0x2 {
            enqueue_message!("Found reset command").ok();
        }
        Ok(())
    }
    fn output(&mut self, report_id: u8, data: &[u8], size: usize) {
        enqueue_message!("Output buffer [#{}]({})={:?}", report_id, size, data).ok();
        if report_id == 0x2 {
            enqueue_message!("Found reset_key_stream").ok();
        }
    }
}

fn handle_get_report(report_info: ReportInfo, data: &mut [u8]) -> Option<usize> {
    disable_interrupt(|cs| unsafe {
        let mut req_handler = REQ_HANDLER.borrow(cs).borrow_mut();
        req_handler.get_report(report_info, data)
    })
}

fn send_log(args: core::fmt::Arguments) -> Result<(), ()> {
    enqueue_formatted(args).map_err(|_| ())
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

            let mut handler = StreamDeckHandler;

            // Read incoming HID data
            let mut buff = [0u8; 1024];
            if let Ok(n) = hid.pull_raw_output(&mut buff) {
                let report_id = if n >= 1 { buff[0] } else { 0 };
                handler.output(report_id, &buff[1..n], n - 1);
            }

            if let Ok(report_info) = hid.pull_raw_report(&mut buff) {
                let n = report_info.len;
                handler.set_report(report_info, &buff[..n]).ok();
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

#[interrupt]
fn TC3() {
    disable_interrupt(|_| unsafe {
        if let Some(timer) = TIMER.as_mut() {
            if timer.wait().is_ok() {
                // send_revision();
            }
        }
    })
}
