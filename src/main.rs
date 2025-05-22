#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use hal::{clock::GenericClockController};
use pac::{Peripherals};
use usb_device::bus::InterfaceNumber;
use usb_device::bus::UsbBusAllocator;
use usb_device::class::UsbClass;
use usb_device::descriptor::DescriptorWriter;
use usb_device::endpoint::EndpointIn;
use usb_device::endpoint::EndpointOut;
use usbd_hid::hid_class::HIDClass;
use xiao_m0 as bsp;

use bsp::{hal, pac};

use usb_device::prelude::*;
use usbd_hid::descriptor::gen_hid_descriptor;
use usbd_hid::descriptor::generator_prelude::*;

pub struct StreamDeckClass<'a, B: usb_device::bus::UsbBus> {
    interface: InterfaceNumber,
    write_ep: EndpointIn<'a, B>,
    read_ep: EndpointOut<'a, B>,
}

impl<B: usb_device::bus::UsbBus> StreamDeckClass<'_, B> {
    pub fn new(alloc: &UsbBusAllocator<B>) -> StreamDeckClass<B> {
        StreamDeckClass {
            interface: alloc.interface(),
            write_ep: alloc.bulk(64),
            read_ep: alloc.bulk(64),
        }
    }

    pub fn poll(&mut self) {
        let mut buf = [0u8; 64];
        if let Ok(count) = self.read_ep.read(&mut buf) {
            // Echo back the received bytes (or handle Stream Deck protocol)
            let _ = self.write_ep.write(&buf[0..count]);
        }
    }
}

impl<B: usb_device::bus::UsbBus> UsbClass<B> for StreamDeckClass<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> usb_device::Result<()> {
        writer.interface(
            self.interface,
            0xFF, // Vendor-specific class
            0x00, // Subclass
            0x00, // Protocol
        )?;
        writer.endpoint(&self.read_ep)?;
        writer.endpoint(&self.write_ep)?;
        Ok(())
    }
}

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = KEYBOARD) = {
        (usage_page = KEYBOARD, usage_min = 0x00, usage_max = 0xFF) = {
            #[item_settings data, variable, absolute] inside=input
        };
        (usage_page = KEYBOARD, usage_min = 0x00, usage_max = 0xFF) = {
            #[item_settings data, variable, absolute] outside=output
        };
    }
)]
pub struct StreamDeckReport {
    pub inside: u8,
    pub outside: u8
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

    let bus_allocator = bsp::usb_allocator(
        peripherals.USB,
        &mut clocks,
        &mut peripherals.PM,
        pins.usb_dm,
        pins.usb_dp,
    );

    let mut streamdeck = StreamDeckClass::new(&bus_allocator);

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

        loop {
            if usb_dev.poll(&mut [&mut streamdeck, &mut hid]) {
                streamdeck.poll();
            }
        }
}
