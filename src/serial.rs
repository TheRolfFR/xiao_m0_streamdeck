use cortex_m::interrupt::Mutex;
use usbd_serial::SerialPort;
use xiao_m0::hal::usb::UsbBus;
use core::cell::RefCell;
use heapless::{String, Deque};
use core::fmt::Write;


// Configuration
const MAX_MESSAGE_LEN: usize = 128;
const QUEUE_SIZE: usize = 16;

// String message type
pub type Message = String<MAX_MESSAGE_LEN>;


static MESSAGE_QUEUE: Mutex<RefCell<Deque<Message, QUEUE_SIZE>>> =
    Mutex::new(RefCell::new(Deque::new()));


pub fn enqueue_formatted(args: core::fmt::Arguments) -> Result<(), &'static str> {
    let mut message = String::new();

    // Format the message
    match core::fmt::write(&mut message, args) {
        Ok(()) => {
            cortex_m::interrupt::free(|cs| {
                let queue = MESSAGE_QUEUE.borrow(cs);
                let mut queue_ref = queue.borrow_mut();

                match queue_ref.push_back(message) {
                    Ok(()) => Ok(()),
                    Err(_) => Err("Queue full"),
                }
            })
        }
        Err(_) => Err("Format error"),
    }
}

// Macro for easy formatted enqueueing
#[macro_export]
macro_rules! enqueue_message {
    ($($arg:tt)*) => {
        enqueue_formatted(format_args!($($arg)*)).map_err(|_| ())
    };
}

// Dequeue functions
pub fn dequeue_message() -> Option<Message> {
    cortex_m::interrupt::free(|cs| {
        let queue = MESSAGE_QUEUE.borrow(cs);
        let mut queue_ref = queue.borrow_mut();
        queue_ref.pop_front()
    })
}
// Process messages
pub fn process_next_message() -> bool {
    if let Some(message) = dequeue_message() {
        // Unwrap and process the string message
        handle_message(message.as_str());
        true
    } else {
        false
    }
}


// Message handler (implement based on your needs)
fn handle_message(msg: &str) {
    unsafe {
        write!(LOG_WRITER, "{msg}\r\n").ok();
    }
}

/// Empty type to implement write over USB serial
pub struct UsbSerialWriter;

pub static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
/// Log writer object provides core::fmt::Write over the USB_SERIAL object
pub static mut LOG_WRITER: UsbSerialWriter = UsbSerialWriter;

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
