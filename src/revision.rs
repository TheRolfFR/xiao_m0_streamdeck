pub fn prepare_revision_into(output: &mut [u8]) -> usize {
    output[0] = 0x6;
    output[6..11].copy_from_slice("0001\0"[0..5].as_bytes());
    32
}

pub fn prepare_revision() -> [u8; 32] {
    let mut serial_number = [0u8; 32];
    prepare_revision_into(&mut serial_number);
    serial_number
}
