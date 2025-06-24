use usbd_hid::hid_class::ReportInfo;


/// Handler for HID-related control requests.
pub trait RequestHandler {
    /// Reads the value of report `id` into `buf` returning the size.
    ///
    /// Returns `None` if `id` is invalid or no data is available.
    fn get_report(&mut self, report_info: ReportInfo, data: &mut [u8]) -> Option<usize> {
        let _ = (report_info, data);
        None
    }

    /// Sets the value of report `id` to `data`.
    fn set_report(&mut self, report_info: ReportInfo, data: &[u8]) -> Result<(), ()> {
        let _ = (report_info, data);
        Err(())
    }

    /// Handle commands send
    fn output(&mut self, report_id: u8, data: &[u8], size: usize) {
        let _ = (report_id, data, size);
    }
}
