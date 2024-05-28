use crate::setup::SetupPacket;

/// Interface events generated by the USB interface's interrupt handler.
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum UsbEvent {
    /// Received a USB bus reset
    BusReset = 10,

    /// Received a setup packet on `USBx_EP_CONTROL`
    ///
    /// Contents is (`endpoint_number`)
    ReceiveControl(u8) = 11,

    /// Received a data packet on `USBx_EP_OUT`
    ///
    /// Contents is (`endpoint_number`)
    ReceivePacket(u8) = 12,

    /// Send is complete on `USBx_EP_IN`
    ///
    /// Contents is (`endpoint_number`)
    SendComplete(u8) = 13,

    /// Received a setup packet on `USBx_EP_CONTROL`
    ///
    /// An alternate version of `ReceiveControl` that can be used
    /// when the setup packet is read inside the interrupt handler
    /// for lower latency.
    ///
    /// Contents is (`endpoint_number`, `setup_packet`)
    ReceiveSetupPacket(u8, SetupPacket) = 201,

    #[cfg(feature = "chonky_events")]
    /// Received a data packet on USBx_EP_OUT
    ///
    /// An alternate version of `ReceivePacket` that can be used
    /// when the packet is read inside the interrupt handler
    /// for lower latency.
    ///
    /// Contents is (`endpoint_number`, `bytes_read`, `packet_buffer`)
    ReceiveBuffer(u8, usize, [u8; crate::EP_MAX_PACKET_SIZE]) = 202,
}

impl core::fmt::Debug for UsbEvent {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            UsbEvent::BusReset => {
                write!(f, "BusReset")
            }
            UsbEvent::ReceiveControl(endpoint) => {
                write!(f, "ReceiveControl({endpoint})")
            }
            UsbEvent::ReceivePacket(endpoint) => {
                write!(f, "ReceivePacket({endpoint})")
            }
            UsbEvent::SendComplete(endpoint) => {
                write!(f, "SendComplete({endpoint})")
            }
            UsbEvent::ReceiveSetupPacket(endpoint, setup_packet) => {
                write!(f, "ReceiveSetupPacket({endpoint}, {setup_packet:?})")
            }
            #[cfg(feature = "chonky_events")]
            UsbEvent::ReceiveBuffer(endpoint, bytes_read, _buffer) => {
                write!(f, "ReceiveBuffer({}, {})", endpoint, bytes_read)
            }
        }
    }
}

impl From<UsbEvent> for u8 {
    fn from(event: UsbEvent) -> u8 {
        match event {
            UsbEvent::BusReset => 10,
            UsbEvent::ReceiveControl(_) => 11,
            UsbEvent::ReceivePacket(_) => 12,
            UsbEvent::SendComplete(_) => 13,
            UsbEvent::ReceiveSetupPacket(_, _) => 201,
            #[cfg(feature = "chonky_events")]
            UsbEvent::ReceiveBuffer(_, _, _) => 202,
        }
    }
}

impl core::convert::From<UsbEvent> for [u8; 2] {
    fn from(event: UsbEvent) -> Self {
        use UsbEvent::*;
        match event {
            BusReset => [event.into(), 0],
            ReceiveControl(endpoint_number) => [event.into(), endpoint_number],
            ReceiveSetupPacket(endpoint_number, _setup_packet) => [event.into(), endpoint_number],
            ReceivePacket(endpoint_number) => [event.into(), endpoint_number],
            #[cfg(feature = "chonky_events")]
            ReceiveBuffer(endpoint_number, _, _) => [event.into(), endpoint_number],
            SendComplete(endpoint_number) => [event.into(), endpoint_number],
        }
    }
}

impl UsbEvent {
    #[must_use]
    pub fn into_bytes(self) -> [u8; 2] {
        self.into()
    }
}
