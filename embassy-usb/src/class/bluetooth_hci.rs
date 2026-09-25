//! Bluetooth HCI USB transport class.
//!
//! Implements the USB transport for a Bluetooth HCI controller, as described in the
//! Bluetooth Core Specification, Vol 4, Part B ("Host Controller Interface [Transport
//! Layer]", USB Transport Layer). This is the protocol implemented by the Linux `btusb`
//! driver, so exposing this class makes the device usable as a Bluetooth dongle on
//! Linux with no additional drivers.
//!
//! The class consists of:
//!
//! - The default control endpoint (EP0), carrying HCI commands as class-specific
//!   control OUT transfers (bRequest = 0).
//! - An interrupt IN endpoint carrying HCI events.
//! - A bulk OUT endpoint and a bulk IN endpoint carrying HCI ACL data.
//!
//! SCO (isochronous) data is not supported; `btusb` treats the isochronous interface
//! as optional.
//!
//! For the generic `btusb` device match (`USB_DEVICE_INFO(0xe0, 0x01, 0x01)`), set
//! [`USB_CLASS_WIRELESS_CONTROLLER`], [`USB_SUBCLASS_RF`] and
//! [`USB_PROTOCOL_BLUETOOTH`] as `device_class`, `device_sub_class` and
//! `device_protocol` in the [`Config`](crate::Config) used to build the
//! [`UsbDevice`](crate::UsbDevice). `btusb` also matches on the interface class, so
//! this is not strictly required, but it is recommended.

use core::cell::RefCell;
use core::mem::MaybeUninit;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use crate::control::{OutResponse, Recipient, Request, RequestType};
use crate::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};
use crate::types::InterfaceNumber;
use crate::{Builder, Handler};

/// Value to use as `device_class` in the device `Config` for Bluetooth HCI devices.
pub const USB_CLASS_WIRELESS_CONTROLLER: u8 = 0xE0;
/// Value to use as `device_sub_class` in the device `Config` for Bluetooth HCI devices.
pub const USB_SUBCLASS_RF: u8 = 0x01;
/// Value to use as `device_protocol` in the device `Config` for Bluetooth HCI devices.
pub const USB_PROTOCOL_BLUETOOTH: u8 = 0x01;

/// HCI command: bRequest = 0, class-specific control OUT transfer.
const REQ_HCI_COMMAND: u8 = 0x00;

/// Maximum length of an HCI command packet: 3-byte header + up to 255 bytes of parameters.
pub const HCI_COMMAND_MAX_LEN: usize = 3 + 255;
/// Maximum length of an HCI event packet: 2-byte header + up to 255 bytes of parameters.
pub const HCI_EVENT_MAX_LEN: usize = 2 + 255;
/// Maximum length of an HCI ACL packet: 4-byte header + up to 1021 bytes of payload.
///
/// This covers typical controllers; consult your controller's HCI data buffer size
/// if you need more.
pub const HCI_ACL_MAX_LEN: usize = 4 + 1021;

/// Bluetooth HCI class error.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// USB is disconnected and the endpoint is disabled.
    Disabled,
    /// The supplied buffer is too small for the transfer.
    BufferOverflow,
}

impl From<EndpointError> for Error {
    fn from(e: EndpointError) -> Self {
        match e {
            EndpointError::Disabled => Error::Disabled,
            EndpointError::BufferOverflow => Error::BufferOverflow,
        }
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Disabled => f.write_str("Disabled"),
            Self::BufferOverflow => f.write_str("BufferOverflow"),
        }
    }
}

impl core::error::Error for Error {}

/// State of the Bluetooth HCI class.
pub struct State<'a> {
    control: MaybeUninit<Control<'a>>,
    shared: ControlShared,
}

impl<'a> Default for State<'a> {
    fn default() -> Self {
        Self::new()
    }
}

impl<'a> State<'a> {
    /// Create a new `State`.
    pub const fn new() -> Self {
        Self {
            control: MaybeUninit::uninit(),
            shared: ControlShared::new(),
        }
    }
}

struct HciCommand {
    len: u16,
    data: [u8; HCI_COMMAND_MAX_LEN],
}

/// Shared data between Control and BluetoothHciClass.
struct ControlShared {
    commands: Channel<CriticalSectionRawMutex, HciCommand, 2>,
}

impl ControlShared {
    const fn new() -> Self {
        Self {
            commands: Channel::new(),
        }
    }
}

struct Control<'a> {
    if_num: InterfaceNumber,
    shared: &'a ControlShared,
}

impl<'a> Handler for Control<'a> {
    fn reset(&mut self) {
        // Drain pending commands.
        while self.shared.commands.try_receive().is_ok() {}
    }

    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        // HCI commands arrive as class-specific control OUT transfers with bRequest = 0.
        // The Bluetooth spec sends them to the device (bmRequestType = 0x20, wIndex = 0);
        // also accept requests addressed to our interface.
        if req.request_type != RequestType::Class || req.request != REQ_HCI_COMMAND {
            return None;
        }
        match (req.recipient, req.index) {
            (Recipient::Device, 0) => {}
            (Recipient::Interface, i) if i == self.if_num.0 as u16 => {}
            _ => return None,
        }

        // Validate the HCI command header: opcode (2 bytes) + param length (1 byte).
        if data.len() < 3 || data.len() > HCI_COMMAND_MAX_LEN || data.len() != 3 + data[2] as usize {
            warn!("bluetooth hci: rejecting malformed command, len {}", data.len());
            return Some(OutResponse::Rejected);
        }

        let mut cmd = HciCommand {
            len: data.len() as u16,
            data: [0; HCI_COMMAND_MAX_LEN],
        };
        cmd.data[..data.len()].copy_from_slice(data);
        match self.shared.commands.try_send(cmd) {
            Ok(()) => Some(OutResponse::Accepted),
            Err(_) => {
                warn!("bluetooth hci: command queue full, dropping command");
                Some(OutResponse::Rejected)
            }
        }
    }
}

/// Bluetooth HCI USB transport class.
///
/// See the [module-level documentation](self) for details.
pub struct BluetoothHciClass<'d, D: Driver<'d>> {
    event_ep: D::EndpointIn,
    acl_in_ep: D::EndpointIn,
    acl_out_ep: D::EndpointOut,
    shared: &'d ControlShared,
}

impl<'d, D: Driver<'d>> BluetoothHciClass<'d, D> {
    /// Creates a new `BluetoothHciClass`.
    ///
    /// `max_packet_size` is the size of the bulk ACL endpoints in bytes. For
    /// full-speed devices it has to be one of 8, 16, 32 or 64.
    ///
    /// The device's control buffer (`control_buf` in [`Builder::new`]) must be at
    /// least [`HCI_COMMAND_MAX_LEN`] bytes long, since HCI commands are delivered
    /// via the control endpoint.
    pub fn new(builder: &mut Builder<'d, D>, state: &'d mut State<'d>, max_packet_size: u16) -> Self {
        assert!(builder.control_buf_len() >= HCI_COMMAND_MAX_LEN);

        let mut func = builder.function(USB_CLASS_WIRELESS_CONTROLLER, USB_SUBCLASS_RF, USB_PROTOCOL_BLUETOOTH);
        let mut iface = func.interface();
        let if_num = iface.interface_number();
        let mut alt = iface.alt_setting(
            USB_CLASS_WIRELESS_CONTROLLER,
            USB_SUBCLASS_RF,
            USB_PROTOCOL_BLUETOOTH,
            None,
        );

        // HCI events.
        let event_ep = alt.endpoint_interrupt_in(None, 64, 1);
        // HCI ACL data.
        let acl_in_ep = alt.endpoint_bulk_in(None, max_packet_size);
        let acl_out_ep = alt.endpoint_bulk_out(None, max_packet_size);

        drop(func);

        let control = state.control.write(Control {
            if_num,
            shared: &state.shared,
        });
        builder.handler(control);

        BluetoothHciClass {
            event_ep,
            acl_in_ep,
            acl_out_ep,
            shared: &state.shared,
        }
    }

    /// Waits for the USB host to enable this interface.
    pub async fn wait_connection(&mut self) {
        self.acl_out_ep.wait_enabled().await;
    }

    /// Reads one HCI command received over the control endpoint.
    ///
    /// Returns the command length, or [`Error::BufferOverflow`] if `buf` is too
    /// small to hold the command.
    pub async fn read_command(&self, buf: &mut [u8]) -> Result<usize, Error> {
        let cmd = self.shared.commands.receive().await;
        let len = cmd.len as usize;
        if buf.len() < len {
            return Err(Error::BufferOverflow);
        }
        buf[..len].copy_from_slice(&cmd.data[..len]);
        Ok(len)
    }

    /// Reads one HCI ACL packet from the bulk OUT endpoint.
    ///
    /// ACL packets can be split over multiple USB packets by the host; this
    /// reassembles a whole ACL packet. `buf` must be large enough to hold a
    /// whole ACL packet (see [`HCI_ACL_MAX_LEN`]).
    pub async fn read_acl(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let mps = self.acl_out_ep.info().max_packet_size as usize;
        let mut total = 0;
        loop {
            if buf.len() - total < mps {
                return Err(Error::BufferOverflow);
            }
            let n = self.acl_out_ep.read(&mut buf[total..]).await?;
            // The host may terminate transfers with zero-length packets; skip
            // them while we haven't received any data yet.
            if total == 0 && n == 0 {
                continue;
            }
            total += n;
            if total >= 4 {
                let expected = 4 + u16::from_le_bytes(buf[2..4].try_into().unwrap()) as usize;
                if total > expected {
                    warn!(
                        "bluetooth hci: received more ACL data than expected ({} > {})",
                        total, expected
                    );
                }
                if total >= expected {
                    return Ok(total);
                }
            }
        }
    }

    /// Writes one HCI event to the interrupt IN endpoint.
    ///
    /// The event is split into `max_packet_size`-sized USB packets; if it ends on
    /// a packet boundary a zero-length packet is sent to terminate the transfer.
    pub async fn write_event(&mut self, event: &[u8]) -> Result<(), Error> {
        write_chunked(&mut self.event_ep, event).await
    }

    /// Writes one HCI ACL packet to the bulk IN endpoint.
    ///
    /// The packet is split into `max_packet_size`-sized USB packets; if it ends
    /// on a packet boundary a zero-length packet is sent to terminate the
    /// transfer, since the host driver treats a short packet as end of transfer.
    pub async fn write_acl(&mut self, acl: &[u8]) -> Result<(), Error> {
        write_chunked(&mut self.acl_in_ep, acl).await
    }

    /// Split the class into a sender and a receiver.
    ///
    /// This allows concurrently sending (events and ACL data to the host) and
    /// receiving (commands and ACL data from the host) from separate tasks.
    pub fn split(self) -> (Sender<'d, D>, Receiver<'d, D>) {
        (
            Sender {
                event_ep: self.event_ep,
                acl_in_ep: self.acl_in_ep,
            },
            Receiver {
                acl_out_ep: RefCell::new(self.acl_out_ep),
                shared: self.shared,
            },
        )
    }
}

async fn write_chunked<E: EndpointIn>(ep: &mut E, data: &[u8]) -> Result<(), Error> {
    // `write_transfer` splits the data into `max_packet_size`-sized packets and
    // sends a terminating zero-length packet if the length is a multiple of
    // `max_packet_size`, which the host driver requires to complete a transfer.
    ep.write_transfer(data, true).await?;
    Ok(())
}

/// Bluetooth HCI packet sender (device to host: events and ACL data).
///
/// You can obtain a `Sender` with [`BluetoothHciClass::split`].
pub struct Sender<'d, D: Driver<'d>> {
    event_ep: D::EndpointIn,
    acl_in_ep: D::EndpointIn,
}

impl<'d, D: Driver<'d>> Sender<'d, D> {
    /// Waits for the USB host to enable this interface.
    pub async fn wait_connection(&mut self) {
        self.acl_in_ep.wait_enabled().await;
    }

    /// Writes one HCI event to the interrupt IN endpoint.
    ///
    /// The event is split into `max_packet_size`-sized USB packets; if it ends on
    /// a packet boundary a zero-length packet is sent to terminate the transfer.
    pub async fn write_event(&mut self, event: &[u8]) -> Result<(), Error> {
        write_chunked(&mut self.event_ep, event).await
    }

    /// Writes one HCI ACL packet to the bulk IN endpoint.
    ///
    /// The packet is split into `max_packet_size`-sized USB packets; if it ends
    /// on a packet boundary a zero-length packet is sent to terminate the
    /// transfer, since the host driver treats a short packet as end of transfer.
    pub async fn write_acl(&mut self, acl: &[u8]) -> Result<(), Error> {
        write_chunked(&mut self.acl_in_ep, acl).await
    }
}

/// Bluetooth HCI packet receiver (host to device: commands and ACL data).
///
/// `read_command` and `read_acl` take `&self`, so they can be awaited
/// concurrently (for example with [`embassy_futures::select`]).
///
/// You can obtain a `Receiver` with [`BluetoothHciClass::split`].
pub struct Receiver<'d, D: Driver<'d>> {
    acl_out_ep: RefCell<D::EndpointOut>,
    shared: &'d ControlShared,
}

impl<'d, D: Driver<'d>> Receiver<'d, D> {
    /// Waits for the USB host to enable this interface.
    pub async fn wait_connection(&mut self) {
        self.acl_out_ep.get_mut().wait_enabled().await;
    }

    /// Reads one HCI command received over the control endpoint.
    ///
    /// Returns the command length, or [`Error::BufferOverflow`] if `buf` is too
    /// small to hold the command.
    pub async fn read_command(&self, buf: &mut [u8]) -> Result<usize, Error> {
        let cmd = self.shared.commands.receive().await;
        let len = cmd.len as usize;
        if buf.len() < len {
            return Err(Error::BufferOverflow);
        }
        buf[..len].copy_from_slice(&cmd.data[..len]);
        Ok(len)
    }

    /// Reads one HCI ACL packet from the bulk OUT endpoint.
    ///
    /// ACL packets can be split over multiple USB packets by the host; this
    /// reassembles a whole ACL packet. `buf` must be large enough to hold a
    /// whole ACL packet (see [`HCI_ACL_MAX_LEN`]).
    pub async fn read_acl(&self, buf: &mut [u8]) -> Result<usize, Error> {
        let mut ep = self.acl_out_ep.borrow_mut();
        let mps = ep.info().max_packet_size as usize;
        let mut total = 0;
        loop {
            if buf.len() - total < mps {
                return Err(Error::BufferOverflow);
            }
            let n = ep.read(&mut buf[total..]).await?;
            if total == 0 && n == 0 {
                continue;
            }
            total += n;
            if total >= 4 {
                let expected = 4 + u16::from_le_bytes(buf[2..4].try_into().unwrap()) as usize;
                if total > expected {
                    warn!(
                        "bluetooth hci: received more ACL data than expected ({} > {})",
                        total, expected
                    );
                }
                if total >= expected {
                    return Ok(total);
                }
            }
        }
    }
}
