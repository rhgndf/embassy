//! This example exposes the Pico 2 W's CYW43439 Bluetooth controller as a USB HCI
//! dongle, using the Bluetooth HCI USB transport class.
//!
//! Raw HCI packets are bridged between USB and the cyw43 Bluetooth controller:
//! - HCI commands (control endpoint) and ACL data (bulk OUT) are forwarded to the
//!   controller.
//! - HCI events (interrupt IN) and ACL data (bulk IN) from the controller are
//!   forwarded to the USB host.
//!
//! On Linux, the `btusb` driver binds to the device automatically; verify with
//! `hciconfig -a`, `bluetoothctl` or `btmon`. SCO (voice) data is not supported.
//!
//! It does not work with the RP Pico 2 board (no WiFi chip). See `blinky.rs`.

#![no_std]
#![no_main]

use cyw43::aligned_bytes;
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join4;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::{bind_interrupts, dma};
use embassy_usb::Builder;
use embassy_usb::class::bluetooth_hci::{
    BluetoothHciClass, HCI_COMMAND_MAX_LEN, State, USB_CLASS_WIRELESS_CONTROLLER, USB_PROTOCOL_BLUETOOTH,
    USB_SUBCLASS_RF,
};
use panic_probe as _;
use static_cell::StaticCell;

// Program metadata for `picotool info`.
// This isn't needed, but it's recommended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"USB Bluetooth HCI Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"This example exposes the RP Pico 2 W's cyw43 Bluetooth controller as a USB HCI dongle."
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

// HCI H4 packet type indicators.
const HCI_COMMAND: u8 = 0x01;
const HCI_ACL: u8 = 0x02;
const HCI_EVENT: u8 = 0x04;

// HCI packet buffer size. The cyw43 HCI MTU is 1024 bytes including the
// indicator byte.
const HCI_MTU: usize = 1024;

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>, cyw43::Cyw43439>,
) -> ! {
    runner.run().await
}

#[embassy_executor::main(executor = "embassy_rp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let fw = aligned_bytes!("../../../../cyw43-firmware/43439A0.bin");
    let btfw = aligned_bytes!("../../../../cyw43-firmware/43439A0_btfw.bin");
    let clm = aligned_bytes!("../../../../cyw43-firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../../../../cyw43-firmware/nvram_rp2040.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        // SPI communication won't work if the speed is too high, so we use a divider larger than `DEFAULT_CLOCK_DIVIDER`.
        // See: https://github.com/embassy-rs/embassy/issues/3960.
        RM2_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        dma::Channel::new(p.DMA_CH0, Irqs),
        dma::Channel::new(p.DMA_CH1, Irqs),
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, bt_device, mut control, runner) =
        cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw, nvram).await;
    spawner.spawn(unwrap!(cyw43_task(runner)));

    control.init(clm).await;

    // Create the USB driver, from the HAL.
    let driver = UsbDriver::new(p.USB, Irqs);

    // Create embassy-usb Config.
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB Bluetooth HCI");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    // Advertise as a Bluetooth device so the Linux `btusb` driver binds to it.
    config.device_class = USB_CLASS_WIRELESS_CONTROLLER;
    config.device_sub_class = USB_SUBCLASS_RF;
    config.device_protocol = USB_PROTOCOL_BLUETOOTH;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    // HCI commands are received over the control endpoint, so this buffer must
    // be able to hold a whole HCI command packet.
    let mut control_buf = [0; HCI_COMMAND_MAX_LEN];

    let mut class_state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // Create the Bluetooth HCI class on the builder.
    let class = BluetoothHciClass::new(&mut builder, &mut class_state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Split the class into a sender (events + ACL to host), a command receiver
    // and an ACL receiver (both host to controller).
    let (mut sender, cmd_rx, mut acl_rx) = class.split();

    // Run the USB device.
    let usb_fut = usb.run();

    // Forward HCI commands (control endpoint) to the Bluetooth controller.
    // Commands only arrive when the host has configured the device, so there's
    // no need to wait for a connection here.
    let command_fut = async {
        let mut cmd_buf = [0u8; HCI_MTU];
        loop {
            match cmd_rx.read_command(&mut cmd_buf[1..]).await {
                Ok(n) => {
                    cmd_buf[0] = HCI_COMMAND;
                    if let Err(_e) = bt_device.write_raw(&cmd_buf[..1 + n]).await {
                        warn!("failed to send HCI command to controller");
                    }
                }
                Err(_e) => {
                    warn!("failed to read HCI command");
                }
            }
        }
    };

    // Forward HCI ACL packets (bulk OUT) to the Bluetooth controller.
    let acl_fut = async {
        let mut acl_buf = [0u8; HCI_MTU];
        loop {
            acl_rx.wait_connection().await;
            loop {
                match acl_rx.read_acl(&mut acl_buf[1..]).await {
                    Ok(n) => {
                        acl_buf[0] = HCI_ACL;
                        if let Err(_e) = bt_device.write_raw(&acl_buf[..1 + n]).await {
                            warn!("failed to send ACL packet to controller");
                        }
                    }
                    Err(_e) => {
                        // USB disconnected; wait for reconnection.
                        break;
                    }
                }
            }
        }
    };

    // Handle packets from the Bluetooth controller to the USB host.
    let controller_to_host_fut = async {
        let mut buf = [0u8; HCI_MTU];
        loop {
            sender.wait_connection().await;
            loop {
                match bt_device.read_raw(&mut buf).await {
                    Ok(n) => {
                        let res = match buf[0] {
                            HCI_EVENT => sender.write_event(&buf[1..n]).await,
                            HCI_ACL => sender.write_acl(&buf[1..n]).await,
                            _ => {
                                warn!("unknown HCI packet type {=u8}", buf[0]);
                                continue;
                            }
                        };
                        if res.is_err() {
                            // USB disconnected; wait for reconnection.
                            break;
                        }
                    }
                    Err(_e) => {
                        warn!("failed to read HCI packet from controller");
                    }
                }
            }
        }
    };

    // Run everything concurrently.
    join4(usb_fut, command_fut, acl_fut, controller_to_host_fut).await;
}
