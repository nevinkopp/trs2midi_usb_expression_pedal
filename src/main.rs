#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{adc::{Adc, Channel, Config as AdcConfig}, bind_interrupts, peripherals::USB, usb::{Driver, InterruptHandler},};
use embassy_time::{Duration, Timer};
use embassy_usb::{class::midi::MidiClass, Builder, Config as UsbConfig, UsbDevice,};
use static_cell::StaticCell;
use defmt_rtt as _;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
    USBCTRL_IRQ => InterruptHandler<USB>;
});


type UsbDriver = Driver<'static, USB>;
type UsbDeviceType = UsbDevice<'static, UsbDriver>;


#[embassy_executor::task]
async fn usb_task(mut usb: UsbDeviceType) -> ! {
    usb.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("TRS2MIDI expression pedal starting");

    let mut adc = Adc::new(
        p.ADC,
        Irqs,
        AdcConfig::default(),
    );

    let mut pin = Channel::new_pin(
        p.PIN_26,
        embassy_rp::gpio::Pull::None,
    );

    // Setup RAM blocks for USB stuff
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    // build usb driver and configure it
    let driver = Driver::new(p.USB, Irqs);
    let mut config = UsbConfig::new(0xCAFE, 0x4001);

    config.manufacturer = Some("NEKTEK");
    config.product = Some("TRS2MIDI Expression Pedal");
    config.serial_number = Some("DM8008S");
    config.max_power = 100;
    config.max_packet_size_0 = 64;


    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        &mut [],
        CONTROL_BUF.init([0; 64]),
    );

    // Build the USB device and create midi 
    let midi = MidiClass::new(&mut builder, 1, 1, 64);
    let usb = builder.build();
    

    // unwrap fails if no usb host is found
    unwrap!(spawner.spawn(usb_task(usb)));
    info!("USB MIDI initialized");

    // split midi into rx and tx (similar to usart split)
    let (mut sender, _receiver) = midi.split();

    info!("Waiting for USB MIDI host...");
    sender.wait_connection().await;
    info!("USB MIDI host connected");

    let mut last_midi = 255u8;

    // filtering stuff, hysteresis helps with jitters, max/zero thresholds latch values so that near 0 goes to 0 and near 100 goes to 100
    // helps in cases where the pedal was stuck at 1% even though it was all the way up and when the pedal was stuck at 99% even though it was all the way down
    const MIDI_HYSTERESIS: u8 = 1;
    const ZERO_THRESHOLD: u8 = 1;
    const MAX_THRESHOLD: u8 = 126;

    // take an initial adc sample so we can use the 8 sample averaging filter starting with the true pedal position
    let initial_sample = match adc.read(&mut pin).await {
        Ok(value) => value,
        Err(_) => {
            warn!("Initial ADC read error");
            0
        }
    };

    let mut samples = [initial_sample; 8];
    let mut sample_index = 0;

    loop {
        match adc.read(&mut pin).await {
            Ok(adc_value) => {
                samples[sample_index] = adc_value;
                sample_index = (sample_index + 1) % samples.len();

                let average =
                    samples.iter().map(|&x| x as u32).sum::<u32>()
                    / samples.len() as u32;

                let mut midi_value = ((average * 127) / 4095) as u8;

                // set values to the thresholds for low and high (forces 0% and 100% when near min and max so pedal doesn't get stuck at 1% or 99%)
                if midi_value <= ZERO_THRESHOLD {
                    midi_value = 0;
                } else if midi_value >= MAX_THRESHOLD {
                    midi_value = 127;
                }

                // hysteresis keeps values constant unless they change by the MIDI_HYSTERESIS value. 1 Seemed to work well here but if you get a lot of noise you might need to increase it
                if midi_value.abs_diff(last_midi) >= MIDI_HYSTERESIS {
                    info!("MIDI val changed from: {} to {}", last_midi, midi_value);
                    last_midi = midi_value;

                    // 0x0B/0xB0 is a control message flag on midi channel 0. 11 is midi expression control. midi value is the pedal position
                    let packet = [0x0B, 0xB0, 11, midi_value];

                    info!(
                        "TX: {:02x} {:02x} {:02x} {:02x}",
                        packet[0],
                        packet[1],
                        packet[2],
                        packet[3]
                    );
                    // sends midi packet over usb 
                    match sender.write_packet(&packet).await {
                        Ok(()) => {
                            info!("MIDI packet sent");
                        }
                        Err(_) => {
                            warn!("MIDI packet send failed");
                        }
                    }
                }
            }

            Err(_) => {
                warn!("ADC read error");
            }
        }
        // 2 millis felt responsive
        Timer::after(Duration::from_millis(2)).await;
    }
}