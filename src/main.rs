//! Prism LCD driver — drives the MSI MPG CoreLiquid K360 pump LCD.
//!
//! Image DATA is blasted over the MI_00 **bulk** endpoint (ep 0x04) via nusb — the
//! path liquidctl can't reach (hidapi can't do bulk). The three control commands
//! ride the MI_01 **HID** interrupt endpoint via hidapi (the firmware's command
//! parser only listens there).
//!
//! Protocol RE'd from MSI Center; see `docs/coreliquid-lcd-protocol.md`,
//! capture `scratch/caps/upload_bmp.pcap`.
//!
//! Device: USB 0db0:b130 composite.
//!   MI_00 (iface 0, vendor 0xff): ep 0x04 Bulk-OUT, ep 0x83 Bulk-IN   <- nusb (data)
//!   MI_01 (iface 1, HID 0x03):    ep 0x02 Intr-OUT, ep 0x81 Intr-IN   <- hidapi (cmds)
//!
//! Wire format (all packets 64 bytes; data chunks = 60 payload + 2 pad):
//!   BEGIN    HID:  D0 C0 <len:u32 LE> <slot> 00..
//!   DATA     bulk: D0 C1 <up to 60 bytes of raw file> (pad to 64); read 64B ACK on 0x83
//!   FINALIZE HID:  D0 C2 00..
//!   SHOW     HID:  D0 70 00 <slot>  then  D0 70 01 <slot>  (double-write)
//!
//! Usage:
//!   prism-lcd-driver <image.bmp> [slot=0]            single upload (with 2s flash-erase wait)
//!   prism-lcd-driver <image.bmp> <slot> <frames>     STREAM test: loop <frames> uploads,
//!                                                    no erase-wait, alternating normal/inverted
//!                                                    (tests whether bulk is a live-display path)
//!   prism-lcd-driver --list                          dump interface/endpoint map

use std::time::{Duration, Instant};

use hidapi::HidApi;
use nusb::transfer::{Buffer, Bulk, In, Out};
use nusb::MaybeFuture;

const VID: u16 = 0x0db0;
const PID: u16 = 0xb130;
const IFACE_MI00: u8 = 0;
const EP_DATA_OUT: u8 = 0x04;
const EP_DATA_IN: u8 = 0x83;
const PKT: usize = 64;
const DATA_PAYLOAD: usize = 60;
const BMP_HEADER: usize = 54;
const TIMEOUT: Duration = Duration::from_secs(3);

macro_rules! log {
    ($($a:tt)*) => { eprintln!("[prism-lcd] {}", format!($($a)*)) };
}

struct Dev {
    hid: hidapi::HidDevice,
    dout: nusb::Endpoint<Bulk, Out>,
    din: nusb::Endpoint<Bulk, In>,
}

fn main() {
    if let Err(e) = run() {
        log!("FATAL: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .ok_or("usage: prism-lcd-driver <image.bmp|--list> [slot] [frames]")?;

    log!("opening USB {VID:04x}:{PID:04x} ...");
    let di = nusb::list_devices()
        .wait()?
        .find(|d| d.vendor_id() == VID && d.product_id() == PID)
        .ok_or("CoreLiquid 0db0:b130 not found")?;
    let device = di.open().wait()?;

    if path == "--list" {
        return list_descriptor(&device);
    }
    if path == "--probe" {
        return probe_descriptors(&device);
    }

    let slot: u8 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0);
    let frames: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(1);

    let data = std::fs::read(&path)?;
    let len = data.len();
    log!("file '{}' = {} bytes; slot={} frames={}", path, len, slot, frames);

    let interface = device.claim_interface(IFACE_MI00).wait()?;
    let dout = interface.endpoint::<Bulk, Out>(EP_DATA_OUT)?;
    let din = interface.endpoint::<Bulk, In>(EP_DATA_IN)?;
    let api = HidApi::new()?;
    let hid = api.open(VID, PID)?;
    log!("claimed MI_00 bulk + MI_01 HID ({})", hid.get_product_string().ok().flatten().unwrap_or_default());
    let mut dev = Dev { hid, dout, din };

    if frames <= 1 {
        upload_frame(&mut dev, &data, slot, true)?;
        log!("OK — uploaded + shown on slot {}.", slot);
        return Ok(());
    }

    // STREAM TEST — loop uploads with no erase-wait, alternating normal / color-inverted
    // pixel data so a live-display path is visible as flicker between the two.
    let mut inverted = data.clone();
    for b in &mut inverted[BMP_HEADER..] {
        *b = !*b;
    }
    log!("STREAM TEST: {} frames, alternating normal/inverted, no erase-wait ...", frames);
    let t0 = Instant::now();
    for i in 0..frames {
        let frame = if i % 2 == 0 { &data } else { &inverted };
        upload_frame(&mut dev, frame, slot, false)?;
    }
    let dt = t0.elapsed();
    log!("STREAM done: {} frames in {:.2?} = {:.2} fps", frames, dt, frames as f64 / dt.as_secs_f64());
    Ok(())
}

/// One full upload: BEGIN -> (optional 2s erase wait) -> bulk DATA+ACK -> FINALIZE -> SHOW.
fn upload_frame(dev: &mut Dev, data: &[u8], slot: u8, erase_wait: bool) -> Result<(), Box<dyn std::error::Error>> {
    let len = data.len() as u32;

    let mut begin = [0u8; PKT];
    begin[0] = 0xD0;
    begin[1] = 0xC0;
    begin[2..6].copy_from_slice(&len.to_le_bytes());
    begin[6] = slot;
    hid_cmd(&dev.hid, &begin, "BEGIN")?;

    if erase_wait {
        log!("waiting 2s for slot erase/prepare ...");
        std::thread::sleep(Duration::from_secs(2));
    }

    for chunk in data.chunks(DATA_PAYLOAD) {
        let mut pkt = vec![0u8; PKT];
        pkt[0] = 0xD0;
        pkt[1] = 0xC1;
        pkt[2..2 + chunk.len()].copy_from_slice(chunk);
        dev.dout.submit(Buffer::from(pkt));
        dev.dout
            .wait_next_complete(TIMEOUT)
            .ok_or("DATA OUT timeout")?
            .into_result()
            .map_err(|e| format!("DATA OUT failed: {e:?}"))?;
        dev.din.submit(Buffer::new(PKT));
        dev.din
            .wait_next_complete(TIMEOUT)
            .ok_or("DATA IN(ack) timeout")?
            .into_result()
            .map_err(|e| format!("DATA IN(ack) failed: {e:?}"))?;
    }

    let mut fin = [0u8; PKT];
    fin[0] = 0xD0;
    fin[1] = 0xC2;
    hid_cmd(&dev.hid, &fin, "FINALIZE")?;

    let mut show_sel = [0u8; PKT];
    show_sel[0] = 0xD0;
    show_sel[1] = 0x70;
    show_sel[2] = 0x00;
    show_sel[3] = slot;
    hid_cmd(&dev.hid, &show_sel, "SHOW(select)")?;
    let mut show = [0u8; PKT];
    show[0] = 0xD0;
    show[1] = 0x70;
    show[2] = 0x01;
    show[3] = slot;
    hid_cmd(&dev.hid, &show, "SHOW")?;
    Ok(())
}

/// Send one 64-byte command as a HID output report (byte 0 = report id 0xD0).
fn hid_cmd(hid: &hidapi::HidDevice, pkt: &[u8; PKT], _what: &str) -> Result<(), Box<dyn std::error::Error>> {
    let n = hid.write(pkt)?;
    if n != pkt.len() {
        return Err(format!("{_what}: short HID write {n}/{}", pkt.len()).into());
    }
    let mut resp = [0u8; PKT];
    let _ = hid.read_timeout(&mut resp, 50); // drain response; tolerate timeout
    Ok(())
}

/// Read-only dump of the raw USB descriptors (device/config/BOS/MS-OS/strings).
fn probe_descriptors(device: &nusb::Device) -> Result<(), Box<dyn std::error::Error>> {
    let hex = |b: &[u8]| b.iter().map(|x| format!("{x:02x}")).collect::<String>();
    let ascii = |b: &[u8]| b.iter().map(|&x| if (32..127).contains(&x) { x as char } else { '.' }).collect::<String>();

    match device.get_descriptor(0x01, 0, 0, TIMEOUT).wait() {
        Ok(d) => {
            log!("device descriptor ({} B): {}", d.len(), hex(&d));
            if d.len() >= 14 {
                log!("  bcdUSB={:04x} bcdDevice={:04x} class={:#04x} maxpkt0={}",
                    u16::from_le_bytes([d[2], d[3]]), u16::from_le_bytes([d[12], d[13]]), d[4], d[7]);
            }
        }
        Err(e) => log!("device descriptor: err {e:?}"),
    }
    match device.get_descriptor(0x02, 0, 0, TIMEOUT).wait() {
        Ok(d) => log!("config descriptor ({} B): {}", d.len(), hex(&d)),
        Err(e) => log!("config descriptor: err {e:?}"),
    }
    match device.get_descriptor(0x0F, 0, 0, TIMEOUT).wait() {
        Ok(d) => log!("BOS descriptor ({} B): {}", d.len(), hex(&d)),
        Err(e) => log!("BOS descriptor: none ({e:?})"),
    }
    match device.get_descriptor(0x03, 0xEE, 0, TIMEOUT).wait() {
        Ok(d) => log!("MS-OS string (0xEE) ({} B): {}  ascii={}", d.len(), hex(&d), ascii(&d)),
        Err(e) => log!("MS-OS string (0xEE): none ({e:?})"),
    }
    match device.get_string_descriptor_supported_languages(TIMEOUT).wait() {
        Ok(langs) => {
            let langs: Vec<u16> = langs.collect();
            log!("string langids: {:04x?}", langs);
            let lang = langs.first().copied().unwrap_or(0x0409);
            for i in 1u8..=4 {
                let nz = std::num::NonZeroU8::new(i).unwrap();
                match device.get_string_descriptor(nz, lang, TIMEOUT).wait() {
                    Ok(s) => log!("  string[{}] = {:?}", i, s),
                    Err(e) => log!("  string[{}]: ({e:?})", i),
                }
            }
        }
        Err(e) => log!("string langids: err {e:?}"),
    }
    Ok(())
}

fn list_descriptor(device: &nusb::Device) -> Result<(), Box<dyn std::error::Error>> {
    let config = device.active_configuration()?;
    for intf in config.interfaces() {
        for alt in intf.alt_settings() {
            log!("interface {} alt {} class={:#04x} ({} endpoints)", alt.interface_number(), alt.alternate_setting(), alt.class(), alt.num_endpoints());
            for ep in alt.endpoints() {
                log!("    ep {:#04x}  {:?}  {:?}  mps={}", ep.address(), ep.transfer_type(), ep.direction(), ep.max_packet_size());
            }
        }
    }
    Ok(())
}
