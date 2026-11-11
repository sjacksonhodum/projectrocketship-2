
import math
import time

import board
import busio
import digitalio
import displayio
import i2cdisplaybus
import neopixel
import terminalio
import usb_cdc
import adafruit_rfm9x
from adafruit_display_text import label
from adafruit_displayio_ssd1306 import SSD1306


MAGIC_0 = 0xA5
MAGIC_1 = 0x5A

PACKET_FAST = 0x01
PACKET_STATUS = 0x02
PACKET_ORIENTATION = 0x03

PACKET_LENGTHS = {PACKET_FAST: 40, PACKET_STATUS: 32, PACKET_ORIENTATION: 22}

STATUS_GPS_FIX = 1 << 2


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def read_u16_le(buffer, offset):
    return buffer[offset] | (buffer[offset + 1] << 8)


def read_i16_le(buffer, offset):
    value = read_u16_le(buffer, offset)
    return value - 0x10000 if value >= 0x8000 else value



RF95_FREQ_MHZ = 433.0

RF95_SPREADING_FACTOR = 8
RF95_BANDWIDTH_HZ = 250000
RF95_CODING_RATE = 5

LINK_TIMEOUT_S = 3.0

PAGE_INTERVAL_S = 0.5
PAGE_Y = 40

GLOW_PERIOD_S = 2.4
GLOW_MIN = 0.05
GLOW_MAX = 0.55
RX_BLINK_S = 0.08
TX_BLINK_S = 0.08
NEOPIXEL_BRIGHTNESS = 0.15


if usb_cdc.data is None:
    print(
        "# WARNING: usb_cdc.data is not enabled - telemetry/uplink won't "
        "work. Check boot.py is on CIRCUITPY and the board has been fully "
        "power-cycled (not just a code.py save) since it was added."
    )
else:
    print("# usb_cdc.data OK - telemetry/uplink active on the data port")

pixels = neopixel.NeoPixel(
    board.GP22, 3, brightness=NEOPIXEL_BRIGHTNESS, auto_write=False
)

spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
cs = digitalio.DigitalInOut(board.GP8)
reset = digitalio.DigitalInOut(board.GP9)

radio = None
try:
    radio = adafruit_rfm9x.RFM9x(spi, cs, reset, RF95_FREQ_MHZ)
    radio.signal_bandwidth = RF95_BANDWIDTH_HZ
    radio.coding_rate = RF95_CODING_RATE
    radio.spreading_factor = RF95_SPREADING_FACTOR
    print("# Radio OK")
except RuntimeError as exc:
    print("# RADIO INIT FAILED:", exc)

displayio.release_displays()
display = None
header_label = None
page_label = None
try:
    i2c = busio.I2C(board.GP1, board.GP0, frequency=400000)
    display_bus = i2cdisplaybus.I2CDisplayBus(i2c, device_address=0x3C)
    display = SSD1306(display_bus, width=128, height=64)

    group = displayio.Group()

    header_label = label.Label(
        terminalio.FONT, text="GROUND STATION\nNO SIGNAL",
        color=0xFFFFFF, x=0, y=6
    )
    group.append(header_label)

    page_label = label.Label(
        terminalio.FONT, text="waiting for packets...",
        color=0xFFFFFF, x=0, y=PAGE_Y
    )
    group.append(page_label)

    display.root_group = group
    display.auto_refresh = False
except Exception as exc:
    print("# OLED INIT FAILED:", exc)


state = {
    "link_up": False,
    "rssi": -127,
    "snr": 0.0,
    "gps_fix": False,
    "gps_satellites": 0,
    "altitude_m": 0.0,
    "battery_mv": 0,
    "total_good": 0,
    "packet_rate": 0,
    "last_rx": 0.0,
}



def write_data_line(text):
    data_port = usb_cdc.data
    if data_port is None:
        return
    try:
        data_port.write((text + "\n").encode("ascii"))
    except Exception as exc:
        print("# DATA WRITE ERROR:", exc)


def patch_status_rssi(buffer):
    rssi = max(-127, min(127, radio.last_rssi))
    snr = max(-127, min(127, round(radio.last_snr)))
    buffer[14] = rssi & 0xFF
    buffer[15] = snr & 0xFF

    crc_coverage = PACKET_LENGTHS[PACKET_STATUS] - 2
    crc = crc16(buffer[:crc_coverage])
    buffer[crc_coverage] = crc & 0xFF
    buffer[crc_coverage + 1] = (crc >> 8) & 0xFF


def handle_packet(packet):
    buffer = bytearray(packet)
    if len(buffer) < 4 or buffer[0] != MAGIC_0 or buffer[1] != MAGIC_1:
        return False

    packet_type = buffer[3]
    expected = PACKET_LENGTHS.get(packet_type)
    if expected is None or len(buffer) != expected:
        return False

    if packet_type == PACKET_FAST:
        state["altitude_m"] = read_i16_le(buffer, 18) / 100.0
        status = buffer[36]
        state["gps_fix"] = bool(status & STATUS_GPS_FIX)
        state["gps_satellites"] = buffer[37]
    elif packet_type == PACKET_STATUS:
        patch_status_rssi(buffer)
        state["battery_mv"] = read_u16_le(buffer, 10)

    write_data_line(" ".join("%02X" % b for b in buffer))

    state["rssi"] = radio.last_rssi
    state["snr"] = radio.last_snr
    state["link_up"] = True
    state["total_good"] += 1
    state["last_rx"] = time.monotonic()
    return True



_last_header_text = None


def update_header():
    global _last_header_text

    if header_label is None:
        return

    link_line = (
        "LINK OK  {} pkt/s".format(state["packet_rate"])
        if state["link_up"] else "NO SIGNAL"
    )
    text = "GROUND STATION\n" + link_line
    if text != _last_header_text:
        header_label.text = text
        _last_header_text = text
        if display is not None:
            display.refresh(minimum_frames_per_second=0)


def build_pages():
    gps_line = "FIX" if state["gps_fix"] else "NO FIX"
    return [
        "RSSI {}dBm SNR {:.1f}\n{} pkt/s TOTAL {}".format(
            state["rssi"], state["snr"], state["packet_rate"], state["total_good"],
        ),
        "GPS {} sats {}\nALT {:.0f}ft BATT {:.2f}V".format(
            gps_line, state["gps_satellites"], state["altitude_m"] * 3.28084,
            state["battery_mv"] / 1000.0,
        ),
    ]


_page_index = 0
_last_page_switch = 0.0


def update_page(now):
    global _page_index, _last_page_switch

    if page_label is None:
        return

    if now - _last_page_switch < PAGE_INTERVAL_S:
        return

    _last_page_switch = now
    pages = build_pages()
    _page_index = (_page_index + 1) % len(pages)
    page_label.text = pages[_page_index]
    if display is not None:
        display.refresh(minimum_frames_per_second=0)



RX_PIXEL = 0
TX_PIXEL = 1
IDLE_PIXEL = 2

_rx_until = 0.0
_tx_start = 0.0
_tx_until = 0.0


def trigger_pixel_event():
    global _rx_until, _tx_start, _tx_until
    now = time.monotonic()
    _rx_until = now + RX_BLINK_S
    _tx_start = _rx_until
    _tx_until = _tx_start + TX_BLINK_S


def update_pixels(now):
    phase = (now % GLOW_PERIOD_S) / GLOW_PERIOD_S
    level = GLOW_MIN + (GLOW_MAX - GLOW_MIN) * (0.5 - 0.5 * math.cos(phase * 2 * math.pi))
    v = round(level * 255)

    pixels[IDLE_PIXEL] = (v, v, v)
    pixels[RX_PIXEL] = (0, 0, 255) if now < _rx_until else (0, 0, 0)
    pixels[TX_PIXEL] = (255, 0, 0) if _tx_start <= now < _tx_until else (0, 0, 0)
    pixels.show()



_usb_line_buffer = bytearray()


def hex_to_bytes(text):
    if len(text) % 2 != 0:
        return None
    try:
        return bytes(int(text[i:i + 2], 16) for i in range(0, len(text), 2))
    except ValueError:
        return None


def handle_usb_line(line):
    try:
        text = line.decode("ascii").strip()
    except UnicodeError:
        return

    if not text.startswith("TX:"):
        return

    payload = hex_to_bytes(text[3:])
    if payload is None:
        print("# BAD TX HEX")
        return

    if radio is None:
        print("# TX requested but radio not available")
        return

    radio.send(payload)
    trigger_pixel_event()
    print("# TX", text[3:])


def check_usb_commands():
    try:
        _check_usb_commands_inner()
    except Exception as exc:
        print("# USB command handling error:", exc)
        _usb_line_buffer.clear()


def _check_usb_commands_inner():
    data_port = usb_cdc.data
    if data_port is None:
        return

    while data_port.in_waiting > 0:
        chunk = data_port.read(data_port.in_waiting)
        if not chunk:
            break
        for byte in chunk:
            if byte in (0x0A, 0x0D):
                if _usb_line_buffer:
                    handle_usb_line(bytes(_usb_line_buffer))
                    _usb_line_buffer.clear()
            elif len(_usb_line_buffer) < 64:
                _usb_line_buffer.append(byte)



rate_window_start = time.monotonic()
rate_window_count = 0

while True:
    check_usb_commands()

    packet = None
    if radio is not None:
        packet = radio.receive(timeout=0, keep_listening=True)

    now = time.monotonic()

    if packet is not None and handle_packet(packet):
        trigger_pixel_event()
        rate_window_count += 1

    if now - rate_window_start >= 1.0:
        state["packet_rate"] = rate_window_count
        rate_window_count = 0
        rate_window_start = now

    if state["link_up"] and (now - state["last_rx"]) > LINK_TIMEOUT_S:
        state["link_up"] = False

    update_header()
    update_page(now)
    update_pixels(now)
