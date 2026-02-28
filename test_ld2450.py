from machine import UART, Pin
import time

# ================== XRPRadar CLASS ==================
class XRPRadar:
    COMMAND_HEADER = b'\xfd\xfc\xfb\xfa'
    COMMAND_TAIL   = b'\x04\x03\x02\x01'
    REPORT_HEADER  = b'\xaa\xff\x03\x00'
    REPORT_TAIL    = b'\x55\xcc'

    BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800]

    def __init__(self, uart_id, tx_pin, rx_pin, baudrate=256000):
        self.ser = UART(uart_id, baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))
        self.buffer = b""

    def poll_for_response(self):
        if self.ser.any():
            chunk = self.ser.read()
            if chunk:
                self.buffer += chunk
        if self.COMMAND_TAIL in self.buffer:
            parts = self.buffer.split(self.COMMAND_TAIL, 1)
            full = parts[0] + self.COMMAND_TAIL
            self.buffer = parts[1]
            return full
        return None

    def _send_command(self, intra_frame_length, command_word, command_value):
        command = self.COMMAND_HEADER + intra_frame_length + command_word + command_value + self.COMMAND_TAIL
        self.ser.write(command)

    def multi_target_tracking(self):
        self._send_command((2).to_bytes(2, 'little'), b'\x90\x00', b'')

    def parse_radar_report(self):
        if self.ser.any():
            self.buffer += self.ser.read()
        if self.REPORT_HEADER in self.buffer:
            start = self.buffer.find(self.REPORT_HEADER)
            if len(self.buffer) >= start + 30:
                frame = self.buffer[start : start + 30]
                if self.REPORT_TAIL in frame:
                    self.buffer = self.buffer[start + 30 :]
                    targets = []
                    for i in range(3):
                        offset = 4 + (i * 8)
                        t = frame[offset:offset+8]
                        # Decode as unsigned 16-bit little-endian
                        x_raw = int.from_bytes(t[0:2], 'little')
                        y_raw = int.from_bytes(t[2:4], 'little')
                        speed_raw = int.from_bytes(t[4:6], 'little')
                        res_raw = int.from_bytes(t[6:8], 'little')  # unsigned
    
                        # Sign-magnitude for x, y, speed
                        def signed_mag(raw):
                            sign = (raw & 0x8000) >> 15  # 1 positive, 0 negative
                            mag = raw & 0x7FFF
                            return mag if sign == 1 else -mag

                        x = signed_mag(x_raw) / 10.0  # cm
                        y = signed_mag(y_raw) / 10.0  # cm
                        speed = signed_mag(speed_raw)  # cm/s
                        res = res_raw / 10.0  # cm

                        targets.extend([x, y, speed, res])
                    return tuple(targets)
        return None

# ================== TEST PROGRAM ==================
radar = XRPRadar(uart_id=0, tx_pin=0, rx_pin=1)

print("HLK-LD2450 test starting...")
radar.multi_target_tracking()
time.sleep(0.5)

print("Looking for people... (move around in front of the sensor)")

while True:
    report = radar.parse_radar_report()
    if report:
        print("\n--- TARGETS DETECTED ---")
        for i in range(0, len(report), 4):
            x, y, speed, res = report[i:i+4]
            print(f"Target {i//4+1}:  X={x:4} cm   Y={y:4} cm   Speed={speed:3} cm/s   Res={res}")
    time.sleep(0.05)