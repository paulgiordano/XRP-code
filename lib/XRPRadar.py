from machine import UART, Pin
import time

class XRPRadar:
    # Protocol Constants
    COMMAND_HEADER = b'\xfd\xfc\xfb\xfa'
    COMMAND_TAIL = b'\x04\x03\x02\x01'
    REPORT_HEADER = b'\xaa\xff\x03\x00'
    REPORT_TAIL = b'\x55\xcc'
    
    BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800]

    def __init__(self, uart_id, tx_pin, rx_pin, baudrate=256000):
        """Initializes the UART and persistent buffer."""
        self.ser = UART(uart_id, baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))
        self.buffer = b""

    def poll_for_response(self):
        """
        Reads available UART data into the buffer.
        Returns the full response if COMMAND_TAIL is found, else None.
        """
        if self.ser.any():
            chunk = self.ser.read()
            if chunk:
                self.buffer += chunk
        
        if self.COMMAND_TAIL in self.buffer:
            # Extract the full command packet up to the tail
            parts = self.buffer.split(self.COMMAND_TAIL, 1)
            full_response = parts[0] + self.COMMAND_TAIL
            self.buffer = parts[1]  # Keep any trailing data (like report frames)
            return full_response
        return None

    def _send_command(self, intra_frame_length, command_word, command_value):
        """Constructs and sends a command frame."""
        command = self.COMMAND_HEADER + intra_frame_length + command_word + command_value + self.COMMAND_TAIL
        self.ser.write(command)

    def get_command_success(self, response):
        """Analyzes the response packet for the success code."""
        if response and len(response) >= 10:
            # Byte 8-9 is the success code (0 = success)
            success_int = int.from_bytes(response[8:10], 'little')
            return success_int == 0
        return False

    # --- Command Methods ---

    def enable_configuration_mode(self):
        self._send_command((4).to_bytes(2, 'little'), b'\xff\x00', b'\x01\x00')

    def end_configuration_mode(self):
        self._send_command((2).to_bytes(2, 'little'), b'\xfe\x00', b'')

    def single_target_tracking(self):
        self._send_command((2).to_bytes(2, 'little'), b'\x80\x00', b'')

    def multi_target_tracking(self):
        self._send_command((2).to_bytes(2, 'little'), b'\x90\x00', b'')

    def query_target_tracking(self):
        self._send_command((2).to_bytes(2, 'little'), b'\x91\x00', b'')

    def read_firmware_version(self):
        self._send_command((2).to_bytes(2, 'little'), b'\xa0\x00', b'')

    def set_baud_rate(self, baud_rate):
        if baud_rate not in self.BAUD_RATES:
            raise ValueError("Invalid baud rate")
        index = self.BAUD_RATES.index(baud_rate)
        self._send_command((4).to_bytes(2, 'little'), b'\xa1\x00', index.to_bytes(2, 'little'))

    def restore_factory_settings(self):
        self._send_command((2).to_bytes(2, 'little'), b'\xa2\x00', b'')

    def restart_module(self):
        self._send_command((2).to_bytes(2, 'little'), b'\xa3\x00', b'')

    def bluetooth_setup(self, on=True):
        val = b'\x01\x00' if on else b'\x00\x00'
        self._send_command((4).to_bytes(2, 'little'), b'\xa4\x00', val)

    def get_mac_address(self):
        self._send_command((4).to_bytes(2, 'little'), b'\xa5\x00', b'\x01\x00')

    # --- Data Parsing ---
    def parse_radar_report(self):
        if self.ser.any():
            self.buffer += self.ser.read()
        while self.REPORT_HEADER in self.buffer:
            start = self.buffer.find(self.REPORT_HEADER)
            # Look for tail starting from start + 28
            tail_start = self.buffer.find(self.REPORT_TAIL, start + 28)
            if tail_start != -1 and tail_start - start == 28:
                frame = self.buffer[start : tail_start + 2]  # 30 bytes: header to tail inclusive
#                print("Frame hex:", ' '.join(f'{b:02X}' for b in frame))  # debug
                self.buffer = self.buffer[tail_start + 2 :]  # remove processed frame
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
#                print("Parsed targets:", targets)  # debug
                return tuple(targets)
            else:
                # Invalid frame - discard up to next header or clear partial
                self.buffer = self.buffer[start + 4 :]  # skip header, try next
        return None