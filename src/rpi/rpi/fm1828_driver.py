import serial
import serial.tools.list_ports
from serial import SerialException
from math import pi

# Constants
FRAME_SIZE = 62  # Number of bytes in a valid row
FRAME_START_VAL = 250  # The first frame of each byte starts with this unique value. Indicates the start of a frame.
FRAME_END_FIRST_INDEX = (
    50  # The end of a frame is repeating zeros, starting at this index
)
FRAME_END_LAST_INDEX = 60  # The end of a frame is repeating zeros, ending at this index
SECOND_BYTE_MIN = 160  # If the second byte of a detect frame is >= this value, then it is a distance reading that we want
SECOND_BYTE_MAX = 249  # If the second byte of a detect frame is <= this value, then it is a distance reading that we want
DISTANCE_FIRST_INDEX = (
    4  # The index of a single frame of bytes that the first distance reading begins at
)
DISTANCE_BYTES_PER_ROW = 32  # Number of bytes in a frame that contain distance info - starting from DISTANCE_FIRST_INDEX. Keep in mind every second byte is a distance multiplier
NUM_READINGS_PER_REV = int(
    (SECOND_BYTE_MAX - SECOND_BYTE_MIN + 1) * (DISTANCE_BYTES_PER_ROW / 2)
)  # Total number of distance readings per 360deg revolution


class FM1828Driver:
    def __init__(
        self, port="/dev/serial0", baudrate=460800, timeout=1, data_callback=None
    ):
        """Initialize the serial connection and set the callback."""
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            print(f"Connected to serial port: {port}")
            self.ser.write(b"startlds$\n")
            print("Sent initialization command: startlds$")
        except SerialException as e:
            print(f"Error opening serial port: {e}")
            raise

        self.buffer = bytearray()
        self.ranges = [float("inf")] * NUM_READINGS_PER_REV
        self.intensities = [0.0] * NUM_READINGS_PER_REV
        self.received_sequences = set()
        self.data_callback = (
            data_callback  # Callback function to notify when data is ready
        )

    def read_serial_data(self):
        """Reads data from the serial port and processes frames."""
        try:
            # Read bytes from the serial port
            data = self.ser.read(
                self.ser.in_waiting or 1
            )  # Read available data (or 1 byte if no data available)

            if data:
                # Append the incoming data to the buffer
                self.buffer.extend(data)

                # Process frames if we have enough data
                while len(self.buffer) >= FRAME_SIZE:
                    # Check if the start byte is correct (250)
                    if self.buffer[0] != FRAME_START_VAL:
                        # If the first byte is not 250, discard it and move on
                        self.buffer.pop(0)
                        continue

                    # Now we have a potential frame starting with 250, check if we have enough bytes for a full frame
                    if len(self.buffer) < FRAME_SIZE:
                        break  # Not enough bytes yet, wait for more data

                    # Extract a frame of 62 bytes
                    frame = self.buffer[:FRAME_SIZE]
                    self.buffer = self.buffer[
                        FRAME_SIZE:
                    ]  # Remove the processed frame from the buffer

                    # Check if the second byte (sequence number) is within the valid range
                    if SECOND_BYTE_MIN <= frame[1] <= SECOND_BYTE_MAX:
                        # Ensure bytes 50 to 59 are all zeros
                        if all(
                            b == 0
                            for b in frame[FRAME_END_FIRST_INDEX:FRAME_END_LAST_INDEX]
                        ):
                            # Valid frame detected, process the frame
                            sequence_num = frame[1]
                            self.process_frame(sequence_num, frame)
        except Exception as e:
            print(f"Error while reading serial data: {e}")

    def process_frame(self, sequence_num, frame):
        """Processes a valid frame and extracts distance data."""
        index = sequence_num - SECOND_BYTE_MIN

        for row_index in range(
            DISTANCE_FIRST_INDEX, DISTANCE_BYTES_PER_ROW + DISTANCE_FIRST_INDEX, 2
        ):
            distance = frame[row_index] + (frame[row_index + 1] * 255)
            range_index = (index * int(DISTANCE_BYTES_PER_ROW / 2)) + (
                int(row_index / 2) - DISTANCE_FIRST_INDEX
            )
            self.ranges[range_index] = distance / 1000.0
            self.intensities[range_index] = 1.0

            self.received_sequences.add(range_index)

        if len(self.received_sequences) == NUM_READINGS_PER_REV:
            self.notify_callback()

    def notify_callback(self):
        """Calls the callback function with the scan data if available."""
        if self.data_callback:
            scan_data = (self.ranges[:], self.intensities[:])
            self.data_callback(scan_data)

        # Reset for the next scan
        self.ranges = [float("inf")] * NUM_READINGS_PER_REV
        self.intensities = [0.0] * NUM_READINGS_PER_REV
        self.received_sequences.clear()

    def close(self):
        """Closes the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial port closed.")
