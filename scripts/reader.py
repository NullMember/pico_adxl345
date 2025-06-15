import serial
import numpy
import struct
import soundfile

def read_serial_data(ser: serial.Serial, length: int):
    data = []
    count = 0
    adxl_range = 0
    adxl_rate = 0
    adxl_watermark = 0
    gain = 0.0039
    while count < length:
        serial_bytes = ser.read_until(bytes([0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF]))
        adxl_range = int(serial_bytes[0])
        adxl_rate = int(serial_bytes[1])
        adxl_watermark = int(serial_bytes[2])
        data.append(serial_bytes[3:-6])
        count += adxl_watermark
    for i in range(len(data)):
        if len(data[i]) != (adxl_watermark * 3 * 2):
            # print(f"Warning: Expected {16 * 3 * 2 + 6} bytes, got {len(data[i])} bytes.")
            continue
        data[i] = numpy.asarray(struct.unpack(f'<{16 * 3}h', data[i][:(16 * 3 * 2)])).reshape((16, 3))
    data = numpy.concatenate(data, axis=0)
    # print(f"Read {data.shape[0]} samples in {accumulated_time:.2f} seconds.")
    # print(f"Average time per sample: {accumulated_time / data.shape[0]:.6f} seconds.")
    # print(f"Calculated sample rate: {data.shape[0] / accumulated_time:.2f} Hz.")
    return data, adxl_range, adxl_rate, adxl_watermark

def main():
    samplerate = 3200
    duration = 10.0
    baudrate = 250000
    port = '/dev/tty.usbmodem11201'

    ser = serial.Serial(port, baudrate)

    gain = 0.0039

    data, adxl_range, adxl_rate, adxl_watermark = read_serial_data(ser, int(samplerate * duration))
    data = (data * gain) / (2 ** adxl_range)
    data_x = data[:, 0]
    data_y = data[:, 1]
    data_z = data[:, 2]

    soundfile.write('data_x.wav', data_x, samplerate)
    soundfile.write('data_y.wav', data_y, samplerate)
    soundfile.write('data_z.wav', data_z, samplerate)

if __name__ == "__main__":
    main()