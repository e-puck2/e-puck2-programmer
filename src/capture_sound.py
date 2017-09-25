#!/usr/bin/env python3
"""
Reads sounds from the F7 discovery kit microphone
"""
import argparse
import progressbar
import serial
import wave
import struct


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port")
    parser.add_argument("output", help="WAV file for output")
    parser.add_argument(
        "--length",
        "-l",
        help="Buffer length (44.1k by default)",
        type=int,
        default=44100)
    parser.add_argument(
        "-b",
        "--baudrate",
        type=int,
        default=921600,
        help="Baudrate, default=921600")

    parser.add_argument(
        "--left",
        help="Use left/U5 microphone (default is right/U6)",
        action="store_true")

    parser.add_argument(
        "--gain",
        type=float,
        help="Manual gain for the signal (default is auto)")

    return parser.parse_args()


def main():
    args = parse_args()
    conn = serial.Serial(args.port, args.baudrate)

    # First place the board in dfsdm acquisition mode
    if args.left:
        conn.write("dfsdm left\r\n".encode())
    else:
        conn.write("dfsdm right\r\n".encode())
    buf = bytes()
    print("Placing board in acquisition mode... ", end="")
    while not buf.decode().endswith("Done !\r\n"):
        buf = buf + conn.read(1)
    print("done")

    # Then read the whole sample out
    buf = bytes()
    pbar = progressbar.ProgressBar(maxval=args.length).start()
    while len(buf) < 4 * args.length:
        pbar.update(len(buf) / 4)
        buf += conn.read(100)
    pbar.finish()

    # Unpack the buffer (sent as a 32 bit integers)
    data = struct.unpack('<' + 'i' * args.length, buf)

    # Compute the gain to maximize the dynamic range in the WAV file
    if args.gain:
        gain = args.gain
    else:
        gain = ((2**31) - 1) / max(abs(s) for s in data)

    # Write the WAV file
    with wave.open(args.output, 'wb') as f:
        f.setnchannels(1)
        f.setsampwidth(4)
        f.setframerate(44.1e3)
        print("Writing WAV file")
        bar = progressbar.ProgressBar()
        for d in bar(data):
            f.writeframes(struct.pack('i', int(d * gain)))

    print("The board is still capturing data, please"
          "reset it by pressing the black button.")


if __name__ == '__main__':
    main()
