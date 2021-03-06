#!/usr/bin/python
import time,datetime
import serial
import termios, os

# Map from the numbers to the termios constants (which are pretty much
# the same numbers).

BPS_SYMS = {
  4800: termios.B4800,
  9600: termios.B9600,
  19200: termios.B19200,
  38400: termios.B38400,
  57600: termios.B57600,
  115200: termios.B115200
  }


# Indices into the termios tuple.

IFLAG = 0
OFLAG = 1
CFLAG = 2
LFLAG = 3
ISPEED = 4
OSPEED = 5
CC = 6


def bps_to_termios_sym(bps):
  return BPS_SYMS[bps]

  
class SerialPort:

  def __init__(self, serialport, bps):
    """Takes the string name of the serial port
(e.g. "/dev/tty.usbserial","COM1") and a baud rate (bps) and
connects to that port at that speed and 8N1. Opens the port in
fully raw mode so you can send binary data.
"""
    self.fd = os.open(serialport, os.O_RDWR | os.O_NOCTTY | os.O_NDELAY)
    attrs = termios.tcgetattr(self.fd)
    bps_sym = bps_to_termios_sym(bps)
    # Set I/O speed.
    attrs[ISPEED] = bps_sym
    attrs[OSPEED] = bps_sym

    # 8N1
    attrs[CFLAG] &= ~termios.PARENB
    attrs[CFLAG] &= ~termios.CSTOPB
    attrs[CFLAG] &= ~termios.CSIZE
    attrs[CFLAG] |= termios.CS8
    # No flow control
    attrs[CFLAG] &= ~termios.CRTSCTS

    # Turn on READ & ignore contrll lines.
    attrs[CFLAG] |= termios.CREAD | termios.CLOCAL
    # Turn off software flow control.
    attrs[IFLAG] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)

    # Make raw.
    attrs[LFLAG] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
    attrs[OFLAG] &= ~termios.OPOST

    # It's complicated--See
    # http://unixwiz.net/techtips/termios-vmin-vtime.html
    attrs[CC][termios.VMIN] = 0;
    attrs[CC][termios.VTIME] = 20;
    termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

  def write(self, str):
    os.write(self.fd, str)

  def write_byte(self, byte):
    os.write(self.fd, chr(byte))

  def flush(self):
    return

arduino = serial.Serial(
    port='/dev/ttyUSB0', 
    baudrate=9600, 
    timeout=1,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
#arduino = SerialPort('/dev/ttyUSB0',9600)
time.sleep(2)
arduino.write("il est " + datetime.datetime.now().strftime('%H:%M'))
time.sleep(1)
arduino.write("et tout va bien")
arduino.flush()
