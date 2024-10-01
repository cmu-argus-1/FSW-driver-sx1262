from sx1262 import SX1262
from microcontroller import pin
import time
import digitalio

tx_en = digitalio.DigitalInOut(pin.GPIO6)
tx_en.direction = digitalio.Direction.OUTPUT
tx_en.value = True

sx = SX1262(spi_bus=1, clk=pin.GPIO10, mosi=pin.GPIO11, miso=pin.GPIO12,
            cs=pin.GPIO13, irq=pin.GPIO14, rst=pin.GPIO15, gpio=pin.GPIO16)

# LoRa
sx.begin(freq=915.6, bw=125, sf=7, cr=8, syncWord=0x12,
         power=22, currentLimit=140.0, preambleLength=8,
         implicit=False, implicitLen=0xFF,
         crcOn=True, txIq=False, rxIq=False,
         tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)

while True:
    tx_radiohead_hdr = [0xFF, 0xFF, 0x00, 0x00]
    tx_argus_msg = [0x01, 0x00, 0x00, 0x04, 0xFF, 0xEE, 0xDD, 0xCC]
    tx_msg = bytearray(tx_radiohead_hdr + tx_argus_msg)

    sx.send(tx_msg)
    print("Transmittted")

    msg, err = sx.recv(len=0, timeout_en=True, timeout_ms=1000)

    if (len(msg) > 0):
        error = SX1262.STATUS[err]
        msg = msg[4:]
        print(msg)

    time.sleep(5)
