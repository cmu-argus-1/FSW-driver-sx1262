from sx1262 import SX1262
from microcontroller import pin
import time

sx = SX1262(spi_bus=1, clk=pin.GPIO2, mosi=pin.GPIO3, miso=pin.GPIO4, cs=pin.GPIO5, irq=pin.GPIO14, rst=pin.GPIO13, gpio=pin.GPIO15)

# LoRa
sx.begin(freq=915.6, bw=125.0, sf=7, cr=8, syncWord=0x12,
         power=22, currentLimit=140.0, preambleLength=8,
         implicit=False, implicitLen=0xFF,
         crcOn=True, txIq=False, rxIq=False,
         tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)

# FSK
##sx.beginFSK(freq=923, br=48.0, freqDev=50.0, rxBw=156.2, power=-5, currentLimit=60.0,
##            preambleLength=16, dataShaping=0.5, syncWord=[0x2D, 0x01], syncBitsLength=16,
##            addrFilter=SX126X_GFSK_ADDRESS_FILT_OFF, addr=0x00, crcLength=2, crcInitial=0x1D0F, crcPolynomial=0x1021,
##            crcInverted=True, whiteningOn=True, whiteningInitial=0x0100,
##            fixedPacketLength=False, packetLength=0xFF, preambleDetectorLength=SX126X_GFSK_PREAMBLE_DETECT_16,
##            tcxoVoltage=1.6, useRegulatorLDO=False,
##            blocking=True)

while True:
    tx_radiohead_hdr = [0xFF, 0xFF, 0x00, 0x00]
    tx_argus_msg = [0x01, 0x00, 0x00, 0x01, 0xFF]
    tx_msg = bytearray(tx_radiohead_hdr + tx_argus_msg)

    sx.send(tx_msg)
    print("Transmittted")

    msg, err = sx.recv(len=0, timeout_en=True, timeout_ms=1000)

    if len(msg) > 0:
        error = SX1262.STATUS[err]
        msg = msg[4:]
        print(msg)

    time.sleep(10)
