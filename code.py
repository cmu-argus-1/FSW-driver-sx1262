from hal.drivers.sx126x import SX1262
from microcontroller import pin
import time
import digitalio
import board
from busio import SPI


SPI_SCK = board.CLK0  # GPIO18
SPI_MOSI = board.MOSI0  # GPIO19
SPI_MISO = board.MISO0  # GPIO16
# SPI1 = SPI(SPI_SCK, MOSI=SPI_MOSI, MISO=SPI_MISO)
# RADIO_SPI = ArgusV2Interfaces.SPI
RADIO_CS = board.LORA_CS  # GPIO17
RADIO_RESET = board.LORA_nRST  # GPIO21
RADIO_ENABLE = board.LORA_EN  # GPIO28_ADC2
RADIO_TX_EN = board.LORA_TX_EN  # GPIO22
RADIO_RX_EN = board.LORA_RX_EN  # GPIO20
RADIO_BUSY = board.LORA_BUSY  # GPIO23
RADIO_IRQ = board.GPS_EN
# RADIO_FREQ = 915.6

sx = SX1262(
    spi_bus=1,
    clk=SPI_SCK,
    mosi=SPI_MOSI,
    miso=SPI_MISO,
    cs=RADIO_CS,
    irq=RADIO_IRQ,
    rst=RADIO_RESET,
    gpio=RADIO_BUSY,
)

# LoRa
sx.begin(
    freq=915.6,
    bw=125,
    sf=7,
    cr=8,
    syncWord=0x12,
    power=22,
    currentLimit=140.0,
    preambleLength=8,
    implicit=False,
    implicitLen=0xFF,
    crcOn=True,
    txIq=False,
    rxIq=False,
    tcxoVoltage=1.7,
    useRegulatorLDO=False,
    blocking=True,
)

radioEn = digitalio.DigitalInOut(RADIO_ENABLE)
radioRxEn = digitalio.DigitalInOut(RADIO_RX_EN)
radioTxEn = digitalio.DigitalInOut(RADIO_TX_EN)

radioEn.direction = digitalio.Direction.OUTPUT
radioRxEn.direction = digitalio.Direction.OUTPUT
radioTxEn.direction = digitalio.Direction.OUTPUT

radioEn.value = True
radioRxEn.value = True
radioTxEn.value = True

while True:
    tx_radiohead_hdr = [0xFF, 0xFF, 0x00, 0x00]
    tx_argus_msg = [0x01, 0x00, 0x00, 0x04, 0xFF, 0xEE, 0xDD, 0xCC]
    tx_msg = bytearray(tx_radiohead_hdr + tx_argus_msg)

    sx.send(tx_msg)
    print("Transmittted")
    time.sleep(5)

    if sx.rx_available():
        print("Message in RX buffer")
        msg, err = sx.recv(len=0, timeout_en=True, timeout_ms=1000)
        if (len(msg) > 0):
            error = SX1262.STATUS[err]
            msg = msg[4:]
            print(msg)
    else:
        print("Nothing in buffer")

    time.sleep(5)
