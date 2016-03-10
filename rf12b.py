#!/usr/bin/python
#Modules used in this program

import time                     #For the usual naps
import spidev                  #SPI interface
import json                     #For writing index values to file
import RPi.GPIO as GPIO
import crcmod.predefined

#RF12 command codes
RF_RECV_CONTROL=0x94A0
RF_RECEIVER_ON=0x82D9
RF_XMITTER_ON=0x8279
RF_RX_FIFO_READ=0xB000
RF_IDLE_MODE=0x820D
READ_FIFO=0xB000
STATUS_READ=0x0000

STATUS_TX_READY=1


CRC_OFFSET = 5
SIZE_OFFSET= 0
RX_MODE = 1
TX_MODE = 0

STATE_LENGTH = 1
READ_DATA = 2

_mode = TX_MODE
_recv_state = STATE_LENGTH
_recv_buffer = [0] * 255
_r_buf_pos = 0
_remaining = 0
_packet_received = False
crc16_func = crcmod.predefined.mkCrcFun('crc-ccitt-false')

def status():
    return int(writeCmd(STATUS_READ))

def writeCmd(word) :
    res = spi.xfer2([(word >> 8) & 0xff, word & 0xff])
    return int((res[0] << 8) + res[1])

def packetAvailable() :
    return _packet_received

def receivePacket() :
    global _recv_buffer,_packet_received
    _packet_received = False
    print("Got packet: ")
    for b in _recv_buffer:
        print("%X " % b,)
def fifoReset():
    writeCmd(0xCA81)
    writeCmd(0xCA83)

def receiveFunction(pin) :
    pass

def init_RF12() :
   GPIO.setmode(GPIO.BOARD)
   GPIO.setup(18, GPIO.IN, pull_up_down = GPIO.PUD_UP)
   GPIO.setup(22, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

   writeCmd(0x0000)   # initial SPI transfer added to avoid power-up problem
   time.sleep (5)            # Give RFM12B time to boot up
   writeCmd(0x0000)
   writeCmd(0x80E7)   #CREG 1 EL (ena TX), EF (ena RX FIFO), 12.0pF
   writeCmd(0x82D9)   #er,!ebb,ET,ES,EX,!eb,!ew,DC // receive
   writeCmd(0xA640)   #CREG 3 96-3960 freq range of values within band
   writeCmd(0xC647)   #CREG 4 approx 49.2 Kbps, i.e. 10000/29/(1+6) Kbps
   writeCmd(0x94A0)   #CREG 5 VDI,FAST,BW=134kHz,0dBm,-91dBm
   writeCmd(0xC2AC)   #CREG 6 AL,!ml,DIG,DQD4
   writeCmd(0xCA83)   #CREG 7 0x83 FIFO8,2-SYNC,!ff,!DR
   writeCmd(0xCED4)   #SYNC=2DXX

   writeCmd(0xC483)   #CREG 9 @PWR,NO RSTRIC,!st,!fi,OE,EN
   writeCmd(0x9850)   #C!mp,90kHz,MAX OUT
   writeCmd(0xCC17)   #CPLL Setting Command
   writeCmd(0xE000)   #CREG 12 Wake-Up Timer Command. Disabled
   writeCmd(0xC800)   #CREG 13 Low Duty-Cycle Command. Disabled
   writeCmd(0xC040)   #CREG 14 1.66MHz,3.1V

   fifoReset()

   ##GPIO.add_event_detect(22, GPIO.RISING, callback=receiveFunction)

spi = spidev.SpiDev()
spi.open(0,1)               #Open SPI1 interface with CS0 low
spi.bits_per_word=8          #Couldn't get 16bits to work..
spi.mode=0                  #SPI mode.
spi.max_speed_hz=2000000      #SPI speed, RFM12B max clock 20 MHz (50ns clock cycle)
init_RF12()                  #Set up our RFM12B for receiving

_pan_id = 0xd4
length = 9



def rfSend(byte) :
    while GPIO.input(18) == 1 :
        pass
    spi.writebytes([0xB8,byte])

def sendBuffer(buf) :
    print("TX Packet")
    _mode = TX_MODE
    #GPIO.remove_event_detect(18)
    writeCmd(0x0000)
    writeCmd(RF_XMITTER_ON)
    #spi.writebytes(RF_XMITTER_ON)

    rfSend(0xAA) # PREAMBLE
    rfSend(0xAA)
    rfSend(0xAA)

    rfSend(0x2D); # SYNC
    rfSend(_pan_id)

    rfSend(len(buf)+1)        # SIZE

    # Calc crc
    #crc = checkCRC(buf)
    buf.insert(0,len(buf)+1)
    crc = crc16_func(str(bytearray(buf)))
    #CRCCCITT(version="FFFF").calculate(bytearray(buf))
    buf.pop(0)
    print(crc)
    buf[CRC_OFFSET+1] = (crc >> 8) & 0xff;
    buf[CRC_OFFSET] = crc & 0xff

    for b in buf:
        rfSend(b)

    rfSend(0xAA) # DUMMY BYTES
    rfSend(0xAA)
    rfSend(0xAA)

    #spi.writebytes([0xaa,0xaa,0xaa,0x2d,0xd4,0x09,0x00,0x01, 0x01,0x01,0x01,0xff,0xff,0x65,0xaa,0xaa,0xaa])
    writeCmd(RF_RECEIVER_ON)
    writeCmd(0x0000)
    _mode = RX_MODE
    #GPIO.add_event_detect(18, GPIO.RISING, callback=receiveFunction)

try:
    nextsend = int(time.time())
    recv_buffer = [0] * 255
    while True:
        if GPIO.wait_for_edge(22,GPIO.RISING):
            if status() & 0x8000 :
                remaining = writeCmd(READ_FIFO) & 0x00FF
                print("Remaining: %d" % remaining)
                for i in range(0,remaining):
                    while (status() & 0x8000) != 0x8000:
                        pass
                    recv_buffer[i] = writeCmd(READ_FIFO) & 0x00FF
                print(recv_buffer)
                fifoReset()
        #print status()
        if int(time.time()) > nextsend:
            sendBuffer([1,0,1,0,1,0,0,66,66,61,57,56])
            print("Status: %x" % status())
            nextsend = int(time.time()) + 5
        #if packetAvailable():
        #    receivePacket()
except KeyboardInterrupt:                     # Ctrl+C pressed, so...
    spi.close()
