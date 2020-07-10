# ESPboy_SGE.py for esp8266
# common micropython module for ESPBoy game board designed by Billy Cheung (c) 2020 07 09
# --usage--
# Using this common micropython game module, you can write micropython games to run
# on the SPI drive TFT ST7735S 128x128 RGB display, backlight driven by MCP4725 and buttons driven by MCP23017.
#
# Note:  esp8266 is very bad at running .py micropython source code files
# with its very limited CPU onboard memory of 32K
# so to run any program with > 300 lines of micropython codes combined (including all modules),
# you need to convert source files into byte code first to avoid running out of memory.
# Install a version of the  mpy-cross micropython pre-compiler that can run in your system (available from github).
# Type this command to convert gameESP.py to the byte code file gameESP.mpy  using mpy-cross.
#        mpy-cross ESPboy_SGE.py
# then copy the ESPboy_SGE.mpy file to the micropython's import directory on the flash
# create your game and leaverge the functions to display, read buttons and paddle and make sounds
# from the ESPboy_SGE class module.
# Add this line to your micropython game source code (examples attached, e.g. invader.py)
#       from ESPboy_SGE import ESPboy_sge, Rect
#       g=ESPboy_sge()
#
#
#
#-----------------------------------------
# SPI version of game board layout
# ----------------------------------------
# micropython game hat module to use SSD1306 SPI OLED, 6 buttons and a paddle
# SPI display runs 5 times faster than I2C  display in micropython and you need this speeds
# for games with many moving graphics (e.g. space invdader, breakout).
#
# Buttons are read through A0 using many resistors in a  Voltage Divider circuit
# ESP8266 (node MCU D1 mini)  micropython
#
# ST7735S 128x128 TFT RGB Display (SPI)
# GND    - GND
# VCC    - VCC (3V)
# D0/Sck - D5 (=GPIO14=HSCLK)
# D1/MOSI- D7 (=GPIO13=HMOSI)
# RES    - Reset
# DC     - D0 (=GPIO16=Wake)
# CS     - Connected to PB0 of MCP23017
# SPK1   - D3 (=GPIO0)
# SPK2   - VCC (3V)
# n.c.   - D6  (=GPIO12=HMISO), but cannot be used , otherwise program hangs

# MCP23017
# GND    - GND
# VCC    - VCC (3V)
# SCL    - D1 (SCL)
# SDA    - D2 (SDA)
# PB0    - TFT CS
# PA0    - Left
# PA1    - Up
# PA2    - Down
# PA3    - Right
# PA4    - A
# PA5    - B
# PA6    - side button Left
# PA7    - side button Right


# MCP4725
# GND    - GND
# VCC    - VCC (3V)
# SCL    - D1 (SCL)
# SDA    - D2 (SDA)
# Vout   - TFT BL

#WS2812B RGB LED (Neo Pixel)
# GND    - GND
# VCC    - VCC (3V)
# Din    - D4 (=GPIO2)

import time
import utime
from utime import sleep_ms,ticks_ms, ticks_us, ticks_diff
from machine import Pin, SPI, I2C, PWM, ADC, Timer
#import ssd1306
from random import getrandbits, seed
# MicroPython SSD1306 OLED driver, I2C and SPI interfaces

from micropython import const
import framebuf

# register definitions
SET_CONTRAST        = const(0x81)
SET_ENTIRE_ON       = const(0xa4)
SET_NORM_INV        = const(0xa6)
SET_DISP            = const(0xae)
SET_MEM_ADDR        = const(0x20)
SET_COL_ADDR        = const(0x21)
SET_PAGE_ADDR       = const(0x22)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP       = const(0xa0)
SET_MUX_RATIO       = const(0xa8)
SET_COM_OUT_DIR     = const(0xc0)
SET_DISP_OFFSET     = const(0xd3)
SET_COM_PIN_CFG     = const(0xda)
SET_DISP_CLK_DIV    = const(0xd5)
SET_PRECHARGE       = const(0xd9)
SET_VCOM_DESEL      = const(0xdb)
SET_CHARGE_PUMP     = const(0x8d)

# Subclassing FrameBuffer provides support for graphics primitives
# http://docs.micropython.org/en/latest/pyboard/library/framebuf.html
# modified for GAMEESP to dispable use of CS.
# CS pin will be reused as the 2nd paddel controller pins
#
class SSD1306(framebuf.FrameBuffer):
    def __init__(self, width, height, external_vcc):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        self.buffer = bytearray(self.pages * self.width)
        super().__init__(self.buffer, self.width, self.height, framebuf.MONO_VLSB)
        self.init_display()

    def init_display(self):
        for cmd in (
            SET_DISP | 0x00, # off
            # address setting
            SET_MEM_ADDR, 0x00, # horizontal
            # resolution and layout
            SET_DISP_START_LINE | 0x00,
            SET_SEG_REMAP | 0x01, # column addr 127 mapped to SEG0
            SET_MUX_RATIO, self.height - 1,
            SET_COM_OUT_DIR | 0x08, # scan from COM[N] to COM0
            SET_DISP_OFFSET, 0x00,
            SET_COM_PIN_CFG, 0x02 if self.height == 32 else 0x12,
            # timing and driving scheme
            SET_DISP_CLK_DIV, 0x80,
            SET_PRECHARGE, 0x22 if self.external_vcc else 0xf1,
            SET_VCOM_DESEL, 0x30, # 0.83*Vcc
            # display
            SET_CONTRAST, 0xff, # maximum
            SET_ENTIRE_ON, # output follows RAM contents
            SET_NORM_INV, # not inverted
            # charge pump
            SET_CHARGE_PUMP, 0x10 if self.external_vcc else 0x14,
            SET_DISP | 0x01): # on
            self.write_cmd(cmd)
        self.fill(0)
        self.show()

    def poweroff(self):
        self.write_cmd(SET_DISP | 0x00)

    def poweron(self):
        self.write_cmd(SET_DISP | 0x01)

    def contrast(self, contrast):
        self.write_cmd(SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        self.write_cmd(SET_NORM_INV | (invert & 1))

    def show(self):
        x0 = 0
        x1 = self.width - 1
        if self.width == 64:
            # displays with width of 64 pixels are shifted by 32
            x0 += 32
            x1 += 32
        self.write_cmd(SET_COL_ADDR)
        self.write_cmd(x0)
        self.write_cmd(x1)
        self.write_cmd(SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_data(self.buffer)


class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3c, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        self.write_list = [b'\x40', None] # Co=0, D/C#=1
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.temp[0] = 0x80 # Co=1, D/C#=0
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_data(self, buf):
        self.write_list[1] = buf
        self.i2c.writevto(self.addr, self.write_list)


class SSD1306_SPI(SSD1306):
#    def __init__(self, width, height, spi, dc, res, cs external_vcc=False):
    def __init__(self, width, height, spi, dc, res=None, external_vcc=False):

        self.rate = 10 * 1024 * 1024
        dc.init(dc.OUT, value=0)
#        res.init(res.OUT, value=0)
#        cs.init(cs.OUT, value=1)
        self.spi = spi
        self.dc = dc
#       self.res = res
#       self.cs = cs
        import time
#        self.res(1)
#        time.sleep_ms(1)
#        self.res(0)
#        time.sleep_ms(10)
#        self.res(1)
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.spi.init(baudrate=self.rate, polarity=0, phase=0)
#        self.cs(1)
        self.dc(0)
#        self.cs(0)
        self.spi.write(bytearray([cmd]))
#        self.cs(1)

    def write_data(self, buf):
        self.spi.init(baudrate=self.rate, polarity=0, phase=0)
#        self.cs(1)
        self.dc(1)
#        self.cs(0)
        self.spi.write(buf)
#        self.cs(1)

    def block(self, x0, y0, x1, y1, data):
          """Write a block of data to display.

          Args:
              x0 (int):  Starting X position.
              y0 (int):  Starting Y position.
              x1 (int):  Ending X position.
              y1 (int):  Ending Y position.
              data (bytes): Data buffer to write.
          """
          self.write_cmd(self.SET_COLUMN, x0, x1)
          self.write_cmd(self.SET_ROW, y0, y1)
          self.write_cmd(self.WRITE_RAM)
          self.write_data(data)

    def draw_sprite(self, buf, x, y, w, h):
        """Draw a sprite (optimized for horizontal drawing).

        Args:
            buf (bytearray): Buffer to draw.
            x (int): Starting X position.
            y (int): Starting Y position.
            w (int): Width of drawing.
            h (int): Height of drawing.
        """
        x2 = x + w - 1
        y2 = y + h - 1
        if self.is_off_grid(x, y, x2, y2):
            return
        self.block(x, y, x2, y2, buf)

    def draw_image(self, path, x=0, y=0, w=128, h=128):
        """Draw image from flash.

        Args:
            path (string): Image file path.
            x (int): X coordinate of image left.  Default is 0.
            y (int): Y coordinate of image top.  Default is 0.
            w (int): Width of image.  Default is 128.
            h (int): Height of image.  Default is 128.
        """
        x2 = x + w - 1
        y2 = y + h - 1
        if self.is_off_grid(x, y, x2, y2):
            return
        with open(path, "rb") as f:
            chunk_height = 1024 // w
            chunk_count, remainder = divmod(h, chunk_height)
            chunk_size = chunk_height * w * 2
            chunk_y = y
            if chunk_count:
                for c in range(0, chunk_count):
                    buf = f.read(chunk_size)
                    self.block(x, chunk_y,
                               x2, chunk_y + chunk_height - 1,
                               buf)
                    chunk_y += chunk_height
            if remainder:
                buf = f.read(remainder * w * 2)
                self.block(x, chunk_y,
                           x2, chunk_y + remainder - 1,
                           buf)


class MCP4725:
    def __init__(self,i2c, address=0x60) :
        self.i2c=i2c
        self.address=address
        self._writeBuffer=bytearray(2)

    def write(self,value):
        if value < 0:
            value=0
        value=value & 0xFFF
        self._writeBuffer[0]=(value>>8) & 0xFF
        self._writeBuffer[1]=value & 0xFF
        return self.i2c.writeto(self.address,self._writeBuffer)==2

OUT     = 0
IN      = 1
HIGH    = True
LOW     = False

RISING  = 1
FALLING = 2
BOTH    = 3

PUD_OFF = 0
PUD_DOWN= 1
PUD_UP  = 2

class MCP():
    """Base class to represent an MCP230xx series GPIO extender.  Is compatible
    with the Adafruit_GPIO BaseGPIO class so it can be used as a custom GPIO
    class for interacting with device.
    """

    def __init__(self, i2c, address=0x20):
        """Initialize MCP230xx at specified I2C address and bus number.  If bus
        is not specified it will default to the appropriate platform detected bus.
        """
        self.address = address
        self.i2c = i2c
        # Assume starting in ICON.BANK = 0 mode (sequential access).
        # Compute how many bytes are needed to store count of GPIO.
        self.gpio_bytes = self.NUM_GPIO//8
        # Buffer register values so they can be changed without reading.
        self.iodir = bytearray(self.gpio_bytes)  # Default direction to all inputs.
        self.gppu = bytearray(self.gpio_bytes)  # Default to pullups disabled.
        self.gpio = bytearray(self.gpio_bytes)
        # Write current direction and pullup buffer state.
        self.write_iodir()
        self.write_gppu()

    def _validate_pin(self, pin):
        """Promoted to mcp implementation from prior Adafruit GPIO superclass"""
        # Raise an exception if pin is outside the range of allowed values.
        if pin < 0 or pin >= self.NUM_GPIO:
            raise ValueError('Invalid GPIO value, must be between 0 and {0}.'.format(self.NUM_GPIO))

    def writeList(self, register, data):
        """Introduced to match the writeList implementation of the Adafruit I2C _device member"""
        return self.i2c.writeto_mem(self.address, register, data)

    def readList(self, register, length):
        """Introduced to match the readList implementation of the Adafruit I2C _device member"""
        return self.i2c.readfrom_mem(self.address, register, length)

    def setup(self, pin, value):
        """Set the input or output mode for a specified pin.  Mode should be
        either OUT or IN.
        """
        self._validate_pin(pin)
        # Set bit to 1 for input or 0 for output.
        if value == IN:
            self.iodir[int(pin/8)] |= 1 << (int(pin%8))
        elif value == OUT:
            self.iodir[int(pin/8)] &= ~(1 << (int(pin%8)))
        else:
            raise ValueError('Unexpected value.  Must be IN or OUT.')
        self.write_iodir()


    def output(self, pin, value):
        """Set the specified pin the provided high/low value.  Value should be
        either HIGH/LOW or a boolean (True = HIGH).
        """
        self.output_pins({pin: value})

    def output_pins(self, pins):
        """Set multiple pins high or low at once.  Pins should be a dict of pin
        name to pin value (HIGH/True for 1, LOW/False for 0).  All provided pins
        will be set to the given values.
        """
        [self._validate_pin(pin) for pin in pins.keys()]
        # Set each changed pin's bit.
        for pin, value in iter(pins.items()):
            if value:
                self.gpio[int(pin/8)] |= 1 << (int(pin%8))
            else:
                self.gpio[int(pin/8)] &= ~(1 << (int(pin%8)))
        # Write GPIO state.
        self.write_gpio()


    def input(self, pin, read=True):
        """Read the specified pin and return HIGH/True if the pin is pulled
        high, or LOW/False if pulled low.
        """
        return self.input_pins([pin], read)[0]

    def input_pins(self, pins, read=True):
        """Read multiple pins specified in the given list and return list of pin values
        HIGH/True if the pin is pulled high, or LOW/False if pulled low.
        """
        [self._validate_pin(pin) for pin in pins]
        if read:
            # Get GPIO state.
            self.read_gpio()
        # Return True if pin's bit is set.
        return [(self.gpio[int(pin/8)] & 1 << (int(pin%8))) > 0 for pin in pins]


    def pullup(self, pin, enabled):
        """Turn on the pull-up resistor for the specified pin if enabled is True,
        otherwise turn off the pull-up resistor.
        """
        self._validate_pin(pin)
        if enabled:
            self.gppu[int(pin/8)] |= 1 << (int(pin%8))
        else:
            self.gppu[int(pin/8)] &= ~(1 << (int(pin%8)))
        self.write_gppu()

    def read_gpio(self):
        self.gpio = self.readList(self.GPIO, self.gpio_bytes)

    def write_gpio(self, gpio=None):
        """Write the specified byte value to the GPIO registor.  If no value
        specified the current buffered value will be written.
        """
        if gpio is not None:
            self.gpio = gpio
        self.writeList(self.GPIO, self.gpio)

    def write_iodir(self, iodir=None):
        """Write the specified byte value to the IODIR registor.  If no value
        specified the current buffered value will be written.
        """
        if iodir is not None:
            self.iodir = iodir
        self.writeList(self.IODIR, self.iodir)

    def write_gppu(self, gppu=None):
        """Write the specified byte value to the GPPU registor.  If no value
        specified the current buffered value will be written.
        """
        if gppu is not None:
            self.gppu = gppu
        self.writeList(self.GPPU, self.gppu)


class MCP23017(MCP):
    """MCP23017-based GPIO class with 16 GPIO pins."""
    # Define number of pins and registor addresses.
    NUM_GPIO = 16
    IODIR    = 0x00
    GPIO     = 0x12
    GPPU     = 0x0C


class gameESP():
    max_vol = 6
    duty={0:0,1:1,2:3,3:5,4:10,5:70,6:512}
    tones = {
        ' ': 0,   # silence note
        'c3': 131,
        'd3': 147,
        'e3': 165,
        'f3': 175,
        'f#3': 185,
        'g3': 196,
        'g#3': 208,
        'a3': 220,
        "a#3": 233,
        'b3': 247,
        'c4': 262,
        'd4': 294,
        'e4': 330,
        'f4': 349,
        'f#4': 370,
        'g4': 392,
        'g#4': 415,
        'a4': 440,
        "a#4": 466,
        'b4': 494,
        'c5': 523,
        'c#5': 554,
        'd5': 587,
        'd#5': 622,
        'e5': 659,
        'f5': 698,
        'f#5': 740,
        'g5': 784,
        'g#5': 831,
        'a5': 880,
        'b5': 988,
# note the following can only be played by ESP32, as ESP8266 can play up to 1000Hz only.
        'c6': 1047,
        'c#6': 1109,
        'd6': 1175
    }


    def __init__(self):
        # True =  SPI display, False = I2C display
        self.ESP32 = False
        self.paddle2 = False
        self.useSPI = True
        self.displayTimer = ticks_ms()
        self.vol = int(self.max_vol/2) + 1
        seed(ticks_us())

        self.btnL = 1 << 0
        self.btnU = 1 << 1
        self.btnD = 1 << 2
        self.btnR = 1 << 3
        self.btnA = 1 << 4
        self.btnB = 1 << 5
        self.btnLFT = 1 << 6
        self.btnRGT = 1 << 7

        self.frameRate = 30
        self.screenW = 128
        self.screenH = 64
        self.maxBgm = 1
        self.bgm = 1
        self.songIndex = 0
        self.songStart = -1
        self.songEnd   = -1
        self.songLoop  = -3
        self.silence  = 0
        self.songSpeed = 1
        self.timeunit = 1
        self.notes = False
        self.songBuf = []
        self.Btns = 0
        self.lastBtns = 0
        self.adc = ADC(0)
        self.PinBuzzer = Pin(0, Pin.OUT)
        self.beeper = PWM(self.PinBuzzer, 500, duty=0)
        self.beeper2 = PWM(self.PinBuzzer, 500, duty=0)
        self.timerInitialized = False
        if self.useSPI :
            # configure oled display SPI SSD1306
            self.hspi = SPI(1, baudrate=8000000, polarity=0, phase=0)
            #DC, RES, CS
    #            self.display = SSD1306_SPI(128, 64, self.hspi, Pin(2), Pin(16), Pin(0))
            self.display = SSD1306_SPI(128, 64, self.hspi, Pin(16))
            # configure oled display I2C for MCP23017 and MCP4725
            self.i2c = I2C(-1, Pin(5), Pin(4))   # SCL, SDA
            self.io = MCP23017(self.i2c)

            Pins = list(range(0,7))
            for pinNum in Pins:
                self.io.setup(pinNum, IN)
                self.io.pullup(pinNum, True)
            Pins = list(range(8,15))
            for pinNum in Pins:
                self.io.setup(pinNum, OUT)

        else :  # I2C display

            # configure oled display I2C SSD1306
            self.i2c = I2C(-1, Pin(5), Pin(4))   # SCL, SDA
            self.display = SSD1306_I2C(128, 64, self.i2c)
            self.PinBtnL = Pin(12, Pin.IN, Pin.PULL_UP)
            self.PinBtnR = Pin(13, Pin.IN, Pin.PULL_UP)
            self.PinBtnU = Pin(14, Pin.IN, Pin.PULL_UP)
            self.PinBtnD = Pin(2, Pin.IN, Pin.PULL_UP)
            self.PinBtnA = Pin(0, Pin.IN, Pin.PULL_UP)
            self.PinBtnB = Pin(16, Pin.IN) #GPIO 16 always pull down cannot pull up



    def deinit(self) :
      self.beeper.deinit()
      self.beeper2.deinit()
      self.songIndex = 0
      if self.timerInitialized :
          self.timer.deinit()

    def getPaddle (self) :
      return self.adc.read()

    def getPaddle2 (self) :
      return self.adc.read()

    def pressed (self,btn) :
      return (self.Btns & btn)

    def justPressed (self,btn) :
      return (self.Btns & btn) and not (self.lastBtns & btn)

    def justReleased (self,btn) :
      return (self.lastBtns & btn) and not (self.Btns & btn)

    def getBtn(self) :
      self.lastBtns = self.Btns
      self.Btns = 0
      if self.useSPI :
          self.io.read_gpio()
          self.Btns = self.io.gpio[0]
          self.Btns ^= 0b11111111 #flip the binary values
      else : # I2C board, read buttons directly
           self.Btns = self.Btns | (not self.PinBtnU.value()) << 1 | (not self.PinBtnL.value()) << 2 | (not self.PinBtnR.value()) << 3 | (not self.PinBtnD.value()) << 4 | (not self.PinBtnA.value()) << 5 | (not self.PinBtnB.value())<< 6
      return self.Btns

    def  setVol(self) :
        if self.pressed(self.btnB):
            if self.justPressed(self.btnU) :
                self.vol= min (self.vol+1, self.max_vol)
                self.playTone('c4', 100)
                return True
            elif self.justPressed(self.btnD) :
                self.vol= max (self.vol-1, 0)
                self.playTone('d4', 100)
                return True

        return False

    def setFrameRate(self) :

        if self.justPressed(self.btnR) :
            self.frameRate = self.frameRate + 5 if self.frameRate < 120 else 5
            self.playTone('e4', 100)
            return True
        elif self.pressed(self.btnB) and self.justPressed(self.btnR) :
            self.frameRate = self.frameRate - 5 if self.frameRate > 5 else 120
            self.playTone('f4', 100)
            return True
        return False

    def playTone(self, tone, tone_duration, rest_duration=0):
        beeper = PWM(self.PinBuzzer, freq=self.tones[tone], duty=self.duty[self.vol])
        sleep_ms(tone_duration)
        beeper.deinit()
        sleep_ms(rest_duration)

    def playSound(self, freq, tone_duration, rest_duration=0):
        beeper = PWM(self.PinBuzzer, freq, duty=self.duty[self.vol])
        sleep_ms(tone_duration)
        beeper.deinit()
        sleep_ms(rest_duration)


    def handleInterrupt(self,timer):
        self.beeper2.deinit() # note has been played logn enough, now stop sound

        if self.songBuf[self.songIndex] == self.songLoop :
            self.songIndex = 3 # repeat from first note

        if self.songBuf[self.songIndex] != self.songEnd :
            if self.songBuf[self.songIndex] == 0 :
                self.beeper2 = PWM(self.PinBuzzer, 100,0)
            elif self.notes :
                self.beeper2 = PWM(self.PinBuzzer, self.tones[self.songBuf[self.songIndex]], self.duty[self.vol])
            else :
                self.beeper2 = PWM(self.PinBuzzer, self.songBuf[self.songIndex], self.duty[self.vol])
            self.timer.init(period=int(self.songBuf[self.songIndex+1] * self.timeunit * self.songSpeed) , mode=Timer.ONE_SHOT, callback=self.handleInterrupt)
            self.songIndex +=2

    def startSong(self, songBuf=None):
        if self.bgm :
            if songBuf != None :
                self.songBuf = songBuf
            if self.songBuf[0] != self.songStart :
                print ("Cannot start Song, Invalid songBuf")
                return False
            self.notes = self.songBuf[1]
            self.timeunit = self.songBuf[2]
            self.songIndex = 3
            if not self.timerInitialized :
                self.timerInitialized = True
                self.timer = Timer(1)
            self.timer.init(period=100 , mode=Timer.ONE_SHOT, callback=self.handleInterrupt)

    def stopSong(self):
        self.songIndex = 0

    def random (self, x, y) :
        return  getrandbits(20) % (y-x+1) + x

    def display_and_wait(self) :
        self.display.show()
        timer_dif = int(1000/self.frameRate) - ticks_diff(ticks_ms(), self.displayTimer)
        if timer_dif > 0 :
            sleep_ms(timer_dif)
        self.displayTimer=ticks_ms()

class Rect (object):
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h


    def move (self, vx, vy) :
        self.x = self.x + vx
        self.y = self.y + vy


    def colliderect (self, rect1) :
      if (self.x + self.w   > rect1.x and
        self.x < rect1.x + rect1.w  and
        self.y + self.h > rect1.y and
        self.y < rect1.y + rect1.h) :
        return True
      else:
        return False
