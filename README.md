# gameESPboy

# gameESP.py for esp8266
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
#        mpy-cross ganeESP.py
# then copy the gameESP.mpy file to the micropython's import directory on the flash
# create your game and leaverge the functions to display, read buttons and paddle and make sounds
# from the gameESP class module.
# Add this line to your micropython game source code (examples attached, e.g. invader.py)
#       from gameESP import gameESP, rect
#       g=gameESP()
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
