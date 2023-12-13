#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2017-18 Richard Hull and contributors
# License: https://github.com/rm-hull/luma.led_matrix/blob/master/LICENSE.rst
# Github link: https://github.com/rm-hull/luma.led_matrix/

# Alle benötigten Module importieren
import RPi.GPIO as GPIO
import smbus
import time
import dht11
import re
from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
from luma.core.render import canvas
from luma.core.virtual import viewport
from luma.core.legacy import text, show_message
from luma.core.legacy.font import proportional, CP437_FONT, TINY_FONT, SINCLAIR_FONT, LCD_FONT
import board
import busio
import adafruit_character_lcd.character_lcd_i2c as character_lcd

from adafruit_ht16k33.segments import Seg7x4

# Definiere LCD Zeilen und Spaltenanzahl.
lcd_columns = 16
lcd_rows    = 2

# Initialisierung I2C Bus
i2c = busio.I2C(board.SCL, board.SDA)

# Festlegen des LCDs in die Variable LCD
lcd = character_lcd.Character_LCD_I2C(i2c, lcd_columns, lcd_rows, 0x21)


i2c = board.I2C()
segment = Seg7x4(i2c, address=0x70) #segment der I2C Adresse 0x70 und die Displaydefinition zuweisen

segment.fill(0) # Initialisierung des Displays. Muss einmal ausgeführt werden bevor das Display benutzt wird.

print ("STRG+C Druecken zum beenden.")
# Initialisierung GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()
testTemp = 0
testHum = 0

# Pin 14 benutzen, um Daten auslesen 
instance = dht11.DHT11(pin = 4)
#definieren des Symboles Grad °
degrees = bytes([0x7, 0x5, 0x7, 0x0, 0x0, 0x0, 0x0, 0x0])
lcd.create_char(0, degrees)

if(GPIO.RPI_REVISION == 1):
    bus = smbus.SMBus(0)
else:
    bus = smbus.SMBus(1)

class LightSensor():

    def __init__(self):

        # Definiere Konstante vom Datenblatt

        self.DEVICE = 0x5c # Standart I2C Geräteadresse

        self.POWER_DOWN = 0x00 # Kein aktiver zustand
        self.POWER_ON = 0x01 # Betriebsbereit
        self.RESET = 0x07 # Reset des Data registers

        # Starte Messungen ab 4 Lux.
        self.CONTINUOUS_LOW_RES_MODE = 0x13
        # Starte Messungen ab 1 Lux.
        self.CONTINUOUS_HIGH_RES_MODE_1 = 0x10
        # Starte Messungen ab 0.5 Lux.
        self.CONTINUOUS_HIGH_RES_MODE_2 = 0x11
        # Starte Messungen ab 1 Lux.
        # Nach messung wird Gerät in einen inaktiven Zustand gesetzt.
        self.ONE_TIME_HIGH_RES_MODE_1 = 0x20
        # Starte Messungen ab 0.5 Lux.
        # Nach messung wird Gerät in einen inaktiven Zustand gesetzt.
        self.ONE_TIME_HIGH_RES_MODE_2 = 0x21
        # Starte Messungen ab 4 Lux.
        # Nach messung wird Gerät in einen inaktiven Zustand gesetzt.
        self.ONE_TIME_LOW_RES_MODE = 0x23


    def convertToNumber(self, data):

        # Einfache Funktion um 2 Bytes Daten
        # in eine Dezimalzahl umzuwandeln
        return ((data[1] + (256 * data[0])) / 1.2)

    def readLight(self):

        data = bus.read_i2c_block_data(self.DEVICE,self.ONE_TIME_HIGH_RES_MODE_1)
        return self.convertToNumber(data)

def main(cascaded, block_orientation, rotate):
    
    # Matrix Gerät festlegen und erstellen.
    serial = spi(port=0, device=1, gpio=noop())
    device = max7219(serial, cascaded=cascaded or 1, block_orientation=block_orientation,
    rotate=rotate or 0)
    # Matrix Initialisierung in der Konsole anzeigen
    print("[-] Matrix initialized")

    # Symbole festlegen
    out_pfeilRunter = [(3,1),(4,1),(3,2),(4,2),(3,3),(4,3),(1,4),(2,4),(3,4),(4,4),(5,4),(6,4),(2,5),(3,5),(4,5),(5,5),(3,6),(4,6)]
    out_pfeilHoch = [(3,6),(4,6),(3,5),(4,5),(3,4),(4,4),(1,3),(2,3),(3,3),(4,3),(5,3),(6,3),(2,2),(3,2),(4,2),(5,2),(3,1),(4,1)]
    out_richtigHaken = [(2,4),(3,5),(4,4),(5,3),(6,2)]
    
    sensor = LightSensor()
    try:
        while True:
            result = instance.read()
      
            while not result.is_valid(): # solange auslesen bis valide Werte bei dem DHT11-Sensor herauskommen
                result = instance.read()

            # Symbolausgabe mit ausgelesenen Lichtwerten als Bedingung
            if sensor.readLight() >= 30000 and sensor.readLight() <= 50000:
                with canvas(device) as draw:
                    draw.point(tuple(out_richtigHaken), fill="yellow")
            if sensor.readLight() < 30000:
                with canvas(device) as draw:
                    draw.point(tuple(out_pfeilHoch), fill="yellow")
            if sensor.readLight() > 50000:
                with canvas(device) as draw:
                    draw.point(tuple(out_pfeilRunter), fill="yellow")

            # 7-Segment-Anzeige leeren
            segment.fill(0)

            # lcd Anzeige leeren
            lcd.clear
            # Hintergrundbeleuchtung einschalten
            lcd.backlight = True
            # Anzeigen der Werte auf dem LCD Display
            lcd_message_temp = 'Temperatur:'+(str (int(result.temperature))+'\x00'+'C \n')
            lcd_message_luft = 'Luftfeuchte:'+(str (int(result.humidity))+'%')
            lcd.message = lcd_message_temp + lcd_message_luft

            # Ausgabe Temperatur aus 7-Segment-Anzeige
            segment.print(str(int(result.temperature))+'C')
            segment.show()

            # Verzögerung von 5s
            time.sleep(5) 

            # 7-Segment-Anzeige leeren
            segment.fill(0)

            # Ausgabe Luftfeuchtigkeit auf 7-Segment-Anzeige
            segment.print(str(int(result.humidity))+' ')
            segment.show()
            
            # Verzögerung von 2s
            time.sleep(2) 

    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    
    # cascaded = Anzahl von MAX7219 LED Matrixen, standart=1
    # block_orientation = choices 0, 90, -90, standart=0
    # rotate = choices 0, 1, 2, 3, Rotate display 0=0°, 1=90°, 2=180°, 3=270°, standart=0
   
    try:
        main(cascaded=1, block_orientation=90, rotate=0)
    except KeyboardInterrupt:
        pass

