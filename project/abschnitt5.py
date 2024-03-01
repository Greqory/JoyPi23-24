#!/usr/bin/env python
# Import von genutzten Libraries
import RPi.GPIO as GPIO
import time
import dht11
import board
from adafruit_ht16k33.segments import Seg7x4

i2c = board.I2C() # Initialisierung I2C Bus
segment = Seg7x4(i2c, address=0x70) #segment der I2C Adresse 0x70 und die Displaydefinition zuweisen

# Initialisierung GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()

# "instance" nutzen, um Daten bei GPIO Pin 4, aus DHT11 auszulesen
instance = dht11.DHT11(pin = 4)

print ("STRG+C Druecken zum beenden.")
try:
    while True:
        # solange auslesen bis valide Werte bei dem DHT11-Sensor herauskommen
        result = instance.read()
        while not result.is_valid():
            result = instance.read()

        # 7-Segment-Anzeige leeren
        segment.fill(0)

        # Ausgabe Temperatur aus 7-Segment-Anzeige
        segment.print(str(int(result.temperature)) +'C')
        segment.show()

        # Verzögerung von 2s
        time.sleep(2) 
        # 7-Segment-Anzeige leeren
        segment.fill(0)

        # Ausgabe Luftfeuchtigkeit auf 7-Segment-Anzeige
        segment.print(str(int(result.humidity))+'H')
        segment.show()
            
        # Verzögerung von 2s
        time.sleep(2) 

# Bei Beenden des Programms wird 7-Segment-Anzeige geleert 
except KeyboardInterrupt:
    segment.fill(0)
