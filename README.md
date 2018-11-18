# LED Mirror
	by Dan Gentry
	11/18/2018

An LED strip project for my bathroom mirror

##  Hardware

I started with a Wemos D1 Mini.  The build uses four MOSFET transistors, one for each color on the RGBW strip, is based on these pages:

- https://5p.io/building-a-wifi-controlled-led-strip-controller/
- http://jakebergamin.com/2016/02/08/wifi-led-strip/

There is a TKTK that drops the 12V supply down to 5V for the board.

In addition, I added a push button on the side of the mirror that is used to cycle through a series of presets.

Finally, I added a DHT22 to update temperature and humidity readings for the room. (As you can imagine, both go up significantly durring a shower)

See the pics folder for a few views of the project.

## Software

The LED code is from Github user [corbanmailloux](https://github.com/corbanmailloux/esp-mqtt-rgb-led), who seems to be famous in the LED world.  I took his logic and squeezed it into my usual structure.  Modifications to the LED code include:

- Cycle through six presets using a push button
- Accept color_temp in mireds (converted to Kelvin, then to RGB)
- Color correction for the LED strip. The defaults come from the FastLED library.
- Last state is saved and then used when the light is turned on again.
- A 'strip test' cycles through the LED colors at startup. (Also available via a long press on the pushbutton)

The DHT22 code and publishing are also included.

Libraries support key functions such as Wifi Management, OTA updates, and the MQTT Cient communication.

See the supplied Home Assistant configuration snippet for an example of use.

Before compiing, copy config-sample.h as config.h and make enter your settings, passwords, etc.