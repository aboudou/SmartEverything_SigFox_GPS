# SmartEverything SigFox GPS
Example code showing of to send any datatype - 12 bytes max - (in this example, a struct of GPS coordinate) to SigFox network with a SmartEverything prototyping board (http://www.smarteverything.it/).


## Technical details
- GPS frame decoding is made with TinyGPS library (https://github.com/mikalhart/TinyGPS).
- SigFox modem communication is done through direct AT commands. This code is minimalist : it doesn't manage downlink messages, and it's error management is sort of non existant.


## Why don't I use official SmartEverything libraries ?
Note : you can find them here : https://github.com/ameltech/

- Amel Technology's GPS official library is sometimes unable to parse NMEA frames. It can read altitude, number of locked satellites, but sometimes lacks latitude and longitude.
- Amel Technology's SigFox official library can only send to SigFox network an array of char. If it's OK  to send "Hello" messages, it's rather limited when you want to send any other datatype. Moreover, this library fails to send message smaller than three characters.


## How is the data encoded ?

SigFox data frame is limited to 12 bytes max. The data we send to SigFox network is composed of
- latitude, float datatype, 4 bytes,
- longitude, float datatype, 4 bytes,
- altitude, integer datatype, 4 bytes (SmartEverything prototyping board uses an ARM Cortex M0, which is a 32 bits MCU).

All the bytes of these data are concatenated, into an hexadecimal encoded string. Example :
<pre>
- float value                : 1.5345
- bytes values               : 37 71 45 66
- hexadecimal representation : 25 47 2D 42
</pre>

The following SigFox frame 25472d427f6ac43fdc000000 is
<pre>
- 25472d42 : hexadecimal value of latitude,  43.319477081299
- 7f6ac43f : hexadecimal value of longitude, 1.534500002861
- dc000000 : hexadecimal value of latitude,  220
</pre>


## What can I find in this repository ?
- __sigfox_gps__ : the Arduino sketch + TinyGPS library.
- __decode.php__ : example code to show how to decode sigfox data sent by the Arduino sketch.
