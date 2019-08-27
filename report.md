# Quest 6 Report

Authors: Genny Norman, Cameron MacDonald Surman, Ruby Zaveri, 12-14-18

## Summary

We used the car program from the previous quest but instead controlled it from a web client. The car was to traverse a course and collect information from 4 beacons. This information was concatenated to form a url that sent us to a QR code with a message "Happy Holidays." Since we demoed it via video we had a beacon that could switch through the four different colors and moved it to a different location each time the car collected that bit of data.


## Evaluation Criteria
1. Car is controlled remotely from DDNS, URL and rives successfully.
2. Each beacon is visited by the car and each message is decoded.
3. Webcam is incorporated.
4. Collision Avoidance
5. Concatenated message formed in program and decoded.

## Solution Design

To navigate the car, we settled on setting the ESP32 as a webserver that accepted get requests. At first, we tried to have the car move a set distance on button press, and then stop, but ran into issues using a wait inside the get request handler. However, there was so little latency that we ended up using each button to set the motors, and included an extra stop button. This worked well, and despite faulty motors, we were able to navigate our car across the pitfall table, ultimately receiving all codes. Our receiver works by looking for input. When it finds a 0x1B char, it stores the number associated, as well as the string following. These are stored in a list of strings, with the leading number minus 1 being used for the indices. Also, a flag is set depending on the received message. Once all flags are set, the messages are concatenated based on order, and sent to our tcp_client() function, which opens a socket on our client and sends the completed URL.



## Evidence of Car working for Demo
[Video of car driving set up](https://drive.google.com/open?id=1eccoH6M6_9meseYa3MxEL0ZCOjxj-ljU)

** It's slow and a little jerky BUT it works!
[Video of car Driving](https://drive.google.com/open?id=1aQ8rOSVIQjkBTiJFhy71299-Pi3Yhc5-)


## Explanation of our Solution
[Solution Explanation](https://drive.google.com/open?id=1mfbLqL-iDgJFh8tyoiqqg20EieCBseV_)


## Sketches and Photos

Screenshots of web client

![Video of car Driving](https://i.imgur.com/MIcgJSN.png)

![Video of car Driving](https://i.imgur.com/53u5pj3.png)


Photo of Car

![Video of car Driving](https://i.imgur.com/E15EC86.png)

Photo of webcam set up

![Video of car Driving](https://i.imgur.com/FDi1zdP.png)


## Supporting Artifacts

- [H-Bridge Motor Driver Datasheet](https://cdn-shop.adafruit.com/datasheets/l293d.pdf)
