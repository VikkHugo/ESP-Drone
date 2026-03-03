# ESP-Drone

<img src="https://github.com/Circuit-Digest/ESP-Drone/blob/8e10ca8f1701dee8f7fa0e6d4f82067c9e87177b/Tittle%20Image.png" width="" alt="alt_text" title="image_tooltip">
<br>
<a href="https://circuitdigest.com/tags/ESP32">
  <img src="https://img.shields.io/static/v1?label=&labelColor=505050&message=ESP32%20Tutorials%20Circuit%20Digest&color=0076D6&style=flat-square&logo=google-chrome&logoColor=0076D6" alt="ESP32 Tutorials Circuit Digest"/>
</a>
<br>

What started as a simple DIY ESP32 drone project gradually evolved into LiteWing, a developer-friendly drone development platform.

The original ESP-Drone focused on basic stabilization and Wi-Fi control using the ESP32 and MPU6050. As the project progressed, the firmware and hardware structure were refined and expanded to support additional capabilities such as:

- Height hold

- Position hold


LiteWing builds upon the foundation of this project and extends it into a more structured development platform.

Learn more:

<table align="center" width="100%">
<tr>

<td align="center" width="50%">

<a href="https://circuitdigest.com/wiki/litewing/">
  <b>LiteWing</b>
</a>
<br><br>

<a href="https://circuitdigest.com/wiki/litewing/">
  <img src="https://raw.githubusercontent.com/Circuit-Digest/LiteWing/main/Documents/Images/litewing.gif"
       width="100%"
       alt="LiteWing GIF">
</a>

</td>

<td align="center" width="50%">

<a href="https://circuitdigest.com/wiki/litewing-drone-positioning-module/">
  <b>LiteWing Positioning Module</b>
</a>
<br><br>

<a href="https://circuitdigest.com/wiki/litewing-drone-positioning-module/">
  <img src="https://raw.githubusercontent.com/Circuit-Digest/LiteWing/main/Documents/Images/litewing-drone-positioning-module.gif"
       width="100%"
       alt="LiteWing Positioning Module GIF">
</a>

</td>

</tr>
</table>

Drones have rapidly evolved from niche hobbies to versatile tools with a wide range of applications, from photography to agriculture or even for defense and military purposes. Depending on the application, features and payload capacity their price will vary from pocket changes to a few million. Even though the basic principle behind these drones may appear simple, there is a lot of technology and computation that goes behind it for proper operations and a desired result from these drones. For example, maintaining stability while in the air requires precision sensors such as a Gyroscope and proper processing of reading taken with it. In this project we are going to make such a drone which can be easily controlled using our phones.
This DIY drone is small in size and can be built using easily available components such as ESP32 modules, MPU6050 IMU, coreless motors and plastic propellers.

For a full tutorial check out our [ESP32 Drone Article](https://circuitdigest.com/microcontroller-projects/DIY-wifi-controlled-drone).
For working video:  [How to build an ESP32 Drone](https://youtu.be/uzZjk0TQKtU).


<br>**For anyone who is trying to flash the firmware using ESP32 Flash Download Tool, here are the file offsets for each binary**
<br>**0x1000 bootloader.bin**
<br>**0x8000 partition-table.bin**
<br>**0x10000 ESPDrone.bin**
<br>
[Note: As this projects are very simple we are only providing the code, schemaitic, and a few essential images if you want to get the images or code explanations do check out the Circuit Digest website.
<br>
<br>
