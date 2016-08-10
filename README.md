# Things_Firmware
Lora Clients firmware for Airfy Mini and Airfy Things

=====================================

1. Introduction
----------------
The aim of this project is to show an example of the endpoint LoRaWAN stack implementation.

This LoRaWAN stack is an EU868 and US915 bands Class A and Class C endpoint implementation
fully compatible with LoRaWAN 1.0 specification.
Each LoRaWAN application example includes the LoRaWAN certification protocol implementation.

SX1272/76 radio drivers are also provided.
In case only point to point links are required a Ping-Pong application is provided as example.

*The LoRaWAN stack API documentation can be found at: http://stackforce.github.io/LoRaMac-doc/*

**Note 1:**

*A version 3.x API to version 4.x API wrapper is available.
Applications built using version 3.x API should work without modifications except that 
one must include LoRaMac-api-v3.h instead of LoRaMac.h file*



2. System schematic and definitions
------------------------------------
The available supported hardware platforms schematics can be found in the Doc directory.

3. Acknowledgments
-------------------
The mbed (https://mbed.org/) project was used at the beginning as source of
inspiration.

This program uses the AES algorithm implementation (http://www.gladman.me.uk/)
by Brian Gladman.

This program uses the CMAC algorithm implementation
(http://www.cse.chalmers.se/research/group/dcs/masters/contikisec/) by
Lander Casado, Philippas Tsigas.

4. Dependencies
----------------
This program depends on specific hardware platforms. Currently the supported
platforms are:

    - Airfy Things
        MCU     : nRF51822 
        RADIO   : SX1276
        ANTENNA : MMCX connector
        BUTTONS : Reset, General purpose button
        LEDS    : RGB
        SENSORS : SeeedStuidio Grove Compatible
        GPS     : No
        SDCARD  : No
        EXTENSION No
        REMARK  : Powered by two AAA battery


5. Usage
---------
Projects for CooCox-CoIDE and Keil Integrated Development Environments are available.

One project is available per application and for each hardware platform in each
development environment. Different targets/configurations have been created in
the different projects in order to select different options such as the usage or
not of a bootloader and the radio frequency band to be used.

6. Changelog
-------------

2016-06-06, V1.0
* General
    1. Initial sumbit
    2. xxx

