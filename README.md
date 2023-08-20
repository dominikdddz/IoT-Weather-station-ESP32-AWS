## Table of contents
* [General info](#general-info)
* [Technologies](#technologies)
* [Features](#features)
* [Architecture](#architecture)
* [Workspaces in Grafana](#workspaces-in-grafana)

## General info
The repository contains the code created for my engineering thesis. The topic of engineering thesis is: "Using FreeRTOS and a cloud platform to build a weather station"

The IoT Device use FreeRTOS system to managment sensors device and send data to Cloud Service. IoT project collect weather data (temperature, humidity and pressure) and send to AWS, where data are stored, process and visualization in Grafana service. 

## Technologies
Project is created with:
 * C/C++ Language
 * Arduino IDE
 * Amazon Cloud Service (AWS)
 * microcontroller ESP32
 * weather sensors BMP280 and DHT22
 * OLED display SSD1306

## Features
 - Measurement of temperature (BMP280), humidity and atmospheric pressure (DHT22)
 - uploading weather data to the AWS cloud 
 - OLED display (SSD1306) displaying actual weather data
 - saving data to memory if there is no access to the wifi network and sending them if the network is back
 - device location based on nearby wifi networks (google library)
 - all features on device working on FreeRTOS system

## Architecture
ESP32 connects to the *AWS IoT Core* using wifi and send weather data every few minutes. Data is send using *IoT MQTT Protocol* and after are saved in *AWS Timestream* database service. After that data are visualization in Grafana services using SQL querry to database

![Architecture IoT Weather Station](/Esp32_architecture.png?raw=true "Architecture IoT Weather Station")

## Workspaces in Grafana
First workspace show all connected IoT Weather Station, localization and logs. From this we may move to Weather Station Workspace
![Main workspace from Grafana](/grafanaMain.png?raw=true "Main workspace from Grafana")
Second Workspace show actualy weather data from selected Weather Station
![Second workspace from Grafana](/grafanaDevice.png?raw=true "Second workspace from Grafana")
