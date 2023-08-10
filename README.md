# IoT Weather Station 
The IoT Device use FreeRTOS system to managment sensors device and send data to Cloud Service. IoT project collect weather data (temperature, humidity and pressure) and send to AWS, where data are stored, process and visualization in Grafana service. 

Weather Station has LED display *Ssd1306* where display actual weather data, time and whether it's connected to Wifi network. Additional in offline mode data are save in memory and when he's again online send all data. Additional weather station use Google library to check own localization based on wifi networks near by.

## Architecture
ESP32 connects to the *AWS IoT Core* using wifi and send weather data every few minutes. Data is send using *IoT MQTT Protocol* and after are saved in *AWS Timestream* database service. After that data are visualization in Grafana services using SQL querry to database

![Architecture IoT Weather Station](/Esp32_architecture.png?raw=true "Architecture IoT Weather Station")

## Workspaces in Grafana
First workspace show all connected IoT Weather Station, localization and logs. From this we may move to Weather Station Workspace
![Main workspace from Grafana](/grafanav5.png?raw=true "Main workspace from Grafana")
Second Workspace show actualy weather data from selected Weather Station
![Second workspace from Grafana](/grafanav2.png?raw=true "Second workspace from Grafana")