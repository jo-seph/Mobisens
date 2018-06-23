# Mobile sensor

Mobile Sensor for mobile sensor for particle matter and NOx/CO Gas 
is meant as an open project for everyone

Hardware:
- Nova Fitness SDS011 Particle Matter Sensor PM2.5,PM10; 
- Temperature, Humidity, AirPressure BME280 °C,%,hPA; 
- GPS Sensor NEO-6M GPS; 
- NOx CO Breakout-Board CJMCU-4541 [Sensor MiCS-4541 NOx, CO]; 
- Display SSD1306 OLED
- USB Power Bank


Arduino IDE


Idea based on Airrohr Feinstaubsensor from 
- https://luftdaten.info/ 
- https://github.com/opendata-stuttgart/sensors-software/tree/master/airrohr-firmware


SDS011 Library [sds011_vers] with reading in Query Reporting Mode [Factory Default is active reporting mode]


NOx/CO Sensor
Interpreting the values from the MiCS-4514 on the CJMCU-4541 Breakout Board 
is inspired and based on work from
- Shawn Hymel https://github.com/ShawnHymel/MICS-4514_CO_and_NOx_Sensor_Breakout
- Roland Ortner http://myscope.net/auswertung-der-airpi-gas-sensoren/ 
- OpenairCologne https://github.com/OpenAirCgn/
- own Data Collections for calibration


points of development
-lorawan module
-SD-Card
-


still in working progress



