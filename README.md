# Senso_ESASonde

build command: mbed-tools compile -m SENSORBOARD --custom-targets-json custom_target.json -t GCC_ARM

Datasheets:

Honeywell RSC Series

https://prod-edam.honeywell.com/content/dam/honeywell-edam/sps/siot/de-de/products/sensors/pressure-sensors/board-mount-pressure-sensors/trustability-rsc-series/documents/sps-siot-trustability-rsc-series-data-sheet-32321348-ciid-164408.pdf?download=false

MS5611

https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5611-01BA03%7FB3%7Fpdf%7FEnglish%7FENG_DS_MS5611-01BA03_B3.pdf%7FCAT-BLPS0036


Protokoll:

counter             // counter variable zu alle Datenpakete in richtiger Reihenfolge ankommen
id                  // message id 
timestamp          // Systemzeit des Microcontrollers
sensor1 value
sensor2 value
sensor3 value
sensor4 value
sensor5 value
sensor6 value
sensor7 value
temperatur value sensor1
temperatur value sensor2
temperatur value sensor3
temperatur value sensor4
temperatur value sensor5
temperatur value sensor6
temperatur value sensor7