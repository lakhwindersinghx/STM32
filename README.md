This is code is for major project COEN 6711 FF 2232 2023.
This project features a batter powered node which utilises STM32- f103c8t6 bluepill board and esp-8266
Node primarily collects enviornmental data(TEMP, PRESSURE, HUMIDITY, Light intensity) through BME280, PHOTORESISTOR and bridges to esp8266 which forwards the data to THINGSPEAK using mqtt establishing a PUB-SUB model
Project also features an interrupt based buzzer that triggers upon temperature exceeding 30
Primary focus of the project was to utilise as many peripherals provided by a microcontroller to showcase it's capability
This project was coded in STM32 CubeIDE and Arduino IDE (for esp8266)
Code for both BLUEPILL AND ESP8266 ARE PROVIDED.![node_main pinout](https://github.com/lakhwindersinghx/STM32/assets/116531009/5ee39d0b-2bf6-4028-8765-cef8d73fd66a)

![CIRCUIT](https://github.com/lakhwindersinghx/STM32/assets/116531009/6ba2700a-c7f0-420a-a0bf-98c074abe11a)
![THINGSPEAK_DATA](https://github.com/lakhwindersinghx/STM32/assets/116531009/d66a9b07-e0ee-4400-91be-5106cb8449f4)
