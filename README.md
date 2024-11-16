
# Mobile measurement station





## Specification

- microcontoler STM32L476
- 2x DFRobot HR8833 
- Chassis Rectangle 4WD
- Power Source:
   - 9V battery microcontroler
   - 6V battery engines

## Pinout
 ### Microcontoler   
 #### Engine control
- PA0 PWM1/TIM2
- PA6 PWM2/TIM3
- PA1 LOGIC1
- PA5 LOGIC2
 #### I2C
- PB8 SCL
- PB9 SCD
#### Remote Transmiter/Reiever
![image](https://github.com/user-attachments/assets/1ac823f6-c91b-4a6e-b7b0-fc51b023a784)
##### SPI/Radaio/Remote
- MISO --> PB_14
- MOSI --> PB_15
- SCK  --> PB_13
###### Receiver
- CS   --> PB_11
- CE   --> PB_1
###### Transmitter
- CS   --> PB_12
- CE   --> PB_2
- IRQ  --> X
##### KAmodLSM303C (akcelerometr)
- INT -> PA_7 


##### Buttons (Remote)
- PC-0
- PC-1
- PC-3
- PB-0

## Jumpers
- To enable board power by external source:
- connect positive of your power source to Vin (Vin range : 7V-12V)
- negative to ground
- move jumper J6 to E5V position

##BE WARE

When you use break points with nrf24 the module might start behaving irrationaly
For example: When reading data from receiver and using breakpoints it randomly stop reading received data and reads nonsense data
