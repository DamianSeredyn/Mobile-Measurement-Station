
# Mobile measurement station





## Specification

- microcontoler STM32L476
- 2x DFRobot HR8833 
- Chassis Rectangle 4WD


## Pinout
 ### Microcontoler   
 #### Engine control
- PA0 PWM1/TIM2
- PA6 PWM2/TIM3
- PA1 LOGIC1
- PA5 LOGIC2
#### Remote Transmiter/Reiever
![image](https://github.com/user-attachments/assets/1ac823f6-c91b-4a6e-b7b0-fc51b023a784)
##### SPI/Radaio/Remote
- MISO --> PB_14
- MOSI --> PB_15
- SCK  --> PB_13
- CS   --> PB_11
- CE   --> PB_1
- IRQ  --> X
##### Buttons (Remote)
- PC_1 send 1 (robot forward)
- PC_0 send 2 (robot left)
- PA_4 send 3 (robto right) 
- P send 4 (robot stop) //TO DO
