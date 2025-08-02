# **Electronic-Voting-Machine**
## _RFID based electronic voting machine using ATmega328p AVR microcontroller_

In many elections, the accuracy and security of vote counting are critical to ensure fair results. 
However, traditional paper-based voting systems are prone to errors, delays, and potential 
fraud. While electronic voting machines have improved the process, concerns around 
authentication and unauthorized access remain. This project addresses these challenges by 
developing a small-scale electronic voting machine that uses RFID technology for secure voter 
authentication. The ATmega328p-based system ensures that only authorized voters can 
participate, and each voter can cast only one vote according to the desire. The voting process 
is simple and fast, with buttons for candidate selection and an LCD display for feedback. 

The core of the system is the ATmega328p microcontroller, which is an 8-bit AVR 
microcontroller with 32K bytes in system programmable flash program memory, 1K bytes 
EEPROM and 2K bytes internal SRAM. This system interacts with RC522 RFID  reader 
module, which is used to authenticate voter access and the admin access to the system. 
Basically, this system can store the voters unique RFID address to the system memory and total 
registered voters in real-time during the registration process of the system. The administrator 
has a special RFID card that allows them to access restricted system functions such as starting 
the user registration process, starting the voting process, viewing the election results and 
resetting the system.

* ### Schematic diagram overview for the voting machine –  
![Schematic_Electronic Voting Machine_2024-10-29](https://github.com/user-attachments/assets/ba8bec43-211a-48a2-be22-cdc445995c2c)

This schematic diagram shown above represents the hardware connections between the 
ATmega328p microcontroller and peripheral devices used in the RFID-based electronic voting 
system. The ATmega328p is the core of the system, controlling all operations including RFID 
authentication, LCD display updates, buttons inputs and the data storing into the EEPROM of 
the microcontroller. The general input and output pins (GPIO pins) are used to interface the 
LED feedback indicators, buttons, buzzer (audio feedback) and the RC522 RFID reader 
module. The 16x2 LCD display is connected to the I2C module.

* ### PCB diagram for the voting machine -
[PCB_PCB_Electronic Voting Machine_final.pdf](https://github.com/user-attachments/files/20229145/PCB_PCB_Electronic.Voting.Machine_final.pdf)

* ### Project Demo captures - 
![Screenshot 2025-05-15 190644](https://github.com/user-attachments/assets/3db5df1d-cea1-46db-97ef-a4722fb1ca01)

https://github.com/user-attachments/assets/3c53fb6d-8a31-429a-b386-7d6e71e82732

* ### User guide of the electronic voting machine -
[User_guide.pdf](https://github.com/user-attachments/files/21251236/User_guide.pdf)

> [!TIP]
> To find out the admin ID number of the RFID card, the admin ID folder can be used and uploaded into the ATmega328p and then it will display the admin ID number on the LCD display. The main code admin ID should be replaced with your admin ID card unique number.

* ### Data sheet of the ATmega328p - 
[https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf).
