# Circuits in PCBs

Each breakout board implements several essential circuits to ensure reliable operation of Sailbot’s electrical system. Below are the key circuits and their purposes:

---

## 1. Buck Converter Circuit

![Buck Converter](../images/Buck_Converter_Circuit.png)

This circuit steps down the main 12 V line on the boat to lower regulated voltages that power the different devices and sensors connected to a specific module/ECU.

---

## 2. Protection Circuit

![Protection Circuit](../images/Protection_Circuit.png)

To prevent any short circuits on the mainline from damaging sensitive or costly sensors, this circuit provides **OVP (Over-Voltage Protection), OCP (Over-Current Protection), and UVP (Under-Voltage Protection)**.  
The thresholds were calculated using the formulas provided in the IC’s datasheet.

---

## 3. High-Side Switching Circuit

![High Side Switch](../images/High_Side_Switch_Circuit.png)

To conserve power during the voyage, modules must be able to selectively power sensors and actuators on or off.  
This circuit provides a stable ground reference and allows switching using 3.3 V GPIO outputs.

---

## 4. CAN FD Transceiver

![CANFD Transceiver](../images/CANFD_Tranceiver_Circuit.png)

Supports the **CAN FD protocol** for high-speed data transfer across modules/ECUs.  
It converts digital CAN FD signals from the STM32U575ZiQ microcontroller’s internal CAN controller into a differential CAN bus signal.

---

## 5. CAN 2.0 Controller + Transceiver

![CAN Servo Circuit](../images/CAN_Servo_Circuit.png)

One of the servos on the boat operates on **CAN 2.0**.  
An external SPI-to-CAN 2.0 controller was implemented, paired with the same transceiver, to drive the trim tab servo.

---

## 6. RS232 Transceiver

![RS232 Transceiver](../images/RS232_Tranceiver_Circuit.png)

Many marine sensors follow standards such as **RS232** or **NMEA 0183**.  
The MAX232 transceiver converts UART signals into RS232 levels using a charge-pump capacitor, improving noise immunity and compatibility.

---

### Additional Notes
- Decoupling capacitors are included on all three power rails.  
- Trace widths were sized appropriately for current requirements.  
- Layout decisions ensured both solderability (hand-assembly) and signal integrity.

---
