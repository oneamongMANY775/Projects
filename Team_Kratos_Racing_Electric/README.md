**Team Kratos Racing Electric** 

This project represents a complete vertical integration of embedded systems, covering both custom hardware design and high-reliability firmware. My work focused on migrating legacy systems to high-performance architectures to ensure safety and precision in a Formula Student racing environment.

### 📂 Folder Structure

#### [📁 Hardware](https://www.google.com/search?q=./Hardware)

This folder contains the PCB design and electrical integration files for the Battery Management System (BMS) and Electronic Control Unit (ECU).


**Microcontroller Migration:** Successfully transitioned the BMS from Atmega to **STM32**, enabling advanced debugging via **SWD** and **JTAG**.


**High-Precision Sensing:** Optimized PCB layouts with minimal trace lengths to ensure accurate voltage and temperature sensing for the battery pack.


**EMI/Noise Resilience:** Integrated differential **iso-SPI** for noise-resistant communication and balanced current traces to maintain signal integrity in high-EMI racing environments.



#### [📁 Software](https://www.google.com/search?q=./Software)

This folder contains the firmware and algorithms designed for deterministic control and battery safety.


**Custom Drivers:** Developed and ported specialized **STM32 libraries** for the **LTC6812** battery monitoring IC using a Master-Slave architecture.


**Deterministic Execution:** Designed a timer-based, **interrupt-driven algorithm** to ensure precise task execution timing for both the BMS and ECU.


**Scalability:** Maintained all firmware via **GitHub** with structured documentation to support team scaling and future iterations.




