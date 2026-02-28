Source code and STM32CubeIDE project files for the Battery Management System (BMS) firmware. This folder includes custom C drivers for the LTC6812 multi-cell battery monitor, SPI communication protocols, and FATFS integration for real-time onboard SD card data logging.

You can access the code here:
https://github.com/oneamongMANY775/BMS.git


**Firmware Architecture Overview**
The BMS firmware is built on a layered architecture using the **STM32CubeIDE** toolchain for the STM32F4-series microcontroller. It consists of:

* **Hardware Abstraction Layer (HAL):** Utilizes ST's STM32F4 HAL for robust peripheral management (SPI, DMA, CAN, ADC, and Timers).
* **Middleware (FatFs):** Integrates the FatFs generic FAT file system module for reliable real-time data logging to an onboard SD card.
* **Custom Driver Layer:** Features dedicated, low-level C drivers (`LTC6812.c`, `LT_SPI.c`) tailored for the LTC6812 multi-cell battery monitor, handling ISO-SPI communication and master-slave telemetry.
* **Application Layer:** Manages deterministic task scheduling, fault detection, and file handling (`File_Handling.c`) to seamlessly bridge battery state acquisition with secure data storage.
