﻿# DMA UART Project
# TM4C123 UART Communication with DMA

This repository provides a demonstration of UART communication on a TM4C123 microcontroller using Direct Memory Access (DMA).  DMA significantly improves the efficiency of data transfer by allowing the microcontroller to perform other tasks while the DMA controller handles the UART data transmission and reception.

## Overview

This project implements a basic UART communication system using DMA.  It covers the initialization of the UART peripheral, configuration of the DMA controller, and the setup of DMA transfers for both transmission and reception.  The code serves as a foundation for building more complex UART-based applications on the TM4C123.

## Features

*   **DMA-driven UART:** Utilizes DMA for efficient data transfer, minimizing CPU intervention.
*   **Configurable Baud Rate:**  The baud rate can be easily adjusted in the code. *(Note: The provided example uses 115200 bps.  Verify the calculation for your desired rate.)*
*   **8N1 Configuration:**  The UART is configured for 8 data bits, no parity, and 1 stop bit. *(This can be modified in the code.)*
*   **Clear Code Structure:** The code is organized into functions for initialization, DMA setup, and transfer initiation, promoting readability and maintainability.
*   **Example Buffers:**  Includes example transmit and receive buffers.

## Getting Started

### Prerequisites

*   Keil uVision IDE (or a similar IDE for ARM Cortex-M microcontrollers)
*   TM4C123 Device Family Pack (DFP) installed in Keil
*   A TM4C123G LaunchPad or compatible hardware

### Build Instructions

1.  Clone this repository.
2.  Open the project in Keil uVision.
3.  Ensure the correct TM4C123 device is selected in the project settings.
4.  Verify the include paths in the project options, specifically the path to the TM4C123 header files (e.g., within the Keil DFP installation directory).  *See the included screenshot for an example.*
5.  Build the project.

### Running the Example

1.  Connect your TM4C123 LaunchPad to your computer via USB.
2.  Load the compiled code onto the microcontroller using Keil's debugging tools.
3.  Use a terminal emulator (e.g., PuTTY, Tera Term) to connect to the virtual COM port created by the LaunchPad.
4.  Configure the terminal emulator for the same baud rate used in the code (default 115200 bps).
5.  The example code transmits a message via UART.  You can modify the `txBuffer` to change the transmitted data.  *(Note:  The receive functionality requires further implementation to process data received via DMA.)*

## Code Structure

*   `main.c`: Contains the main function, UART initialization, DMA initialization, and the main program loop.
*   `TM4C123.h`:  (Included) Header file containing register definitions for the TM4C123 microcontroller.

## Key Concepts

*   **Direct Memory Access (DMA):** A hardware mechanism that allows peripherals to transfer data to or from memory without CPU intervention.  This significantly reduces CPU load and improves system performance.
*   **UART (Universal Asynchronous Receiver/Transmitter):** A serial communication protocol commonly used for interfacing with other devices.
*   **Control Table (DMA):** A table in memory that contains the configuration for each DMA channel.
*   **Increment Mode (DMA):**  A DMA feature that automatically increments the source or destination address after each data transfer, simplifying buffer handling.

## Further Development

*   **Circular Buffers:** Implement circular buffers for both transmission and reception to handle data more efficiently and prevent data loss.
*   **Interrupts:**  Incorporate UART interrupts for events like transfer completion or error conditions.
*   **Error Handling:**  Add error handling to detect and manage potential issues during UART communication or DMA transfers.
*   **Data Processing:** Implement application-specific logic to process the data received via UART.

