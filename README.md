# Portable_Function_Generator_and_Oscilloscope
Portable 3-Channel Function Generator &amp; Oscilloscope on an STM32, using low-level HAL functions (DMA, Timers, DAC) 


# STM32 Portable Function Generator & Oscilloscope

This project turns a single STM32 board into a 3-channel portable test instrument. It functions as both a 3-channel arbitrary waveform generator and a 3-channel oscilloscope, using the Arduino Serial Plotter for real-time visualization.

The code is ported from an STM32CubeIDE project to run on the Arduino (STM32 Core) framework, but it retains high-performance, low-level peripheral control.

## 📈 Features

### Function Generator
* **3 Simultaneous Channels:**
    * **Sine Wave (PA4):** Generated using **DAC + DMA + TIM6** for CPU-free operation.
    * **Square Wave (PA5):** Generated using **TIM2 (Output Compare Mode)**.
    * **Triangle Wave (PA6):** Generated using the **DAC's built-in hardware triangle wave generator** + **TIM7**.

### Oscilloscope
* **3-Channel Input (A0, A1, A2):**
    * Reads three analog inputs using the STM32's 12-bit ADC.
    * Designed to self-test the generator outputs.

### Visualization
* **6-Channel Plotting:** Plots all 3 generator values and all 3 oscilloscope inputs to the **Arduino Serial Plotter** simultaneously.
* **Real Voltage Scaling:** All data is converted to volts, and the plotter Y-axis is locked to a 0V-4V range for easy reading.

### External Hardware
* **Analog Filtering:** Includes a simple **RC Low-Pass Filter** to smooth the DAC's stepped sine wave output.
* **Variable Gain:** A **LM741 Op-Amp circuit** with a 10k potentiometer provides variable amplitude control for an output channel.

## 🛠️ Technologies Used

* **Hardware:** STM32 (e.g., Nucleo), LM741 Op-Amp, 10k Potentiometer
* **Software:** Arduino IDE (STM32 Core), STM32 HAL (C++)
* **Key Peripherals:** **DAC**, **ADC**, **TIMERS**, **DMA**, **GPIO**
