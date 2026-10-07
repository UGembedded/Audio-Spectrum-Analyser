# STM32 Audio Spectrum Analyser

A real-time audio spectrum analyser built using an STM32F446RE microcontroller, an I²S digital microphone, and an ST7789 LCD. The system captures sound and processes it using a Fast Fourier Transform (FFT) to display its frequency content.

Three display modes—**audio spectrum**, **peak frequency**, and **audio level**—can be selected using the Nucleo board’s onboard user button. The firmware is written in C using STM32 HAL and CMSIS-DSP, with DMA handling audio data transfers.

## Demonstration

### Audio spectrum

<img src="assets/audio_spectrum.gif" alt="Real-time audio spectrum demonstration" width="720">

### Peak frequency

<img src="assets/peak_frequency.gif" alt="Peak-frequency measurement demonstration" width="720">

### Audio level

<img src="assets/audio_level.gif" alt="Audio-level measurement demonstration" width="720">

## How it works

The I²S microphone captures sound and sends digital audio samples to the STM32F446RE. Direct Memory Access (DMA) transfers these samples into a memory buffer, allowing the processor to process completed blocks while audio capture continues.

The firmware uses CMSIS-DSP to perform an FFT for the frequency spectrum and peak-frequency measurement, and calculates the root mean square (RMS) amplitude for the audio-level measurement. The results are displayed on the ST7789 LCD, with the onboard user button switching between the three modes.

## Parts list

| Part | Model | Purpose |
| --- | --- | --- |
| Microcontroller board | NUCLEO-F446RE | Captures and processes digital audio |
| Microphone | Adafruit SPH0645LM4H I²S MEMS microphone breakout | Supplies digital audio samples |
| Display | Waveshare 2.4-inch ST7789 SPI LCD, 240 × 320 | Displays the spectrum and measurements |
| Power/programming | Data-capable USB cable for the Nucleo board | Powers and programs the STM32 |
| Connections | Jumper wires and breadboard | Connects the modules and distributes power |

## Pin layout

<img width="682" alt="STM32F446RE peripheral pin configuration" src="https://github.com/user-attachments/assets/8f570844-fe94-492a-a8c3-48000fd22e17">

## Wiring diagram

<img width="1000" alt="Audio spectrum analyser wiring diagram" src="https://github.com/user-attachments/assets/8db0639b-d630-47bf-a9b6-867a4c65c52b">

## Code logic

The firmware configures the STM32 peripherals and captures microphone audio over I²S at a configured sample rate of **32 kHz**. DMA fills the audio buffer, while callbacks flag each completed half for processing in the main loop.

`MIC_ProcessBlock()` extracts the microphone samples, removes their DC offset, calculates the RMS audio level, and applies a Hann window before performing a **256-point FFT**. The results produce **16 spectrum bars** and an estimated dominant frequency, with smoothing and silence detection to stabilise the readings.

The LCD functions draw the selected display mode over SPI. The onboard button switches between spectrum, peak-frequency, and audio-level views, while UART provides diagnostic readings.

Audio level is expressed in **dBFS** (decibels relative to digital full scale), rather than calibrated sound pressure level. Peak-frequency mode estimates the strongest frequency component rather than specifically tracking voice pitch.

## Frequency tests

The following demonstrations show the analyser responding to test tones at six frequencies.

### 250 Hz

<img src="assets/250hz_test.gif" alt="250 Hz frequency test" width="720">

### 500 Hz

<img src="assets/500hz_test.gif" alt="500 Hz frequency test" width="720">

### 1 kHz

<img src="assets/1khz_test.gif" alt="1 kHz frequency test" width="720">

### 2 kHz

<img src="assets/2khz_test.gif" alt="2 kHz frequency test" width="720">

### 4 kHz

<img src="assets/4khz_test.gif" alt="4 kHz frequency test" width="720">

### 8 kHz

<img src="assets/8khz_test.gif" alt="8 kHz frequency test" width="720">
