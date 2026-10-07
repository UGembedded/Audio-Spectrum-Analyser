rewrite this, so that i get gifs not images: ## STM32 Audio Spectrum Analyser

A real-time audio spectrum analyser built using an STM32F446RE microcontroller, an I²S digital microphone, and an ST7789 LCD. The system captures sound and processes it using a Fast Fourier Transform (FFT) to display its frequency content.

Three display modes—audio spectrum, peak frequency, and audio level—can be selected using the Nucleo board’s onboard user button. The firmware is written in C using STM32 HAL and CMSIS-DSP, with DMA handling audio data transfers.

<img width="720" height="381" alt="image" src="https://github.com/user-attachments/assets/0eb9693e-1520-4503-a5f6-e41e9bc0f434" />
<img width="720" height="381" alt="image" src="https://github.com/user-attachments/assets/dad4a9dd-877c-4d2d-818d-a7f39d014287" />
<img width="720" height="381" alt="image" src="https://github.com/user-attachments/assets/62e3a161-a6eb-498e-9fb0-595359f25d9c" />

## How it works
The I²S microphone captures sound and sends digital audio samples to the STM32F446RE. Direct Memory Access (DMA) transfers these samples into a memory buffer, allowing the processor to process completed blocks while audio capture continues. The firmware uses the CMSIS-DSP library to perform a Fast Fourier Transform (FFT) for the frequency spectrum and peak-frequency measurement, and calculates the root mean square (RMS) amplitude for the audio-level measurement. The results are displayed on the ST7789 LCD, with the onboard user button switching between the three modes.


## Part list

| Part               | Recommended model                            | Purpose                              |
| ------------------ | -------------------------------------------- | ------------------------------------ |
| Microcontroller    | **ESP32-S3-DevKitC-1-N8R8**                  | Samples and processes audio          |
| Microphone         | **SPH0645LM4H I²S MEMS microphone breakout** | Supplies digital audio samples       |
| Display            | **2.4-inch ILI9341 SPI TFT, 240 × 320**      | Displays spectrum bars and frequency |
| Power/programming  | Data-capable USB-C cable                     | Powers and programs the ESP32        |

## Pin layout
<img width="682" height="680" alt="pin_layout" src="https://github.com/user-attachments/assets/8f570844-fe94-492a-a8c3-48000fd22e17" />

## Wiring diagram
<img width="3200" height="1880" alt="image" src="https://github.com/user-attachments/assets/8db0639b-d630-47bf-a9b6-867a4c65c52b" />

## Code logic
The firmware configures the STM32 peripherals and captures microphone audio over I²S at a configured sample rate of 32 kHz. DMA fills the audio buffer, while callbacks flag each completed half for processing in the main loop.

MIC_ProcessBlock() extracts the microphone samples, removes their DC offset, calculates the root mean square (RMS) audio level, and applies a Hann window before performing a 256-point Fast Fourier Transform (FFT). The results produce 16 spectrum bars and an estimated dominant frequency, with smoothing and silence detection to stabilise the readings.

The LCD functions draw the selected display mode over SPI. The onboard button switches between spectrum, peak-frequency, and audio-level views, while UART provides diagnostic readings. Audio level is expressed in dBFS (decibels relative to digital full scale), rather than calibrated sound pressure level.

## tests


250 Hz

<img width="720" height="405" alt="250hz_test" src="https://github.com/user-attachments/assets/c758a905-8ac1-4119-beea-eb939f950a06" />






500 Hz

<img width="720" height="405" alt="image" src="https://github.com/user-attachments/assets/e6d0faad-2a6c-4bdc-bab9-3b88c57e434a" />




1 kHz

<img width="720" height="405" alt="image" src="https://github.com/user-attachments/assets/69542c11-75b4-4f98-863c-70e38b7176c2" />




2 kHz

<img width="720" height="405" alt="image" src="https://github.com/user-attachments/assets/c38c0239-b925-4ba4-8558-7cc94d34d778" />




4 kHz

<img width="720" height="405" alt="image" src="https://github.com/user-attachments/assets/628d2fd1-233f-46b5-896e-a69f70894201" />




8 kHz

<img width="720" height="405" alt="image" src="https://github.com/user-attachments/assets/96a9f41e-d537-4af2-bd80-aedc21631289" />
