## STM32 Audio Spectrum Analyser

A real-time audio spectrum analyser built using an STM32F446RE microcontroller, an I²S digital microphone, and an ST7789 LCD. The system captures sound and processes it using a Fast Fourier Transform (FFT) to display its frequency content.

Three display modes—audio spectrum, peak frequency, and audio level—can be selected using the Nucleo board’s onboard user button. The firmware is written in C using STM32 HAL and CMSIS-DSP, with DMA handling audio data transfers.

<img width="720" height="381" alt="audio_spectrum_music" src="https://github.com/user-attachments/assets/0ca5cd0f-f0a0-4a8f-b69c-b868461ed4c8" />
<img width="720" height="381" alt="peak_frequency_clap" src="https://github.com/user-attachments/assets/54dc2335-f524-46a0-9a5e-10a635d4bd2c" />
<img width="720" height="381" alt="audio_level_music" src="https://github.com/user-attachments/assets/a961ba13-2cd6-490c-8f57-625aee69b184" />



## How it works
The I²S microphone captures sound and sends digital audio samples to the STM32F446RE. Direct Memory Access (DMA) transfers these samples into a memory buffer, allowing the processor to process completed blocks while audio capture continues. The firmware uses the CMSIS-DSP library to perform a Fast Fourier Transform (FFT) for the frequency spectrum and peak-frequency measurement, and calculates the root mean square (RMS) amplitude for the audio-level measurement. The results are displayed on the ST7789 LCD, with the onboard user button switching between the three modes.


## Part list
| Part              | Model                                              | Purpose                                      |
| ----------------- | -------------------------------------------------- | -------------------------------------------- |
| Microcontroller   | **STM32 NUCLEO-F446RE (STM32F446RET6)**            | Acquires audio, performs FFT/DSP, controls UI |
| Microphone        | **Adafruit SPH0645LM4H I²S MEMS microphone**        | Supplies digital audio samples over I²S      |
| Display           | **Waveshare 2.4-inch SPI TFT, 240 × 320**           | Displays spectrum, peak frequency and level  |
| Display controller| **ILI9341**                                         | Controls the TFT display over SPI             |
| User control      | **On-board B1 push button (PC13)**                  | Cycles between the three display modes        |
| Power/programming | **USB Mini-B data cable**                           | Powers and programs the NUCLEO-F446RE         |

## Pin layout
<img width="682" height="680" alt="pin_layout" src="https://github.com/user-attachments/assets/8f570844-fe94-492a-a8c3-48000fd22e17" />

## Wiring diagram
<img width="3200" height="1880" alt="image" src="https://github.com/user-attachments/assets/8db0639b-d630-47bf-a9b6-867a4c65c52b" />

## Code logic
The firmware configures the STM32 peripherals and captures microphone audio over I²S at a configured sample rate of 32 kHz. DMA fills the audio buffer, while callbacks flag each completed half for processing in the main loop.

MIC_ProcessBlock() extracts the microphone samples, removes their DC offset, calculates the root mean square (RMS) audio level, and applies a Hann window before performing a 256-point Fast Fourier Transform (FFT). The results produce 16 spectrum bars and an estimated dominant frequency, with smoothing and silence detection to stabilise the readings.

The LCD functions draw the selected display mode over SPI. The onboard button switches between spectrum, peak-frequency, and audio-level views, while UART provides diagnostic readings. Audio level is expressed in dBFS (decibels relative to digital full scale), rather than calibrated sound pressure level.

A single main.c file was used for the sake of simplicity when generating and testing the integration through STM32CubeMX, seperated files for different peripheral usage can still be implemented for readability and organization.

## tests


250 Hz

<img width="720" height="405" alt="250hz_test" src="https://github.com/user-attachments/assets/c758a905-8ac1-4119-beea-eb939f950a06" />






500 Hz

<img width="720" height="405" alt="500hz_test" src="https://github.com/user-attachments/assets/edcd4e00-3af7-4f20-9e49-477faaf57ec7" />





1 kHz

<img width="720" height="405" alt="1khz_test" src="https://github.com/user-attachments/assets/6a3b042d-a620-41df-87d8-afee60dcfff9" />





2 kHz

<img width="720" height="405" alt="2khz_test" src="https://github.com/user-attachments/assets/31b79b9d-6cc2-4e37-87af-88152f814d61" />





4 kHz

<img width="720" height="405" alt="4khz_test" src="https://github.com/user-attachments/assets/c82db346-1b1d-4c93-989d-8def905d63ad" />





8 kHz

<img width="720" height="405" alt="8khz_test" src="https://github.com/user-attachments/assets/4eca5ac1-26de-4de4-b374-005189889b63" />

