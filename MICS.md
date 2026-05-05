# Microphone Elements

* general characteristics
  - higher SNR mics provide greater directionality than lower SNR ones; see: https://audioxpress.com/article/microphone-array-beamforming-with-optical-mems-microphones
  - close sensitivity and phase matching between mics is critical for array performance
    * mismatched mics degrade beam pattern and increase side-lobe levels
    * factory-calibrated mics (e.g., Infineon IM69D120) are preferred for large arrays
  - PDM interface is dominant for MEMS arrays; I2S supports 2 mics per link; TDM scales to 16 per link

## Electret

Traditional analog capsule mics with built-in FET buffer. Still used in large professional acoustic cameras (e.g., gfai EVO AC Pro, B&K arrays) where SNR performance exceeds available MEMS options, but less practical for dense arrays due to wiring complexity, phantom power requirements, and cost per channel.

## MEMS

### sensiBel SBM100
  - optical MEMS
  - SNR (20-20KHz): 80 dBA
    * >10dB (8x) better SNR than capacitive MEMS mics
  - Saturation: 146 dBSPL
  - THD: <0.5%
  - Sensitivity: -46 dBFS (typ @ 1KHz)
  - Dynamic Range: 132 dB
  - Sample Rate: up to 192KHz
  - Clock freq: 1.5-12MHz
  - Digital Interface: PDM, I2S, 8-channel TDM

### Knowles SPH0645LM4H-B
  - digital MEMS
  - Power Supply: 1.8V (typ) <3.6V (max) @ 600uA (typ), <100mA (max)
  - Interface: I2S
  - Bottom Ported
  - onboard SigmaDelta converter and filter
  - Clock Freq: 3.072MHz (typ)
  - Sensitivity: 94dBSPL @ 1KHz: -26dBFS
  - SNR: 94dBSPL @ 1KHz A: 65 dBA
  - THD:
    * 94dBSPL @ 1KHz: 0.2-1%
    * 1% THD: 94-110 dBSPL
  - AOP: 10% THD @ 1KHZ, 120dBSPL
  - PSSR: 200mVpp sine @ 1KHz: -86dBA
  - PCM output, 18b precision, 24b format

### InvenSense ICS-52000
  - digital MEMS
  - Bottom Ported
  - on-board ADC, decimation, filtering, signal conditioning
  - 24b TDM interface
  - up to 16x per TDM link, with synchronous sampling
  - SNR: 65dBA
  - Sensitivity: -26dBFS, +/-1dB
  - Freq: 50-20KHz
  - PSSR: -89dBFS
  - AOP: 117dBSPL
  - Interface: SCK, SD, WS, WSO
  - Vdd: -0.3 to 3.63V (max)
  - startup time: 85ms@48KHz to 512ms@8KHz
  - Interface: 24b, twos complement, MSB first
    * each mic in daisy-chain emits in subsequent 32b slots

### InvenSense ICS-43434
  - digital MEMS
  - Bottom Ported
  - on-board ADC, decimation, filtering, signal conditioning
  - 24b I2S interface
  - SNR: 65dBA
  - Sensitivity: -26dBFS, +/-1dB
  - Freq: 60-20KHz
  - PSR (217Hz 100mVpp square wave): -100dBFS
  - AOP (10% THD): 120dBSPL
  - Sample Rate: 23-51.6KHz
  - Vdd: -0.3 to 3.63V (max) @ 490uA
  - startup time: 85ms@48KHz to 512ms@8KHz
  - Interface: I2S (and L/R control)
    * 24b, twos complement, MSB first
    * two mics per shared link (stereo)
    * must be 64 SCK cycles in each WS stereo frame
    * LR pin selects whether data is output on rising/falling edge of WS
  - Noise Floor (20-20KHz, A-weighted RMS): -90 dBFS
  - Group Delay (sound input to digital output): 2/fs secs

### InvenSense ICS-43432
  - Digital MEMS
  - Bottom Ported
  - on-board ADC, decimation, filtering, signal conditioning
  - 24b I2S interface
  - SNR: 65dBA
  - Sensitivity: -26dBFS, +/-1dB
  - Freq: 50-20KHz
  - AOP (10% THD): 120dBSPL
  - EIN: 29dBSPL
  - Dynamic Range
    * (EIN to AOP): 87dB
    * (EIN to FS): 91dB
  - PSR
    * 217Hz 100mVpp square wave: -80dBFS
    * 1KHz sine wave: -90dBFS
  - Noise Floor (20-20KHz A-weighted RMS): -91dBFS
  - Interface
    * 24b, twos complement, MSB first
    * two mics per shared link (stereo)
    * must be 64 SCK cycles in each WS stereo frame
    * LR pin selects whether data is output on rising/falling edge of WS
  - SCK Freq: 3.379MHz (max), 3.072MHz (typ)
  - WS Freq: 52.8KHz (max)
  - startup time: 85ms@48KHz to 512ms@8KHz
  - Vdd: -0.3 to 3.63V (max) @ 490uA
  - Group Delay (sound input to digital output): 2/fs secs
  - synchronize mics via WS signal

### Infineon IM69D120
  - Digital MEMS
  - Dynamic Range: 95dBA
  - Distortion: <=1% @ up to 118dBSPL
  - Freq Range: 28 Hz – 20 kHz
  - close matching for arrays
    * phase matching: +/-2deg
    * sensitivity matching: +/-1dB
  - on-board LNA, SigmaDelta ADC
  - factory calibrated
  - Latency: 6usec @ 1KHz
  - Interface: PDM, (Data, Select, Clock)
  - Pattern: omni
  - Sensitivity (1KHz, 94dBSPL): -26dBFS (typ), -27dBFS (min)
  - AOP (THD=10%): 120dBSPL
  - SNR (A-weighted, Fclk=3.072MHz): 69dBA
  - EIN (A-weighted, Fclk=3.072MHz): 25dBSPL
  - THD (1KHz): 94dBSPL=0.5% ... 120dBSPL=10:0%
  - Group Delay: 25Hz=70usec, 4KHz=1usec
  - Phase Response: 75Hz=19deg ... 3KHz=-1deg
  - Freq Range (3dB): 25-11KHz?
  - Power Supply
    * Vdd: 1.62-3.6V
    * Idd @ 3.072MHz: .98mA (typ), 1.3mA (max)
  - Fclk: 3.072MHz (typ)
  - startup time: 50ms

### Solid State System 3SM222KMT1KA-P (aka 3SM222)
  - Digital MEMS
  - on-board pre-amp, ADC
  - Top Ported
  - Interface: PDM
    * supports two mics on single link
    * L/R, Clock, Data
    * polarity: increasing sound pressure = increasing density of 1's
  - Sensitivity deviation: +/-1dB
  - Pattern: omni
  - Freq Range: 50 Hz – 10 kHz
  - Power Supply
    * Vdd: 1.6-3.6V
    * Isb: 550uA @ 1.8V, 850uA @ 3.6V
  - Sensitivity (1KHz 94dBSPL): -27 to -25dBFS
  - SNR: 64dBA
  - EIN: 30dBA
  - THD: <0.2% @ 94dBSPL, 1% @ 110dBSPL
  - AOP (10% THD @ 1KHz): 120dBSPL
  - Clock Freq: 1-4.8MHz
  - PSSR (1KHz 200mVpp sine wave): 60dBV/FS
  - PSR+N: -80dBFS(A)
  - has low-power mode (different specs)

### ST Micro MP23DB01HP
  - Digital MEMS
  - Bottom Ported
  - Pattern: omni
  - modes: sleep, low-power, performance
  - Power Supply:
    * Vdd: 1.6-3.6V, 1.8V (typ)
    * Idd: 880uA (perf mode @ Fc=3.072MHz)
  - sensitivity matched
  - Interface: PDM (L/R channel selector)
  - wakeup time: 10ms
  - Freq Response: 35-?Hz
  - Sensitivity (94dBSPL @ 1KHz): -42 to -40dBFS
  - SNR (94dBSPL @ 1KHz A-weighted): 65.5dBA (3.072MHz)
  - THD (94dBSPL @ 1KHz): 0.2%
  - AOP (94dBSPL @ 1KHz): 135dBSPL
  - PSR (100mVpp sine wave): -95dBFS

### ST Micro MP34DT05-A
  - Digital MEMS
  - Top Ported
  - Pattern: omni
  - Interface: PDM (L/R channel selector)
  - Power Supply:
    * Vdd: 1.6-3.6V, 1.8V (typ)
    * Idd: 650uA (typ)
  - AOP: 122.5dBSPL
  - Sensitivity: -29 to -23dBFS
  - SNR (94dBSPL @ 1KHz A-weighted): 64dBA
  - PSR (100mVpp sine wave): -90dBFS
  - Fclk: 1.2-3.25Mhz
  - wakeup time: 10ms
  - THD (1KHz): 94dBSPL=0.2%, 110dBSPL=0.7%, 120dBSPL=6% (all typ)
  - Freq Response: 35-10KHz?

### PUI Audio AMM-3738-B-R
  - Analog MEMS
  - Pattern: omni
  - on-board pre-amp
  - Sensitivity (1KHz): -38dB +/-1
  - Power Supply
    * Vdd: 1.5-3.6V, 2V (typ)
    * Idd @ 2V: 90uA
  - SNR (1KHz 94dB A-weighted): 62dB
  - Freq Range: 20-20KHz
  - TDH (94dB 1KHz): 0.5%
  - AOP (1KHz 10% THD): 125dB

### InvenSense (TDK) T3902
  - Digital MEMS
  - Bottom Ported
  - Pattern: omni
  - on-board ADC, PDM modulator
  - Interface: PDM (clk, data, select)
  - Power Supply
    * Vdd: 1.65-3.63V
    * Idd (Vdd=1.8V): 650uA (typ), 750uA (max)
  - Sensitivity (1KHz 94dBSPL): -33 to -31dBFS
  - SNR (20KHz BW, A-weighted): 64dBA
  - EIN (20KHz BW, A-weighted): 30dBASPL
  - Dynamic Range (EIN to AOP): 96dB
  - Freq Range: 36-?KHz?
  - THD (105dBSPL): 0.2-1.0%
  - PSR
    * 100mVpp square wave A-weighted: -94dBFS
    * 1KHz sine: -100dBFS
  - AOP (10% THD): 126dBSPL
  - FS Acoustic Level (0dBFS output): 126dBSPL
  - standard and low-power modes (different specs)
  - wakeup time: 20ms
  - Fclk (std): 1-3.3MHz

### INMP441
  - Digital MEMS
  - Interface: I2C
  - Bottom Port
  - PCM output, 18b precision, 24b format
  - Freq Response: 60Hz-15KHz
  - Sensitivity: -26dBFS @ 94dBSPL
  - SNR: 61dBA
  - AOP: 120 dBSPL
  - PSRR: -75 dBFS

### MSM261S4030H0
  - on Xiao ESP32-S3 Sense
  - Digital MEMS
  - PCM output, 18b precision, 24b format
  - Bottom Port
  - SNR: 63dBA
  - Sensitivity: -26dBFS @1KHz, 1Pa
  - Freq Response: 100Hz-10KHz
  - AOP: 120dBSPL
  - THD: <1% @ 100dBSPL, <10% @ 120dBSPL
  - PSR: -72dBFSA
