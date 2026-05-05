# Acoustic Camera Design Tradeoffs

* key characteristics
  - beam width (measured at 10dB from peak of main lobe)
  - side-lobe levels (measured as difference between main lobe peak and highest side-lobe)
* design objectives: narrow beam, low side-lobe levels
  - the larger number of mics, the lower the side-lobe levels
  - the larger the effective array size, the narrower the beam
* mic array characteristics
  - aperture response
  - spatial extent of array
  - type and number of transducers
  - transducer physical orientation and arrangement
* general array characteristics
  - larger mic arrays
    * larger arrays prioritize low-frequency captures and wide-area coverage
    * pros
      - improved spatial resolution
      - enhanced low-frequency (<1KHz) detection
        * larger mic spacing captures longer wavelengths
        * good for industrial noise
      - higher dynamic range
      - better noise suppression
      - increased localization accuracy
      - better in open/outdoor environments
        * due to better low-frequency directionality
    * cons
      - cumbersome, less portable, larger, heavier
      - more complex calibration
      - limits to effectiveness in broadband applications
        * due to problems with high-frequency grating lobes
  - more mics in the array
    * arrays with more mics optimize high-frequency detail and adaptive beamforming
      - but comes with increased computation and power costs
    * pros
      - enhances spatial resolution and high-frequency accuracy
        * reduced spatial aliasing
        * sharper beamforming and better noise rejection >=5KHz
      - better SNR
        * through constructive interference
      - works better in indoor/reverberant spaces
        * leverages phase differences for more precision source localization
    * cons
      - low-freq performance limited by mic spacing in smaller array size
      - more processing power and interconnect required
      - more power required
      - higher cost
  - frequency response and directionality
    * closer mic spacing improves high-frequency capture ability
      - provides wider BW before spatial aliasing occurs
        * spatial aliasing is an artifact of undersampling
    * wider spacing of mics enhances directionality at low freq
      - makes array more effective at blocking or localizing low-frequency sounds
  - spatial aliasing
    * if mics are spaced too far apart, high-frequency sounds can cause spatial aliasing
      - array can't distinguish between real and ghost sources
    * mic spacing should generally be less than half the wavelength of the highest freq of interest
  - SNR
    * optimal spacing can maximize SNR improvements
      - too wide or too narrow spacing can reduce an array's effectiveness across different frequencies
    * BW
      - narrow spacing provides broader usable freq range
      - wide spacing limits the array's BW -- only optimal for narrow freq range
  - mic spacing balances freq response, directionality, and spatial aliasing
* sound wavelength
  - a function of pressure and temperature
  - low freq sound (20-250Hz) wavelengths range from 18m to 1.4m
  - in human speech, low freq is where the energy is and higher freqs are where the information is located
* localizing sound in the 100-1000Hz range
  - ideal mic spacing: 170mm
  - spacing of 100-200mm balances aliasing risk and low-frequency coverage
* a dense array with 30+ mics and 100mm spacing provides good broadband coverage
  - but comes with cost in processing power and complexity
* a sparse array with 500mm spacing does well at low freq but fails above ~350Hz
* industrial leak detection centers around 500Hz
* broadband acoustic cameras typically use 100-150mm spacing and cover 200-1000Hz
  - they give up low-frequency precision for aliasing-free operation
* appears like many commercial products use arrays with spiral arms
  - many are 20-40cm in diameter
* processing
  - algorithm: MUSIC, DAMAS, ?
  - required resources: compute, memory, ?
* video overlay of sound energy
  - the amount of time needed to create the energy map will dictate the effective frame rate of the Acoustic Camera
  - do we need to capture a frame of video at the same time (start/end of block?) that the audio is captured and overlay on that frame?
    * should we try to keep time consistent between audio overlay and video display
  - need fast frame rate to capture transients -- means more compute demands
