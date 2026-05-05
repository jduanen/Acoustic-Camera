# Acoustic-Camera-Related Projects

* Ben Wang (2023)
  - https://benwang.dev/2023/02/26/Phased-Array-Microphone.html
  - https://github.com/kingoflolz/mic_host
  - 192-channel MEMS mic array
    * 24x radial arms with linear array of 8x mics, exponentially-spaced
    * 8x concentric circles
  - three types of HW: 24x mics boards, hub, and host
  - frequency-domain beamforming
  - two beamforming visualizations are implemented
    * 3D Near-Field Beamformer
    * 2D Far-Field Beamformer
  - converts PDM from MEMS mics and packetizes samples with an FPGA (50MHz)
    * sends packets to host (with GPU) via GBE
  - calibration
    * move noise source in front of the array
    * calculate cross-correlation between all pairs of mics (18K unique pairs)
      - FFT pair -> complex multiply together -> IFFT -> find peak
        * fused compute on GPU -- don't write intermediate results to memory
    * used gradient descent to optimize the positions of the source at each time step
      - PyTorch on GPU
      - loss function minimizes difference between the measured and ideal correlations
        * do this while trying to minimize the deviation of the mic positions from the inital positions
        * and minimize the jerk of the source trajectory
      - speed of sound is a parameter which is optimized to get the best model
        * speed of sound is a function of temperature and pressure
      - converged to reasonable solution for source and mic positions (and speed of sound)
        * took a few hundred iterations (a few secs) on the GPU

* Sonicam (2020)
  - https://hackaday.com/2020/08/10/acoustic-camera-uses-many-many-microphones/
  - https://hackaday.io/project/172053-acoustic-camera
  - https://course.ece.cmu.edu/~ece500/projects/s20-teame0/
  - Senior ECE project at CMU in 2020
  - 6x panels of 4x4 rectangular grids of MEMS mics
  - 96x mic array, FPGA, Host
  - TDK InvenSense mics
  - FPGA with mic array, ethernet to the Host
  - [rather large]

* acoular Python library
  - https://github.com/acoular/acoular
  - example of 64 mic beamforming map

* Kickstarter: The First Handheld Sound Camera for Everyone (2018)
  - https://www.kickstarter.com/projects/351002836/the-first-handheld-sound-camera-for-everyone
  - 64x MEMS mics, camera, display, ~4K euro
  - mic arrays
    * three concentric rings: ~20 mics per ring
  - quad-core ARM53 @ 1.2GHz, Linux, 1GB DRAM
  - Display: 800x480, 7", capacitive touch screen
  - Range: 10-24KHz
  - Size: 340x340x95mm
  - Sample Rate: 48KHz
  - Resolution: 24b
  - Camera: 320x240 @ 50fps or 640x480 @ 16fps, FOV = +/-38deg, global shutter
  - battery powered
  - up to 100 fps acoustic, up to 50 fps optical
  - recording, replay, export sounds/images/distances

* 1024 “Pixel” Sound Camera Treats Eyes To Real-Time Audio (2016)
  - https://hackaday.com/2016/07/01/1024-pixel-sound-camera-treats-eyes-to-real-time-audio/
  - 32x32 MEMS mic array, made of 4x 8x8 arrays

