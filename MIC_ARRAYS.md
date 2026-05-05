# Microphone Array Configurations

The pattern of the microphone array used in acoustic camera applications can be 2D or 3D.

The distribution of mics in an array influences the achievable spatial resolution (aka the beamwidth) and the Maximum Side-Lobe Levels (MSL) (which is a measure of the ability of the array to reject sources that are off-beam).  Deconvolution methods (e.g., DAMAS) attempt to remove the properties of the array from beamforming results, but in practice, the array properties remain important in the quality of the results that can be provided.

One study (https://www.acoustics.asn.au/conference_proceedings/AAS2013/papers/p5.pdf) demonstrated that the Underbrink pattern outperforms other array patterns in both resolution and MSL (over the tested frequencies). This study showed that arrays based on multiple arms, with mics more evenly distributed over the array's area, tended to get the best resolution (with adequate MSLs for most of the area). Also, arrays with a high density of mics in the center of the array tended to get the best MSLs directly below the array, at the expense of array resolution and performance over a larger array area.

In general, for a given pattern, more mics improve noise rejection and directionality, providing better spatial resolution and better SNR. However, there are diminishing returns for doubling the number of mics in an array, and there's an increase in cost that grows linearly with the number of mics.

Irregular spacing is preferred to avoid spatial aliasing or side-lobe artifacts that come with uniform mic spacing.

Mics have to be closely matched in sensitivity and frequency response. Mismatched mics can degrade beam performance and reduce the directionality of the array.

TBD: look into spherical arrays

## 2D Array Patterns

### Regular Grid
  - 2D grid with mics at intersections
  - uniform spacing leads to predictable, but higher, side-lobe levels (artifacts)
    * this is due to periodic gaps in spatial sampling
    * this can mask weaker sources or create false/ghost sources
  - beam width is consistent across directions but wider compared to spiral arrays
    * this reduces resolution for closely spaced sources
  - prone to spatial aliasing at high frequencies
    * if mic spacing exceeds half of the wavelength -- violating Nyquist criterion
  - best suited for narrowband or mid-frequency applications that need uniform coverage
  - easier to construct than other geometries
  - better suited for planar (as opposed to 3D) measurements

### Archimedean Spiral
  - radius increases linearly with angle
    * 'r(theta) = a + b*theta'
  - parameters: min radius, max radius, number of turns, number of mics
  - mics are spaced so angular positions are evenly distributed along the whole spirale angle, and their radii increase linearly from the center outward
  - simple, single-arm spiral layout, relatively even distribution of mics from center outward

### Dougherty log-Spiral
  - logarithmic spiral, where radius increases exponentially with the angle
    * 'r(theta) = r_0*e^(y*theta)' where y is related to the spiral angle
  - mics are placed at equal arc-lengths along the spiral
    * not at equal angular intervals
  - arc length between mics is kept constant
    * results in denser distribution near the center and sparser at the edge
  - better spatial sampling at the center
    * improved performance for central sources and wideband applications
  - provides >10dB side-lobe suppression at high frequencies

### Arcondoulis Array
  - modified spiral array that adjusts the "squash" of the spiral in the x and y directions
    allows for elliptical or non-circular spiral shapes
  - mic density can be increased at the center by adjusting design parameters
  - parameters: min/max radii (r_0, r_max), spiral angle, number of mics, squash factors (e_x, e_y)
    * squash factors control the stretching of the spiral in x and y dims
  - can choose parameters that put more mics near the center, if so desired
  - flexible design that can emphasize central density or overall coverage
  - often used to improve main-/side-lobe characteristics for sources near the center of the array

### Underbrink
  - patented
  - multi-arm logarithmic spiral array, several arms radiating from the center
  - each arm is a log-spiral and mics are distributed along each arm
  - parameters: min/max radii (r_0, r_max), spiral angle, number of mics, number of arms
  - mics rotated equally around the origin
    * more uniform distribution of mics across the array aperture -- especially the outer regions
  - high spatial resolution and good side-lobe performance across a wide area
    * suitable for both central and off-axis source localization
  - considered best all-around performance among spiral arrays

### Bruel&Kjaer Spiral
  - patented
  - designed to be easily assembled/disassembled
  - two concentric hoops with mics located on spokes between the hoops
  - spokes are placed at an angle to the inner hoop, and mics have a non-uniform spacing along the spoke
  - parameters: number of spokes, number of mics per spoke, spoke angle, and distribution of mic locations along the spoke
  - looks like spokes come off the inner hoop at a tangent 
  - mics are distributed with the same spacing on each spoke

## 3D Array Patterns

### Spherical Arrays
  - mics distributed uniformly on a sphere surface (e.g., Lebedev grid, t-design points)
  - provides full 4π steradian coverage — elevation and azimuth simultaneously
  - mathematical framework: Higher-Order Ambisonics (HOA)
    * sound field decomposed into Spherical Harmonics (SH) up to order N
    * order N requires (N+1)² mics minimum: order 4 = 25 mics, order 7 = 64 mics, order 9 = 100 mics
    * higher order → narrower beams, better spatial resolution
    * HOA is the standard framework for processing spherical arrays; used in VR/XR spatial audio, broadcast, and research (Eigenmike em32, mh Acoustics)
  - parameters: sphere radius, SH order N, grid scheme (Lebedev, Gaussian, t-design)
  - radius sets frequency range: smaller radius → higher usable frequency before spatial aliasing

### Cylindrical Arrays
  - mics arranged on the surface of a cylinder (rings stacked along the axis)
  - strong azimuthal resolution; elevation resolution limited by cylinder height
  - good for horizontal-plane localization in vertically elongated environments
  - Cylindrical Harmonics analog of HOA exists for cylindrical geometry

### Tetrahedral / Platonic Solid Arrays
  - minimum of 4 mics required for 3D DoA estimation
  - tetrahedral geometry (e.g., Soundfield SPS200, Core Sound TetraMic) is the smallest practical 3D array
    * First-order Ambisonics (FOA / B-format); 4 mics, SH order 1
  - larger platonic solids (octahedron, icosahedron) support higher-order SH decompositions
  - dual-layer icosahedral arrays (e.g., Zylia ZM-1, 19 mics) offer 3rd-order Ambisonics at low cost

### Nested / Concentric 3D Arrays
  - multiple spherical shells at different radii
  - inner shell handles high frequencies (avoids aliasing); outer shell extends low-frequency range
  - near-field and far-field source distinction becomes possible via range estimation
