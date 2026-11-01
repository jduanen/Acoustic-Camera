# Bring-up and test plan: front_end + mic_test

The mic side is tested with `mic_test`, a 120 × 120 mm centre cut of `mic_array`; the front
end is tested on the real `front_end` board. See "Mic test board" in
[`../SCHEMATIC_NOTES.md`](../SCHEMATIC_NOTES.md) for what the board carries and why.

Equipment: current-limited bench supply (5 V and 3.6 V), DMM, scope with ≥200 MHz bandwidth
and ground-spring probe tips, Vivado + JTAG cable (2 mm 2×7, J4), a GbE NIC on the host,
Wireshark, a phone/speaker for test tones.

## 1. front_end, no SOM fitted
1. Visual: U3/U5 exposed pads and paste, J2/CON1–4 alignment, D1 orientation.
2. Resistance to GND at TP1 (+5V), TP3 (+3V6), TP4 (+3V3_ETH), TP5 (+2V5_ETH),
   TP6 (+1V0_ETH): none below ~100 Ω.
3. Apply 5 V at J1, 200 mA limit. Expect TP1 5.0 V, TP3 ≈3.59 V, TP4 ≈3.32 V, TP5 2.50 V,
   TP6 1.00 V. Idle current is the PHY only.
4. Ripple at TP3/TP4 with the ground spring (TP2/TP10 ground): note the buck switching
   ripple, then repeat under load in step 2.5.

## 2. front_end with the AC7200 SOM
1. Raise the limit to 1.5 A, power up. D1 stays off until configuration (R19–R21 pull-downs).
2. Vivado Hardware Manager over J4: the XC7A200T is detected; program the bitstream.
3. D1: yellow during reset, then green. Solid red = PHY MDIO read-back fault.
4. GbE to the host: the link comes up at 1000 Mb/s only (the FPGA advertises 1000 only),
   the board answers ARP (`arping`), and the UDP stream appears in Wireshark while D1
   alternates green/blue.
5. Repeat the step 1.4 ripple check with the FPGA running.

## 3. mic_test on its own
Inject 3.6 V (100 mA limit) at TP5 "3V6 IN" (J1 side of the shunt R48), GND at TP1.
1. TP3 (+3V6_A, after the shunt), TP2 MIC_3V3 3.30 V, TP4 CLK_3V3 3.30 V.
2. Current: (TP5 − TP3) / 0.1 Ω. Expect roughly 30 mA for the 28 fitted mics plus the TCXO.
   The mics draw little until they are clocked.
3. Y1: 12.288 MHz at R15 / J1 pin 41. U110 and the arm buffers stay quiet because PDM_CLK
   comes from the FPGA.

## 4. Mechanical mate
1. Stack the boards on M4 standoffs (HENC1–4, the same 100 mm square on both boards), mated
   height 9.0 mm for J1/J2.
2. Check that the J1/J2 positioning bosses enter their holes and pin 1 meets pin 1. This settles
   the open boss-hole orientation item in SCHEMATIC_NOTES.
3. Check that nothing on front_end B.Cu touches mic_test F.Cu parts, and that the camera fits
   the HCAM holes and the Ø12 cut-out.

## 5. Stacked: clocks and signal integrity
Power front_end only; mic_test is fed through J2/J1.
1. TCXO_CLK at front_end TP7 (GND TP9): 12.288 MHz, clean edges.
2. PDM_CLK at front_end TP8: 3.072 MHz, 50 % duty.
3. ARM_02_CLK at mic_test TP6 (GND TP7). This is U99's input, at the end of the longest arm feed
   (156 mm, matched to mic_array). Check:
   - overshoot and undershoot within the CDCLVC1108's input ratings (−0.3 V / VDD+0.3 V);
   - monotonic edges through VIL/VIH, with no ledge or ring-back across the thresholds;
   - duty cycle and period jitter.
4. MIC_CLK_72 at mic_test TP8 (GND TP9). This is U73's CLK pin, at the end of the longest fitted
   mic clock (91 mm, matched). Same checks, against the IM72D128 clock input.
5. CLK_3V3 ripple at TP4 with all 12 + 12×(2–3) outputs switching (simultaneous-switching
   noise).
6. DATA_00 at mic_test TP10 (GND TP11): the mic 0/1 pair. Check the data transition timing
   against PDM_CLK at TP8 on the front_end. The FPGA constrains DATA to 8–89 ns after the edge.

## 6. Stacked: data path
Channel mapping on this board: the 28 fitted mics are U1, U2, U9–U11, U17, U18, U25, U26,
U33–U35, U41, U42, U49, U50, U57–U59, U65, U66, U73, U74, U81–U83, U89, U90 (mic index = U − 1). They
use 16 DATA lines: 00, 04, 05, 08, 12, 16, 17, 20, 24, 28, 29, 32, 36, 40, 41, 44. The other
32 lines have 10 kΩ pull-ups (R16–R47).

1. Every fitted channel responds to a tap or a voice; the unfitted channels on the same lines stay
   silent.
2. Cover one mic of a pair: only its slot drops. This confirms L/R (SEL) for each pair.
3. All 32 pulled-up lines read a constant full-scale value. This proves continuity of every DATA
   pin through J1/J2 and CON3.
   - A line reading near zero or noise is open.
   - A line that follows another line is shorted to it.
4. On DATA_05, 17, 29 and 41 only the L mic is fitted. The R slot floats, so a test build should
   add `PULLDOWN true` to DATA in the XDC.
5. Play a 1 kHz tone from 1 m on axis. Per-channel level spread should be within ±1 dB, the
   IM72D128's sensitivity tolerance.
6. Coarse direction check: with the fitted inner-ring positions from `test/phase4/array_xy.csv`,
   a delay-and-sum steer toward a tone source should peak at its bearing. There is no host-side
   UDP capture/analysis script in the repo yet; this step needs one.

## 7. Load
1. Fit R49 (51 Ω 1 W, 2512). It adds ≈65 mA on MIC_3V3, roughly what the 68 mics missing from
   this board would draw.
2. Check MIC_3V3 droop at TP2, VR1 temperature, and the +3V6 shunt current (TP5 − TP3). Then
   check front_end TP3 (+3V6 at the buck) to see the drop across J2/J1.

## 8. Soak
Run 30 min stacked and streaming at room temperature. Then repeat steps 5.3–5.5 and 6.5, and
compare against the earlier results.
