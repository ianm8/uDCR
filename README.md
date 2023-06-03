# uDCR
Micro direct conversion receiver covering 80m, 40m, 20m bands. Although it is continuously tunable from 3.5MHz to 15MHz the three nominated bands are easy to get to using the band change function. The receiver has fairly good large signal handling capability and an effective AGC mechanism. The controls consist of an LED (for frequency readout and signal level) and rotary encoder (for frequency and volume changes) and a pushbutton (used to change frequency step, band and volume setting in conjunction with the rotary encoder).

# How does it work?
The signal at the antenna is capacitively coupled to a pair of back-to-back diodes to protect the rest of the circuit from very large signals. Then follows a band-pass filter with a lower frequency cutoff of about 4MHz and a higher frequency cut-off of about 15MHz. The switching mixer responds to the fundamental frequency tuned as well as the third harmonic where signals will be reduced by about 9 db. This means that tuning below about 5Mhz will potentially allow reception of signals at 15Mhz and below but at reduced sensitivity. In the range 3.5Mhz to 4MHz (the 80m amateur band) this corresponds to 10.5Mhz to 12MHz where there are few interfering signals. In practice the 80m band is a nice inclusion without the complexity of switching in an additional filter - even though receiving this band of frequencies is a compromise. From the 40m band and higher there is no issue because the third harmonic of the tuned frequency is well above the filter cut-off in addition to the 9db reduction in sensitivity.

After the filter is a low cost MMIC RF amplifier and a 50 Ohm PI attenuator with a combined gain of about 9db. This sets the noise figure to about 6db. More than sufficient to receive any signals in the HF spectrum. The mixer is a fast-switching bus multiplexer with a balanced output. The mixer is clocked by an MS5351M (SI5351 equivalent) which is configured to generate the tuned frequency by a XIAO RP2040 microcontroller. The reference oscillator is generated by a TCXO with a frequency of 27MHz. Because this is a TCXO, no calibration is required. R7 and R8 bias the mixer to half VCC to minimise distortion and maximise dynamic range. The mixer also biases the following audio amplifier which is implemented by a low noise op amp - in this case a TLV9162. Resistors R15 and R19 and capacitors C22 and C25 form a low-pass filter with a cut-off of about 4Khz. R13 and C20 also form a low-pass filter, which, in conjunction with the following notch filter and C19, provide an SSB filter bandwidth of about 3KHz

A headphone amplifier is implemented with an LM4875 which has a maximum gain of 20db, operating in bridge mode (so no DC blocking capacitor is required) and a voltage controlled gain/attenuation function. The volume and AGC is controlled by the microcontroller using PWM which is filtered by R20 and C26 to provide a DC voltage to the VOL input of the amplifier. An LM1117 5V regulator provides DC to all the devices except the 3.3V devices which are powered from the 3.3V regulator on the microcontroller board.

# Automatic Gain Control
The AGC is digitally controlled by the microcontroller. The audio signal is connected to one of the analog inputs of the uC which is converted to a number in the range 0 to 4095 (12 bits). First the DC offset is subtracted from the audio signal. This value is 3102 (ie 2.5/3.3 * 4095). In addtion an IIR high-pass filter function removes any residual DC offset. The value is now a digital signed representation of the audio signal. Now the absolute value of the signal is calculated to get the instantaneous peak value. For AGC what we need is a fast attack and slow decay of the gain (or attenuation) applied to the audio signal. To get the fast attack, if the instantaneous value is greater than the current peak value then the new peak value is set to the current instantaneous value. To get the slow decay, the peak value is multiplied by 512 and decremented by one on every pass. Thus it takes a couple of seconds for the peak value to decrease if the audio signal significantly decreases. The peak value is now used in a look-up table to obtain an attenuation value relative to the current volume setting and using PWM applied to the VOL pin of the LM4875. Thus for the current volume setting the audio gain remains relatively steady with increasing signal strength. In addition the peak value is used to set the brightness of the LED again using PWM, as an indication of signal strength.

# How to use it
There are only two controls. A push-button and a rotary encoder. By default the rotary encoder changes the frequency in steps of 1Khz. By momentarily pressing and releasing the button the frequency steps will toggle from 1Khz to 100Hz, to 10Hz and back to 1Khz. A long press of the button will change to the next higher band in order as follows: 40m, 20m, 80m, 40m, etc. The volume can be adjusted by holding the button in and rotating the encoder.

When the frequency changes the LED flashes the frequency as described in the code (loop1() function), that is, a long flash represents 5 and a shot flash represents one. The frequency is indicated to the nearest Khz. At other times the LED brightness reflects the signal strength.
