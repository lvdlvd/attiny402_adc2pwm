# attiny402_adc2pwm
hello world bare metal attiny402 example with adc, pwm and uart

    Measures VDD and outputs a 488Hz PWM signal with a duty cycle
    equal to 1 - (2.5V / VDD) \in [ 0% ... 50% ] for VDD \in [ 2.5V ... 5V ].
    That means VDD can be recovered as 2.5V / (1-PWM).
    
    d(PWM)/d(VDD) = 2.5 / VDD^2 \in 0.4 .. 0.1 / V  or ~ 1.6 clock ticks of TCA0 per mv
   
    Theory of operation:

    - VREF is set to 2.5 V, valid for Vdd = 3.0...5.5V, ±3% 0..85° , ±5% -45..125°
    - ADC is configured as follows:
        * single ended VREF as +input
        * VDD as Vref
        * sample duration 2+0...15 (adsv), oversample 64x
            conversion time = ( xx * 64 ) / 1MHz = ... us.
    - TCA0 is set to count to 16384 on the 8MHz clock 2.048 ms period or 488.28Hz
    - an update triggers the ADC event
    - Channel 0 is configured to PWM mode, inverted output
    - ADC IRQ handler copies the accumulated result to the TCA0 channel 0 pulse with register

    - optionally an HDLC encoded stream of uint16 values with Vdd [mV] is 
      sent out over the serial port at 115200 8N1
    - optionally PA1 is toggled whenever the ADC is ready, 
      useful for validating with an oscilloscope together with the PWM signal

    Pinout (section 5)

    The 402 is an 8-pins version:

    PA3   WO0  PWM-out
    PA6   TXD  serial out
    PA1   toggled on ADC conversion-ready for debug
