/*
    adc2pwm for Attiny402

    Measures VDD and outputs a 488Hz PWM signal with a duty cycle
    equal to 1 - (2.5V / VDD) \in [ 0% ... 50% ] for VDD \in [ 2.5V ... 5V ].
    That means VDD can be recovered as 2.5V / (1-PWM).
    
    d(PWM)/d(VDD) = 2.5 / VDD^2 \in 0.4 .. 0.1 / V  or ~ 1.6 clock ticks of TCA0 per mv
   
    Theory of operation:

    - VREF is set to 2.5 V, valid for Vdd = 3.0...5.5V, ±3% 0..85° , ±5% -45..125°
    - ADC is configured as follows:
        * single ended VREF as +input
        * VDD as Vref
        * sample duration 2, oversample 16x
            conversion time = ( 13 * 16 ) / 1MHz = 208us.
    - TCA0 is set to count to 16384 on the 8MHz clock 2.048 ms period or 488.28Hz
    - an update triggers the ADC event
    - Channel 0 is configured to PWM mode, inverted output
    - ADC IRQ handler copies the accumulated result to the TCA0 channel 0 pulse with register

    - optionally an HDLC encoded stream of uint16 values with Vdd [mV] is 
      sent out over the serial port at 115200 8N1
    - optionally PA1 is toggled whenever the ADC is ready, 
      useful for validating with an oscilloscope together with the PWM signal

    * TODO
        - deep sleep, delays for ADC when starting up

    Pinout (section 5)

    the 402 is an 8-pins version:
    PA3   WO0  PWM-out
    PA6   TXD  serial out
    PA1   toggled on ADC conversion ready for debug

*/
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>

// Configuration.  The compiler will optimize out unused branches.
// set either to zero to avoid outputting the signal on the respective pins.
enum {
    OUTPUT_PWM    = 1,   // PA3
    OUTPUT_SERIAL = 1,   // PA6
    OUTPUT_DEBUG  = 1,   // PA1: toggle on ADC result
};


// If you are pressed for space you can replace this with a hand-computed constant
// cf. 24.3.2.1.1 but then you don't have the correction from SIGROW.OSCxxERRyV
static void setbaud(USART_t* usart, uint32_t bd) {
    uint32_t clk = F_CPU; // no check for validity, used if OSCCFG is not set as expected
    switch (FUSE.OSCCFG & 0x3) { // Section 7.10.4.3
    case 1: // 16MHhz mainclock
         clk = 16000000 * (1024 + SIGROW.OSC16ERR3V); // should be the error at 3V
    }

    if (CLKCTRL.MCLKCTRLB & 1) {
        clk /= ((uint8_t[16]){ 2, 4, 8, 16, 32, 64, 1, 1, 6, 10, 12, 24, 48, 1, 1, 1})[(CLKCTRL.MCLKCTRLB >> 1) & 0xf];
    }

    clk <<= 2;  // * 4
    clk /= bd;
    clk >>= 10; //  / 1024

    usart->BAUD = (uint16_t)clk;
}

static uint8_t txbuf[16];
static uint8_t txhead = 0;
static uint8_t txtail = 0;
static inline int txbuf_empty() { return txhead == txtail; }

enum {
        HDLC_ESC   = 0x7d, // 0b0111 1101
        HDLC_FLAG  = 0x7e, // 0b0111 1110
        HDLC_ABORT = 0x7f, // 0b0111 1111
};

static inline void put_char(uint8_t c) {
    switch (c) {
    case HDLC_ESC:
    case HDLC_FLAG:
    case HDLC_ABORT:
            c ^= 0x20;
            txbuf[txhead++ % sizeof txbuf] = HDLC_ESC;
    }
    txbuf[txhead++ % sizeof txbuf] = c;
}
static inline void put_flag() { txbuf[txhead++ % sizeof txbuf] = HDLC_FLAG; }

ISR(USART0_DRE_vect) {
    if (OUTPUT_SERIAL) {
        if (txbuf_empty()) {
            USART0.TXDATAL = HDLC_FLAG;
        } else {
            USART0.TXDATAL = txbuf[txtail++ % sizeof txbuf];
        }
    }
}

ISR(ADC0_RESRDY_vect) {
    if (OUTPUT_DEBUG) {
        PORTA.OUTTGL = 1<<1;       // toggle PA1
    }

    uint16_t val = ADC0.RES;       // reading clears the interrupt flag

    if (OUTPUT_PWM) {
        TCA0.SINGLE.CMP0BUF = val;     // val has a 14 bit range
    }

    if (OUTPUT_SERIAL) {
        val = (2500*65536UL) / val;  // convert to Vdd[mv]

        // no race because the USART0_DRE can't run while this ISR runs
        put_char(val >> 8);
        put_char(val);
        put_flag();
    }
}

int main() {
    // The attiny402 only comes with a 16MHz main clock.
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 1);  // 11.5.2 mainclock div 2, for 8MHz 

    VREF.CTRLA   = VREF_ADC0REFSEL_2V5_gc;    // sec 19.5.1 2.500V
    VREF.CTRLB   = VREF_ADC0REFEN_bm;         // sec 19.5.2 keep on

    ADC0.CTRLA   = ADC_RESSEL_10BIT_gc;       // full resolution.
    ADC0.CTRLB   = ADC_SAMPNUM_ACC16_gc;      // 16 samples (4 extra bits, 14 bits total)   
    ADC0.CTRLC   = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV8_gc;   // Vref = Vdd, lower cap, run at 8/8 = 1MHz
    ADC0.MUXPOS  = ADC_MUXPOS_INTREF_gc;       // internal Vref as positive input (single ended)
    ADC0.EVCTRL  = ADC_STARTEI_bm;            // start on event
    ADC0.INTCTRL = ADC_RESRDY_bm;             // irq on result ready
    ADC0.CTRLA  |= ADC_ENABLE_bm;             // enable the ADC

    // Configure event routing: TCA overflow -> synch0 -> user1 -> ADC start
    EVSYS.SYNCCH0    = EVSYS_SYNCCH0_TCA0_OVF_LUNF_gc;  // sync channel 0 = TCA overflow
    EVSYS.ASYNCUSER1 = EVSYS_ASYNCUSER1_SYNCCH0_gc;     // ADC (= user 1) start on event sync channel 0

    // TCA0 count in 14 bits: 8MHz/16384 = 488Hz or 2.048ms, with CMP0 generating PWM
    TCA0.SINGLE.PER   = 0x3fff;         
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;  
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;

    if (OUTPUT_PWM) {
        PORTA.DIRSET   = 1<<3;             // set PA3 to be TCA0 WO0 signal from CMP0
        PORTA.PIN3CTRL = PORT_INVEN_bm;    // invert PA3 out (WO0) so we have PWM=1-(2.5V / Vdd)
    }

    if (OUTPUT_SERIAL) {
        setbaud(&USART0, 115200);
        USART0.CTRLB = USART_TXEN_bm;       // enable TX, add USART_ODME_bm to set open drain mode
        USART0.CTRLA = USART_DREIE_bm;      // enable TX register empty irq
        PORTA.DIRSET = 1<<6;                // set PA6 as TXD 
    }

    if (OUTPUT_DEBUG) {
        PORTA.DIRSET = 1<<1;                // PA1 for debug output
    }

    sei(); // enable interrupts

    for (;;)
        sleep_mode();
}





