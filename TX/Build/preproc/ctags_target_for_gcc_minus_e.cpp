# 1 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"

//#include <avr/sleep.h>
# 4 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 2


# 5 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
//FUSES = { .low = 0xFF, .high = 0xD6, .extended = 0xFD };   // Fuse settings should be set at programming (Arduino IDE > Tools > Burn bootloader)



//#define log2(n) (log(n) / log(2))
uint8_t log2(uint16_t x){
  uint8_t y = 0;
  for(; x>>=1;) y++;
  return y;
}

enum dsp_cap_t { ANALOG, DSP, SDR };
const uint8_t ssb_cap = 1;
const uint8_t dsp_cap = SDR;

enum mode_t { LSB, USB, CW, FM, AM };
volatile uint8_t mode = USB;
volatile uint16_t numSamples = 0;

volatile int32_t freq = 14000000;
static int32_t vfo[] = { 7074000, 14074000 };
static uint8_t vfomode[] = { USB, USB };
# 35 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
inline int16_t arctan3(int16_t q, int16_t i) // error ~ 0.8 degree
{ // source: [1] http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
//#define _atan2(z)  (_UA/8 + _UA/44) * z  // very much of a simplification...not accurate at all, but fast

//#define _atan2(z)  (_UA/8 + _UA/24 - _UA/24 * z) * z  //derived from (7) [1]
  int16_t r;
  if(((q)>0?(q):-(q)) > ((i)>0?(i):-(i)))
    r = 600 /*=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision*/ / 4 - (600 /*=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision*//8 + 600 /*=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision*//22 - 600 /*=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision*//22 * ((i)>0?(i):-(i)) / ((q)>0?(q):-(q))) * ((i)>0?(i):-(i)) / ((q)>0?(q):-(q)) /*derived from (5) [1]   note that atan2 can overflow easily so keep _UA low*/; // arctan(z) = 90-arctan(1/z)
  else
    r = (i == 0) ? 0 : (600 /*=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision*//8 + 600 /*=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision*//22 - 600 /*=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision*//22 * ((q)>0?(q):-(q)) / ((i)>0?(i):-(i))) * ((q)>0?(q):-(q)) / ((i)>0?(i):-(i)) /*derived from (5) [1]   note that atan2 can overflow easily so keep _UA low*/; // arctan(z)
  r = (i < 0) ? 600 /*=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision*/ / 2 - r : r; // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r; // arctan(-z) = -arctan(z)
}




uint8_t lut[256];
volatile uint8_t amp;


volatile uint8_t vox_thresh = (1 << 2);



volatile uint8_t drive = 2; // hmm.. drive>2 impacts cpu load..why?

volatile uint8_t quad = 0;


inline int16_t ssb(int16_t in)
{
  static int16_t dc, z1;

  int16_t i, q;
  uint8_t j;
  static int16_t v[16];
  for(j = 0; j != 15; j++) v[j] = v[j + 1];

//#define DIG_MODE  // optimization for digital modes: for super flat TX spectrum, (only down < 100Hz to cut-off DC components)
  int16_t ac = in * 2; //   6dB gain (justified since lpf/hpf is losing -3dB)
  ac = ac + z1; // lpf
  z1 = (in - (2) * z1) / (2 + 1); // lpf: notch at Fs/2 (alias rejecting)
  dc = (ac + (2) * dc) / (2 + 1); // hpf: slow average
  v[15] = (ac - dc); // hpf (dc decoupling)
  i = v[7] * 2; // 6dB gain for i, q  (to prevent quanitization issues in hilbert transformer and phase calculation, corrected for magnitude calc)
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 16) / 64 + (v[6] - v[8]); // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = (((i / 2)>0?(i / 2):-(i / 2)) > ((q / 2)>0?(q / 2):-(q / 2)) ? ((i / 2)>0?(i / 2):-(i / 2)) + ((q / 2)>0?(q / 2):-(q / 2)) / 4 : ((q / 2)>0?(q / 2):-(q / 2)) + ((i / 2)>0?(i / 2):-(i / 2)) / 4) /* approximation of: magnitude = sqrt(i*i + q*q); error 0.95dB*/; // -6dB gain (correction)
# 98 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
  _amp = _amp << (drive);
  _amp = ((_amp > 255) || (drive == 8)) ? 255 : _amp; // clip or when drive=8 use max output
  amp = lut[_amp];

  static int16_t prev_phase;
  int16_t phase = arctan3(q, i);

  int16_t dp = phase - prev_phase; // phase difference and restriction
  //dp = (amp) ? dp : 0;  // dp = 0 when amp = 0
  prev_phase = phase;

  if(dp < 0) dp = dp + 600 /*=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision*/; // make negative phase shifts positive: prevents negative frequencies and will reduce spurs on other sideband





  if(dp > 600 /*=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision*//2 /*((filt == 0) ? _UA : (filt == 3) ?  _UA/4 : _UA/2)     //(_UA/2) // the occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to _UA/2).*/){ // dp should be less than half unit-angle in order to keep frequencies below F_SAMP_TX/2
    prev_phase = phase - (dp - 600 /*=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision*//2 /*((filt == 0) ? _UA : (filt == 3) ?  _UA/4 : _UA/2)     //(_UA/2) // the occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to _UA/2).*/); // substract restdp
    dp = 600 /*=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision*//2 /*((filt == 0) ? _UA : (filt == 3) ?  _UA/4 : _UA/2)     //(_UA/2) // the occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to _UA/2).*/;
  }

  if(mode == USB)
    return dp * ( 4800 /*4810 //4805 // 4402 // (Design) ADC sample-rate; is best a multiple of _UA and fits exactly in OCR2A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization*/ / 600 /*=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision*/); // calculate frequency-difference based on phase-difference
  else
    return dp * (-4800 /*4810 //4805 // 4402 // (Design) ADC sample-rate; is best a multiple of _UA and fits exactly in OCR2A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization*/ / 600 /*=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision*/);
}

// This is the ADC ISR, issued with sample-rate via timer1 compb interrupt.
// It performs in real-time the ADC sampling, calculation of SSB phase-differences, calculation of SI5351 frequency registers and send the registers to SI5351 over I2C.
static int16_t _adc;
void dsp_tx()
{ // jitter dependent things first

  int16_t adc; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  adc = 
# 133 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
       (*(volatile uint16_t *)(0x78))
# 133 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
          ;
  
# 134 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7A)) 
# 134 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        |= (1 << 
# 134 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                 6
# 134 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                     );
  //OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
//  si5351.SendPLLRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
# 145 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
  
# 145 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x8A)) 
# 145 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        = amp; // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  adc += 
# 146 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
        (*(volatile uint16_t *)(0x78))
# 146 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
           ;

# 147 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
(*(volatile uint8_t *)(0x7A)) 
# 147 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
      |= (1 << 
# 147 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
               6
# 147 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                   ); // causes RFI on QCX-SSB units (not on units with direct biasing); ENABLE this line when using direct biasing!!
  int16_t df = ssb(_adc >> 0 /* 0*6dB attenuation (note that the LSB bits are quite noisy)*/); // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  adc += 
# 149 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
        (*(volatile uint16_t *)(0x78))
# 149 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
           ;
  
# 150 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7A)) 
# 150 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        |= (1 << 
# 150 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                 6
# 150 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                     );
//  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
  adc += 
# 152 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
        (*(volatile uint16_t *)(0x78))
# 152 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
           ;
  
# 153 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7A)) 
# 153 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        |= (1 << 
# 153 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                 6
# 153 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                     );
  //_adc = (adc/4 - 512);

  _adc = (adc/4 - (512 - 32)); // now make sure that we keep a postive bias offset (to prevent the phase swapping 180 degrees and potentially causing negative feedback (RFI)
# 168 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
//  if(tx == 1){ OCR1BL = 0; si5351.SendRegister(SI_CLK_OE, TX0RX0); }   // disable carrier
//  if(tx == 255){ si5351.SendRegister(SI_CLK_OE, TX1RX0); } // enable carrier


}







# 179 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
extern "C" void __vector_7 /* Timer/Counter2 Compare Match A */ (void) __attribute__ ((signal,used, externally_visible)) ; void __vector_7 /* Timer/Counter2 Compare Match A */ (void) 
# 179 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                       // Timer2 COMPA interrupt
{
  Serial.print(".");
  //dsp_tx(); 
}

volatile uint8_t admux[3];

#pragma GCC pop_options

/*ISR (TIMER2_COMPA_vect  ,ISR_NAKED) {
asm("push r24         \n\t"
    "lds r24,  0\n\t"
    "sts 0xB4, r24    \n\t"
    "pop r24          \n\t"
    "reti             \n\t");
}*/

void adc_start(uint8_t adcpin, bool ref1v1, uint32_t fs)
{
  
# 199 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7E)) 
# 199 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
       |= (1 << adcpin); // disable digital input 
  
# 200 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7A)) 
# 200 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        = 0; // clear ADCSRA register
  
# 201 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7B)) 
# 201 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        = 0; // clear ADCSRB register
  
# 202 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7C)) 
# 202 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
       = 0; // clear ADMUX register
  
# 203 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7C)) 
# 203 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
       |= (adcpin & 0x0f); // set analog input pin
  
# 204 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7C)) 
# 204 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
       |= ((ref1v1) ? (1 << 
# 204 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                            7
# 204 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                                 ) : 0) | (1 << 
# 204 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                                                6
# 204 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                                                     ); // If reflvl == true, set AREF=1.1V (Internal ref); otherwise AREF=AVCC=(5V)
  
# 205 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7A)) 
# 205 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        |= ((uint8_t)log2((uint8_t)(16000000L / 13 / fs))) & 0x07; // ADC Prescaler (for normal conversions non-auto-triggered): ADPS = log2(F_CPU / 13 / Fs) - 1; ADSP=0..7 resulting in resp. conversion rate of 1536, 768, 384, 192, 96, 48, 24, 12 kHz
  //ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  
# 207 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7A)) 
# 207 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        |= (1 << 
# 207 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                 7
# 207 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                     ); // enable ADC
  //ADCSRA |= (1 << ADSC);  // start ADC measurements





}

void adc_stop()
{
  //ADCSRA &= ~(1 << ADATE); // disable auto trigger
  
# 219 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7A)) 
# 219 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        &= ~(1 << 
# 219 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                  3
# 219 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                      ); // disable interrupts when measurement complete
  
# 220 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7A)) 
# 220 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        |= (1 << 
# 220 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                 2
# 220 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                      ) | (1 << 
# 220 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                                1
# 220 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                                     ) | (1 << 
# 220 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                                               0
# 220 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                                                    ); // 128 prescaler for 9.6kHz



  
# 224 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7C)) 
# 224 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
       = (1 << 
# 224 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
               6
# 224 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                    ); // restore reference voltage AREF (5V)
}

void timer1_start(uint32_t fs)
{ // Timer 1: OC1A and OC1B in PWM mode
  
# 229 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x80)) 
# 229 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        = 0;
  
# 230 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x81)) 
# 230 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        = 0;
  
# 231 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x80)) 
# 231 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        |= (1 << 
# 231 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                 7
# 231 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                       ) | (1 << 
# 231 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                                 5
# 231 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                                       ) | (1 << 
# 231 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                                                 1
# 231 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                                                      ); // Clear OC1A/OC1B on compare match, set OC1A/OC1B at BOTTOM (non-inverting mode)
  
# 232 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x81)) 
# 232 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        |= (1 << 
# 232 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                 0
# 232 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                     ) | (1 << 
# 232 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                               4
# 232 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                                    ) | (1 << 
# 232 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                                              3
# 232 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                                                   ); // Mode 14 - Fast PWM;  CS10: clkI/O/1 (No prescaling)
  
# 233 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x87)) 
# 233 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
       = 0x00;
  
# 234 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x86)) 
# 234 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
       = ((255)<(16000000L / fs)?(255):(16000000L / fs)); // PWM value range (fs>78431):  Fpwm = F_CPU / [Prescaler * (1 + TOP)]
  //TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10); // Clear OC1A/OC1B on compare match, set OC1A/OC1B at BOTTOM (non-inverting mode)
  //TCCR1B |= (1 << CS10) | (1 << WGM12); // Mode 5 - Fast PWM, 8-bit;  CS10: clkI/O/1 (No prescaling)
  
# 237 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x89)) 
# 237 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        = 0x00;
  
# 238 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x88)) 
# 238 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        = 0x00; // OC1A (SIDETONE) PWM duty-cycle (span defined by ICR).
  
# 239 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x8B)) 
# 239 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        = 0x00;
  
# 240 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x8A)) 
# 240 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        = 0x00; // OC1B (KEY_OUT) PWM duty-cycle (span defined by ICR).
}

void timer1_stop()
{
  
# 245 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x88)) 
# 245 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        = 0x00;
  
# 246 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x8A)) 
# 246 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        = 0x00;
}

void timer2_start(uint32_t fs)
{ // Timer 2: interrupt mode
  
# 251 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0xB6)) 
# 251 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
      &= ~(1 << 
# 251 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                5
# 251 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                   ); // Timer 2 clocked from CLK I/O (like Timer 0 and 1)
  
# 252 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0xB0)) 
# 252 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        = 0;
  
# 253 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0xB1)) 
# 253 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        = 0;
  
# 254 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0xB2)) 
# 254 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
       = 0;
  
# 255 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0xB0)) 
# 255 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        |= (1 << 
# 255 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                 1
# 255 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                      ); // WGM21: Mode 2 - CTC (Clear Timer on Compare Match)
  
# 256 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0xB1)) 
# 256 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        |= (1 << 
# 256 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                 2
# 256 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                     ); // Set C22 bits for 64 prescaler
  
# 257 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x70)) 
# 257 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        |= (1 << 
# 257 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                 1
# 257 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                       ); // enable timer compare interrupt TIMER2_COMPA_vect
  uint8_t ocr = ((16000000L / 64) / fs) - 1; // OCRn = (F_CPU / pre-scaler / fs) - 1;
  
# 259 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0xB3)) 
# 259 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
       = ocr;
}

void timer2_stop()
{ // Stop Timer 2 interrupt
  
# 264 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x70)) 
# 264 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        &= ~(1 << 
# 264 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                  1
# 264 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                        ); // disable timer compare interrupt
  delay(1); // wait until potential in-flight interrupts are finished
}


uint16_t analogSampleMic()
{
  uint16_t adc;
  
# 272 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 __asm__ __volatile__ ("cli" ::: "memory")
# 272 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
               ;
  
# 273 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7A)) 
# 273 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        = (1 << 
# 273 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                7
# 273 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                    ) | (((uint8_t)log2((uint8_t)(16000000L / 13 / (192307/1)))) & 0x07); // hack: faster conversion rate necessary for VOX

  //if((dsp_cap == SDR) && (vox_thresh >= 32)) digitalWrite(RX, LOW);  // disable RF input, only for SDR mod and with low VOX threshold
  //si5351.SendRegister(SI_CLK_OE, TX0RX0);
  uint8_t oldmux = 
# 277 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                  (*(volatile uint8_t *)(0x7C))
# 277 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                       ;
  for(;!(
# 278 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
        (*(volatile uint8_t *)(0x7A)) 
# 278 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
               & (1 << 
# 278 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                       4
# 278 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                           ));); // wait until (a potential previous) ADC conversion is completed
  
# 279 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7C)) 
# 279 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
       = admux[2]; // set MUX for next conversion
  
# 280 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7A)) 
# 280 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        |= (1 << 
# 280 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                 6
# 280 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                     ); // start next ADC conversion
  for(;!(
# 281 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
        (*(volatile uint8_t *)(0x7A)) 
# 281 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
               & (1 << 
# 281 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                       4
# 281 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                           ));); // wait until ADC conversion is completed
  
# 282 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x7C)) 
# 282 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
       = oldmux;
  //if((dsp_cap == SDR) && (vox_thresh >= 32)) digitalWrite(RX, HIGH);  // enable RF input, only for SDR mod and with low VOX threshold
  //si5351.SendRegister(SI_CLK_OE, TX0RX1);
  adc = 
# 285 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
       (*(volatile uint16_t *)(0x78))
# 285 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
          ;
  
# 286 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 __asm__ __volatile__ ("sei" ::: "memory")
# 286 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
             ;
  return adc;
}


uint8_t txdelay = 0;
uint8_t semi_qsk = false;
uint32_t semi_qsk_timeout = 0;

void switch_rxtx(uint8_t tx_enable){
  
# 296 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x70)) 
# 296 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        &= ~(1 << 
# 296 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                  1
# 296 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                        ); // disable timer compare interrupt
  dsp_tx();
  if(tx_enable) 
# 298 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
               (*(volatile uint8_t *)(0x7C)) 
# 298 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                     = admux[2];
  if(tx_enable){ // tx
//      si5351.SendRegister(SI_CLK_OE, TX1RX0);
      
# 301 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
     (*(volatile uint8_t *)(0x88)) 
# 301 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
            = 0x80; // make sure SIDETONE is set at 2.5V
      if(mode != CW) 
# 302 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                    (*(volatile uint8_t *)(0x80)) 
# 302 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                           &= ~(1 << 
# 302 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                                     7
# 302 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                                           ); // disable SIDETONE, prevent interference during SSB TX
      
# 303 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
     (*(volatile uint8_t *)(0x80)) 
# 303 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
            |= (1 << 
# 303 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                     5
# 303 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                           ); // enable KEY_OUT PWM
    }
  
# 305 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0xB3)) 
# 305 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
       = ((16000000L / 64) / 4800 /*4810 //4805 // 4402 // (Design) ADC sample-rate; is best a multiple of _UA and fits exactly in OCR2A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization*/ ) - 1;
  
# 306 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)(0x70)) 
# 306 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
        |= (1 << 
# 306 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
                 1
# 306 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                       ); // enable timer compare interrupt TIMER2_COMPA_vect
}




void initPins(){
  digitalWrite(14 /*PC0/A0 (pin 23)*/, 0x0); // when used as output, help can mute RX leakage into AREF
  digitalWrite(15 /*PC1/A1 (pin 24)*/, 0x0);
  pinMode(14 /*PC0/A0 (pin 23)*/, 0x0);
  pinMode(15 /*PC1/A1 (pin 24)*/, 0x0);

}


static uint8_t pwm_min = 0; // PWM value for which PA reaches its minimum: 29 when C31 installed;   0 when C31 removed;   0 for biasing BS170 directly
static uint8_t pwm_max = 255; // PWM value for which PA reaches its maximum: 96 when C31 installed; 255 when C31 removed;

//refresh LUT based on pwm_min, pwm_max
void build_lut()
{
  for(uint16_t i = 0; i != 256; i++) // refresh LUT based on pwm_min, pwm_max
    lut[i] = (i * (pwm_max - pwm_min)) / 255 + pwm_min;
    //lut[i] = min(pwm_max, (float)106*log(i) + pwm_min);  // compressed microphone output: drive=0, pwm_min=115, pwm_max=220
}


void setup()
{
//     s4i5351.powerDown();  // disable all CLK outputs (especially needed for si5351 variants that has CLK2 enabled by default, such as Si5351A-B04486-GT)

  Serial.begin(115200);

  
# 339 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
 (*(volatile uint8_t *)((0x34) + 0x20)) 
# 339 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
       = 0;
  wdt_enable(
# 340 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino" 3
            8
# 340 "/home/baris/PROJECTS/QO100-Partable/TX/QO100_tx.ino"
                   ); // Enable watchdog
//  uint32_t t0, t1;
//  ADMUX = (1 << REFS0);  // restore reference voltage AREF (5V)
//  // disable external interrupts
//  PCICR = 0;
//  PCMSK0 = 0;
//  PCMSK1 = 0;
//  PCMSK2 = 0;

  initPins();
  delay(3000); // at least 40ms after power rises above 2.7V before sending commands
//  drive = 4;  // Init settings

  //if(abs((int32_t)F_XTAL - (int32_t)si5351.fxtal) > 50000){ si5351.fxtal = F_XTAL; }  // if F_XTAL frequency deviates too much with actual setting -> use default
//  si5351.iqmsa = 0;  // enforce PLL reset
  freq = vfo[0];
  mode = vfomode[0];

//  build_lut();

  Serial.print("BASLADIK");
}


void loop()
{
    //switch_rxtx(1);
}
