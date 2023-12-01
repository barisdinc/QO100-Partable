#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <SPI.h>

#define ADF4351_LE 6
#define LOCK_DETECT 8
LiquidCrystal lcd(9, 7, 5, 4, 3, 2 );


#if !(defined(ARDUINO_ARCH_AVR))
   #error "Unsupported architecture, select Arduino IDE > Tools > Board > Arduino AVR Boards > Arduino Uno."
#endif
#if(F_CPU != 16000000)
   #error "Unsupported clock frequency, Arduino IDE must specify 16MHz clock; alternate crystal frequencies may be specified with F_MCU."
#endif

#undef F_CPU
#define F_CPU 16000000//20007000  // Actual crystal frequency of 20MHz XTAL1, note that this declaration is just informative and does not correct the timing in Arduino functions like delay(); hence a 1.25 factor needs to be added for correction.
#ifndef F_MCU
#define F_MCU 16000000 //20000000  // 20MHz ATMEGA328P crystal
#endif


#define F_SAMP_TX 4800 //4810 //4805 // 4402 // (Design) ADC sample-rate; is best a multiple of _UA and fits exactly in OCR2A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization
#if(F_MCU != 20000000)
const int16_t _F_SAMP_TX = (F_MCU * 4800LL / 20000000);  // Actual ADC sample-rate; used for phase calculations
#else
#define _F_SAMP_TX  F_SAMP_TX
#endif

#define F_SAMP_PWM (78125/1)
#define F_SAMP_RX 62500

byte poscursor = 0;
byte line = 0;
byte memory,RWtemp, msg_no;
byte rtx_state = 0;
volatile uint16_t numSamples = 0;


uint32_t registers[6] =  {0x4580A8, 0x80080C9, 0x4E42, 0x4B3, 0xBC803C, 0x580005} ; 
int address,modif=0,WEE=0;
int lcd_key = 0;
int adc_key_in = 0;
int timer = 0,timer2=0;
unsigned int i = 0;

double RFout, REFin, INT, PFDRFout, OutputChannelSpacing, FRACF;
double RFoutMin = 35, RFoutMax = 4400, REFinMax = 250, PDFMax = 32;
unsigned int long RFint,RFintold,INTA,RFcalc,PDRFout, MOD, FRAC;
byte OutputDivider;
byte lock=2;
unsigned int long reg0, reg1;

void SetADF4351();

typedef void (*func_t)(void);
volatile func_t func_ptr;

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnTX     5
#define btnNONE   6

#define magn(i, q) (abs(i) > abs(q) ? abs(i) + abs(q) / 4 : abs(q) + abs(i) / 4) // approximation of: magnitude = sqrt(i*i + q*q); error 0.95dB
//#define MULTI_ADC  1
#define VOX_ENABLE       0//1   // Voice-On-Xmit which is switching the transceiver into transmit as soon audio is detected (above noise gate level)
static uint8_t vox_tx = 0;
static uint8_t vox_sample = 0;
static uint16_t vox_adc = 0;

char radio_mod[3][3] = {"CW ", "USB", "LSB"};

char cq_station = "TA7W" ;

const char m2c[] PROGMEM = "~ ETIANMSURWDKGOHVF*L*PJBXCYZQ**54S3***2**+***J16=/***H*7*G*8*90************?_****\"**.****@***'**-********;!*)*****,****:****";

char cw_msg[6][128] = { "CQ CQ CQ QO100 TEST TEST DE OH2UDS OH2UDS TEST TEST +", "RRR RRR TA1D TA1D DE OH2UDS - TNX FER CALL - MY NAME IS BARIS BARIS - UR RPT 599 599 BK", "RRR RRR TA1D TA1D DE OH2UDS - RIG HM - QTH KP20ID KP20ID - BK", "RRR DE OH2UDS - TNX 73 73 CUAGN T U E E", "73 73 73", "OH2UDS" };

uint8_t cw_msg_interval = 5; // number of seconds CW message is repeated
uint32_t cw_msg_event = 0;
uint8_t cw_msg_id = 0; // selected message

uint8_t wpm = 25;
static unsigned long ditTime = (1200 * 5/4)/wpm;

uint8_t delayWithKeySense(uint32_t ms){
  uint32_t event = millis() + ms;
  for(; millis() < event;){
//    wdt_reset();
//    if(inv ^ digitalRead(BUTTONS) || !digitalRead(DAH) || !digitalRead(DIT)){
//      for(; inv ^ digitalRead(BUTTONS);) wdt_reset();  // wait until buttons released  
//      return 1;  // stop when button/key pressed
//    }
  }
  return 0;
}

void switch_rxtx(uint8_t tx_enable){
        bitWrite (registers[4], 5, tx_enable);
        SetADF4351();
}


int cw_tx(char ch){    // Transmit message in CW
  char sym;
  for(uint8_t j = 0; (sym = pgm_read_byte_near(m2c + j)); j++){  // lookup msg[i] in m2c, skip if not found
    if(sym == ch){  // found -> transmit CW character j
//      wdt_reset();
      uint8_t k = 0x80; for(; !(j & k); k >>= 1); k >>= 1; // shift start of cw code to MSB
      if(k == 0) delay(ditTime * 4); // space -> add word space
      else {
        for(; k; k >>= 1){ // send dit/dah one by one, until everythng is sent
          switch_rxtx(1);  // key-on  tx
          if(delayWithKeySense(ditTime * ((j & k) ? 3 : 1))){ switch_rxtx(0); return 1; } // symbol: dah or dih length
          switch_rxtx(0);  // key-off tx
          if(delayWithKeySense(ditTime)) return 1;   // add symbol space
        }
        if(delayWithKeySense(ditTime * 2)) return 1; // add letter space
      }
      break; // next character
    }
  }
  return 0;
}


int cw_tx(char* msg){
  for(uint8_t i = 0; msg[i]; i++){  // loop over message
    //lcd.setCursor(0, 14); lcd.print(msg[i]);
    if(cw_tx(msg[i])) return 1;
  }
  return 0;
}

//#define log2(n) (log(n) / log(2))
uint8_t log2(uint16_t x){
  uint8_t y = 0;
  for(; x>>=1;) y++;
  return y;
}

#define _UA  600 //=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision


inline int16_t arctan3(int16_t q, int16_t i)  // error ~ 0.8 degree
{ // source: [1] http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
//#define _atan2(z)  (_UA/8 + _UA/44) * z  // very much of a simplification...not accurate at all, but fast
#define _atan2(z)  (_UA/8 + _UA/22 - _UA/22 * z) * z  //derived from (5) [1]   note that atan2 can overflow easily so keep _UA low
//#define _atan2(z)  (_UA/8 + _UA/24 - _UA/24 * z) * z  //derived from (7) [1]
  int16_t r;
  if(abs(q) > abs(i))
    r = _UA / 4 - _atan2(abs(i) / abs(q));        // arctan(z) = 90-arctan(1/z)
  else
    r = (i == 0) ? 0 : _atan2(abs(q) / abs(i));   // arctan(z)
  r = (i < 0) ? _UA / 2 - r : r;                  // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r;                        // arctan(-z) = -arctan(z)
}
volatile uint8_t vox = 0;
#define RX      8         //PB0    (pin 14)

#define MORE_MIC_GAIN   1       // adds more microphone gain, improving overall SSB quality (when speaking further away from microphone)
#ifdef MORE_MIC_GAIN
volatile uint8_t vox_thresh = (1 << 2);
#else
volatile uint8_t vox_thresh = (1 << 1); //(1 << 2);
#endif

enum dsp_cap_t { ANALOG, DSP, SDR };
const uint8_t ssb_cap = 1;
const uint8_t dsp_cap = SDR;

volatile uint8_t admux[3];
#define F_ADC_CONV (192307/2)  //was 192307/1, but as noted this produces clicks in audio stream. Slower ADC clock cures this (but is a problem for VOX when sampling mic-input simulatanously).


void adc_start(uint8_t adcpin, bool ref1v1, uint32_t fs)
{
  DIDR0 |= (1 << adcpin); // disable digital input 
  ADCSRA = 0;             // clear ADCSRA register
  ADCSRB = 0;             // clear ADCSRB register
  ADMUX = 0;              // clear ADMUX register
  ADMUX |= (adcpin & 0x0f);    // set analog input pin
  ADMUX |= ((ref1v1) ? (1 << REFS1) : 0) | (1 << REFS0);  // If reflvl == true, set AREF=1.1V (Internal ref); otherwise AREF=AVCC=(5V)
//  ADCSRA |= ((uint8_t)log2((uint8_t)(F_CPU / 13 / fs))) & 0x07;  // ADC Prescaler (for normal conversions non-auto-triggered): ADPS = log2(F_CPU / 13 / Fs) - 1; ADSP=0..7 resulting in resp. conversion rate of 1536, 768, 384, 192, 96, 48, 24, 12 kHz
  ADCSRA |= ((uint8_t)log2((uint8_t)(F_CPU / 10 / fs))) & 0x07;  // ADC Prescaler (for normal conversions non-auto-triggered): ADPS = log2(F_CPU / 13 / Fs) - 1; ADSP=0..7 resulting in resp. conversion rate of 1536, 768, 384, 192, 96, 48, 24, 12 kHz
  //ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  //ADCSRA |= (1 << ADSC);  // start ADC measurements
}

void timer2_start(uint32_t fs)
{  // Timer 2: interrupt mode
  ASSR &= ~(1 << AS2);  // Timer 2 clocked from CLK I/O (like Timer 0 and 1)
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  TCCR2A |= (1 << WGM21); // WGM21: Mode 2 - CTC (Clear Timer on Compare Match)
  TCCR2B |= (1 << CS22);  // Set C22 bits for 64 prescaler
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt TIMER2_COMPA_vect
  uint8_t ocr = ((F_CPU / 64) / fs) - 1;   // OCRn = (F_CPU / pre-scaler / fs) - 1;
  OCR2A = ocr;
}

void timer2_stop()
{ // Stop Timer 2 interrupt
  TIMSK2 &= ~(1 << OCIE2A);  // disable timer compare interrupt
  delay(1);  // wait until potential in-flight interrupts are finished
}


void timer1_start(uint32_t fs)
{  // Timer 1: OC1A and OC1B in PWM mode
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); // Clear OC1A/OC1B on compare match, set OC1A/OC1B at BOTTOM (non-inverting mode)
  TCCR1B |= (1 << CS10) | (1 << WGM13) | (1 << WGM12); // Mode 14 - Fast PWM;  CS10: clkI/O/1 (No prescaling)
  ICR1H = 0x00;
  ICR1L = min(255, F_CPU / fs);  // PWM value range (fs>78431):  Fpwm = F_CPU / [Prescaler * (1 + TOP)]
  //TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10); // Clear OC1A/OC1B on compare match, set OC1A/OC1B at BOTTOM (non-inverting mode)
  //TCCR1B |= (1 << CS10) | (1 << WGM12); // Mode 5 - Fast PWM, 8-bit;  CS10: clkI/O/1 (No prescaling)
  OCR1AH = 0x00;
  OCR1AL = 0x00;  // OC1A (SIDETONE) PWM duty-cycle (span defined by ICR).
  OCR1BH = 0x00;
  OCR1BL = 0x00;  // OC1B (KEY_OUT) PWM duty-cycle (span defined by ICR).
}

void timer1_stop()
{
  OCR1AL = 0x00;
  OCR1BL = 0x00;
}
uint8_t lut[256];
volatile uint8_t amp;
volatile uint8_t drive = 2;   // hmm.. drive>2 impacts cpu load..why?

volatile uint8_t quad = 0;
volatile uint8_t tx = 0;
enum mode_t { LSB, USB, CW, FM, AM };
volatile uint8_t mode = USB;

inline void _vox(bool trigger)
{
  if(trigger){
    tx = (tx) ? 254 : 255; // hangtime = 255 / 4402 = 58ms (the time that TX at least stays on when not triggered again). tx == 255 when triggered first, 254 follows for subsequent triggers, until tx is off.
  } else {
    if(tx) tx--;
  }
}





// #define FAST __attribute__((optimize("Ofast")))
// inline void FAST freq_calc_fast(int16_t df)  // note: relies on cached variables: _msb128, _msa128min512, _div, _fout, fxtal
// {
//   #define _MSC  0x10000
//   uint32_t msb128 = _msb128 + ((int64_t)(_div * (int32_t)df) * _MSC * 128) / fxtal;

//   uint16_t msp1 = _msa128min512 + msb128 / _MSC; // = 128 * _msa + msb128 / _MSC - 512;
//   uint16_t msp2 = msb128; // = msb128 % _MSC;  assuming MSC is covering exact uint16_t so the mod operation can dissapear (and the upper BB2 byte) // = msb128 - msb128/_MSC * _MSC;

//   //pll_regs[0] = BB1(msc);  // 3 regs are constant
//   //pll_regs[1] = BB0(msc);
//   //pll_regs[2] = BB2(msp1);
//   //pll_regs[3] = BB1(msp1);
//   pll_regs[4] = BB0(msp1);
//   pll_regs[5] = ((_MSC&0xF0000)>>(16-4))/*|BB2(msp2)*/; // top nibble MUST be same as top nibble of _MSC !  assuming that BB2(msp2) is always 0 -> so reg is constant
//   pll_regs[6] = BB1(msp2);
//   pll_regs[7] = BB0(msp2);
// }




inline int16_t ssb(int16_t in)
{
  static int16_t dc, z1;

  int16_t i, q;
  uint8_t j;
  static int16_t v[16];
  for(j = 0; j != 15; j++) v[j] = v[j + 1];
#ifdef MORE_MIC_GAIN
//#define DIG_MODE  // optimization for digital modes: for super flat TX spectrum, (only down < 100Hz to cut-off DC components)
#ifdef DIG_MODE
  int16_t ac = in;
  dc = (ac + (7) * dc) / (7 + 1);  // hpf: slow average
  v[15] = (ac - dc) / 2;           // hpf (dc decoupling)  (-6dB gain to compensate for DC-noise)
#else
  int16_t ac = in * 2;             //   6dB gain (justified since lpf/hpf is losing -3dB)
  ac = ac + z1;                    // lpf
  z1 = (in - (2) * z1) / (2 + 1);  // lpf: notch at Fs/2 (alias rejecting)
  dc = (ac + (2) * dc) / (2 + 1);  // hpf: slow average
  v[15] = (ac - dc);               // hpf (dc decoupling)
#endif //DIG_MODE
  i = v[7] * 2;  // 6dB gain for i, q  (to prevent quanitization issues in hilbert transformer and phase calculation, corrected for magnitude calc)
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 16) / 64 + (v[6] - v[8]); // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i / 2, q / 2);  // -6dB gain (correction)
#else  // !MORE_MIC_GAIN
  //dc += (in - dc) / 2;       // fast moving average
  dc = (in + dc) / 2;        // average
  int16_t ac = (in - dc);   // DC decoupling
  //v[15] = ac;// - z1;        // high-pass (emphasis) filter
  v[15] = (ac + z1);// / 2;           // low-pass filter with notch at Fs/2
  z1 = ac;

  i = v[7];
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i, q);
#endif  // MORE_MIC_GAIN

#ifdef CARRIER_COMPLETELY_OFF_ON_LOW
  _vox(_amp > vox_thresh);
#else
  if(vox) _vox(_amp > vox_thresh);
#endif
  //_amp = (_amp > vox_thresh) ? _amp : 0;   // vox_thresh = 4 is a good setting
  //if(!(_amp > vox_thresh)) return 0;

  _amp = _amp << (drive);
  _amp = ((_amp > 255) || (drive == 8)) ? 255 : _amp; // clip or when drive=8 use max output
  amp = (tx) ? lut[_amp] : 0;

  static int16_t prev_phase;
  int16_t phase = arctan3(q, i);

  int16_t dp = phase - prev_phase;  // phase difference and restriction
  //dp = (amp) ? dp : 0;  // dp = 0 when amp = 0
  prev_phase = phase;

  if(dp < 0) dp = dp + _UA; // make negative phase shifts positive: prevents negative frequencies and will reduce spurs on other sideband
#ifdef QUAD
  if(dp >= (_UA/2)){ dp = dp - _UA/2; quad = !quad; }
#endif

#ifdef MAX_DP
  if(dp > MAX_DP){ // dp should be less than half unit-angle in order to keep frequencies below F_SAMP_TX/2
    prev_phase = phase - (dp - MAX_DP);  // substract restdp
    dp = MAX_DP;
  }
#endif
  if(mode == USB)
    return dp * ( _F_SAMP_TX / _UA); // calculate frequency-difference based on phase-difference
  else
    return dp * (-_F_SAMP_TX / _UA);
}

#define MIC_ATTEN  0  // 0*6dB attenuation (note that the LSB bits are quite noisy)

// This is the ADC ISR, issued with sample-rate via timer1 compb interrupt.
// It performs in real-time the ADC sampling, calculation of SSB phase-differences, calculation of SI5351 frequency registers and send the registers to SI5351 over I2C.
static int16_t _adc;
void dsp_tx()
{ // jitter dependent things first
  ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  //OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
//  si5351.SendPLLRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  OCR1BL = amp;                        // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  int16_t adc = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH

//Serial.println(adc+512);
  int16_t df = ssb(adc >> MIC_ATTEN);  // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
//  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
Serial.println(df);
//registers[1] = 0;
//registers[1] |= df << 15; //Phase shift
//registers[1] |= MOD << 3;
//registers[1] |= registers[1] + 1 ; // set address to Register "001"
//bitSet (registers[1], 27); // Prescaler sur 8/9  
//TODO: send only register 1
//SetADF4351(); //send all registers to ADF4531 

// #ifdef CARRIER_COMPLETELY_OFF_ON_LOW
//   if(tx == 1){ OCR1BL = 0; si5351.SendRegister(SI_CLK_OE, TX0RX0); }   // disable carrier
//   if(tx == 255){ si5351.SendRegister(SI_CLK_OE, TX1RX0); } // enable carrier
// #endif

#ifdef MOX_ENABLE
  if(!mox) return;
  OCR1AL = (adc << (mox-1)) + 128;  // TX audio monitoring
#endif
}


ISR(TIMER2_COMPA_vect)  // Timer2 COMPA interrupt
{
  func_ptr();
//#define DEBUG 1  
#ifdef DEBUG
  numSamples++;
  if (numSamples==60000) 
  {
    Serial.print("x");
    numSamples=0;
  }
#endif
}

int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the buttons
  if (adc_key_in > 1000)lcd.blink();
 
  if (adc_key_in < 50)return btnLEFT;
  if (adc_key_in < 200)return btnUP;
  if (adc_key_in < 400)return btnSELECT;
  if (adc_key_in < 600)return btnDOWN;
  if (adc_key_in < 800)return btnTX;
  if (adc_key_in < 900)return btnRIGHT;

  return btnNONE;
}

void printAll ()
{
  //RFout=1001.10; // test
  lcd.setCursor(0, 0);
  //lcd.print("RF = ");
  if (RFint < 1000000) lcd.print(" ");
  if (RFint < 100000)  lcd.print(" ");
  lcd.print(RFint/1000);lcd.print(".");
//  Serial.println(RFint/1000);
  RFcalc=RFint-((RFint/1000)*1000);
  if (RFcalc<100)lcd.print("0");
  if (RFcalc<10)lcd.print("0");
  lcd.print(RFcalc);
//  Serial.println(RFcalc);
  lcd.print("MHz    ");
  lcd.print(msg_no,DEC);
  lcd.setCursor(0,1);
  if (WEE==0) {lcd.print("R");}
  else {lcd.print("W");}
  if (memory<10)lcd.print(" ");
  lcd.print(memory,DEC);
  lcd.print("   ");
  if  ((digitalRead(LOCK_DETECT)==1))lcd.print(" LOCKED ");
  else lcd.print(" NOLOCK ");
  //lcd.print(PFDRFout,DEC);
  lcd.print("CW");
  lcd.setCursor(poscursor,line);
}

void WriteRegister32(const uint32_t value)   //Programme un registre 32bits
{
  digitalWrite(ADF4351_LE, LOW);
  for (int i = 3; i >= 0; i--)          // boucle sur 4 x 8bits
  SPI.transfer((value >> 8 * i) & 0xFF); // décalage, masquage de l'octet et envoi via SPI
  digitalWrite(ADF4351_LE, HIGH);
  digitalWrite(ADF4351_LE, LOW);
}

void SetADF4351()  // Programme tous les registres de l'ADF4351
{ 
  Serial.println("Resgiters");
  for (int i = 5; i >= 0; i--)  // programmation ADF4351 en commencant par R5
  {
    WriteRegister32(registers[i]);
    Serial.print(i,DEC);
    Serial.print(" ");
    Serial.println(registers[i],HEX);
  }
}

// *************** SP ecriture Mot long (32bits) en EEPROM  entre adress et adress+3 **************
void EEPROMWritelong(int address, long value)
      {
      //Decomposition du long (32bits) en 4 bytes
      //trois = MSB -> quatre = lsb
      byte quatre = (value & 0xFF);
      byte trois = ((value >> 8) & 0xFF);
      byte deux = ((value >> 16) & 0xFF);
      byte un = ((value >> 24) & 0xFF);

      //Ecrit 4 bytes dans la memory EEPROM
      EEPROM.write(address, quatre);
      EEPROM.write(address + 1, trois);
      EEPROM.write(address + 2, deux);
      EEPROM.write(address + 3, un);
      }

// *************** SP lecture Mot long (32bits) en EEPROM situe entre adress et adress+3 **************
long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long quatre = EEPROM.read(address);
      long trois = EEPROM.read(address + 1);
      long deux = EEPROM.read(address + 2);
      long un = EEPROM.read(address + 3);

      //Retourne le long(32bits) en utilisant le shift de 0, 8, 16 et 24 bits et des masques
      return ((quatre << 0) & 0xFF) + ((trois << 8) & 0xFFFF) + ((deux << 16) & 0xFFFFFF) + ((un << 24) & 0xFFFFFFFF);
      }

static uint8_t pwm_min = 0;    // PWM value for which PA reaches its minimum: 29 when C31 installed;   0 when C31 removed;   0 for biasing BS170 directly
static uint8_t pwm_max = 128;  // PWM value for which PA reaches its maximum: 
//refresh LUT based on pwm_min, pwm_max
void build_lut()
{
  for(uint16_t i = 0; i != 256; i++)    // refresh LUT based on pwm_min, pwm_max
    lut[i] = (i * (pwm_max - pwm_min)) / 255 + pwm_min;
    //lut[i] = min(pwm_max, (float)106*log(i) + pwm_min);  // compressed microphone output: drive=0, pwm_min=115, pwm_max=220
}

uint8_t  first_run = 1;

//************************************ Setup ****************************************
void setup() {
  
  lcd.begin(16, 2);
  lcd.display();
  analogWrite(10,5);

  Serial.begin (115200);
  
  lcd.print("QO-100 CW Trans.");
  lcd.setCursor(0, 1);
  lcd.print(" TA7W / OH2UDS  ");
  poscursor = 0; line = 0; 
  delay(1000);


  pinMode(LOCK_DETECT, INPUT);
  pinMode(ADF4351_LE, OUTPUT);
  digitalWrite(ADF4351_LE, HIGH);
  SPI.begin();                          // Init SPI bus
  SPI.setDataMode(SPI_MODE0);           // CPHA = 0 et Clock positive
  SPI.setBitOrder(MSBFIRST);            // poids forts en tête

  if (EEPROM.read(100)==55){PFDRFout=EEPROM.read(20*4);}
  else {PFDRFout=25;}

  //if (EEPROM.read(101)==55){RFint=EEPROMReadlong(memory*4);}
  //else {RFint=700000;}
  RFint=2400000;

  RFintold=12345;
  RFout = RFint/100 ;
  OutputChannelSpacing = 0.06;

  WEE=0;  address=0;
  lcd.blink();
  printAll(); delay(500);

  Serial.println("started");

  MCUSR = 0;
  func_ptr = dsp_tx;
  build_lut();
  //TIMER2_COMPA_vect();
  ADMUX = (1 << REFS0);  // restore reference voltage AREF (5V)

/*
  timer1_start(F_SAMP_PWM);
  timer2_start(F_SAMP_RX);
  
  // disable external interrupts
  //PCICR = 0;
  //PCMSK0 = 0;
  //PCMSK1 = 0;
  //PCMSK2 = 0;
   adc_start(2, false, F_ADC_CONV*4); admux[2] = ADMUX;  // Note that conversion-rate for TX is factors more
*/   
} //setup

static uint32_t mdf = 0;
//*************************************Loop***********************************
void loop()
{
  
  RFout=RFint;
  RFout=RFout/1000;
  if ((RFint != RFintold)|| (modif==1) || first_run == 1) {
    first_run = 0;

    Serial.print("RFout=");Serial.print(RFout,DEC);Serial.print("\r\n");
    if (RFout >= 2200) {
      OutputDivider = 1;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    }
    if (RFout < 2200) {
      OutputDivider = 2;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    }
    if (RFout < 1100) {
      OutputDivider = 4;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    }
    if (RFout < 550)  {
      OutputDivider = 8;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 1);
    }
    if (RFout < 275)  {
      OutputDivider = 16;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    }
    if (RFout < 137.5) {
      OutputDivider = 32;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    }
    if (RFout < 68.75) {
      OutputDivider = 64;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    }

    INTA = (RFout * OutputDivider) / PFDRFout;
    MOD = (PFDRFout / 0.01);
    FRACF = (((RFout * OutputDivider) / PFDRFout) - INTA) * MOD;
    FRAC = round(FRACF);
    Serial.print("L");Serial.println(INTA,DEC);
    Serial.print("A");Serial.println(RFout,DEC);
    Serial.print("B");Serial.println(MOD,DEC);
    Serial.print("C");Serial.println(FRACF,DEC);
    //Serial.print("D");Serial.println(PFDRFout,DEC);

    registers[0] = 0;
    registers[0] = INTA << 15; // OK
    FRAC = FRAC << 3;
    registers[0] = registers[0] + FRAC;

    registers[1] = 0;
    registers[1] = MOD << 3;
    registers[1] = registers[1] + 1 ; // ajout de l'adresse "001"
    bitSet (registers[1], 27); // Prescaler sur 8/9

    bitSet (registers[2], 28); // Digital lock == "110" sur b28 b27 b26
    bitSet (registers[2], 27); // digital lock 
    bitClear (registers[2], 26); // digital lock

    bitWrite (registers[4], 5, 1);
    SetADF4351();
   
    //SetADF4351();
    RFintold=RFint;
    modif=0;
    printAll();  // Affichage LCD
  }
  lcd_key = read_LCD_buttons();  // read the buttons

  switch (lcd_key)               // Select action
  {
    case btnRIGHT: 
      poscursor++; 
      if (line == 0) {
        if (poscursor == 4 ) { poscursor = 5;  line = 0; };
        if (poscursor == 8 ) { poscursor = 15; line = 0; };
        if (poscursor == 16) { poscursor = 0; line =  1; };
      }
     if (line == 1) {
        if (poscursor == 1 ) {poscursor = 2;  line = 1; }
        if (poscursor == 3 ) {poscursor = 13; line = 1; }
        if (poscursor == 14) {poscursor = 0;  line = 0; }
      }  
      lcd.setCursor(poscursor, line);
      break;
      
    case btnLEFT: 
      poscursor--; 
      if (line == 0) {
        if (poscursor == 255) {poscursor = 13; line = 1;};
        if (poscursor == 1)   {poscursor = 0;  line = 0;}
      }
       if(line==1){
          if (poscursor==255) {poscursor = 15; line = 0;};
          if (poscursor==1)   {poscursor = 0;  line = 1;};
          if (poscursor==13)  {poscursor = 2;  line = 1;};
      }
      lcd.setCursor(poscursor, line);
      break;
      
    case btnUP:
      if (line == 0)
      { // RFoutfrequency
        //Serial.print(oldRFint,DEC);
        if (poscursor == 0) RFint = RFint + 1000000 ;
        if (poscursor == 1) RFint = RFint + 100000 ;
        if (poscursor == 2) RFint = RFint + 10000 ;
        if (poscursor == 3) RFint = RFint + 1000 ;
        if (poscursor == 5) RFint = RFint + 100 ;
        if (poscursor == 6) RFint = RFint + 10 ;
        if (poscursor == 7) RFint = RFint + 1 ;
        if (poscursor == 15 && msg_no < 9) msg_no++ ;
        if (RFint > 4400000)RFint = RFintold;
        Serial.print(RFint,DEC);
      }
      if (line == 1)
      { 
        if (poscursor == 2){ memory++; 
        if (memory==20)memory=0;
        if (WEE==0){RFint=EEPROMReadlong(memory*4); // lecture EEPROM et Affichage
           if (RFint>4400000) RFint=4400000; 
           } 
        }  
        if (poscursor==15){ 
        if( PFDRFout==10){PFDRFout=25;} //reglage FREF
        else if ( PFDRFout==25){PFDRFout=10;}
        else PFDRFout=25;// au cas ou PFDRF different de 10 et 25
        modif=1;  }
                    
      if( (poscursor==0) && (WEE==1))WEE=0;
      else if ((poscursor==0) && (WEE==0))WEE=1;                  
      }
        printAll();
      break; // fin bouton up

    case btnDOWN: //bas
      if (line == 0) {
        if (poscursor == 0) RFint = RFint - 1000000 ;
        if (poscursor == 1) RFint = RFint - 100000 ;
        if (poscursor == 2) RFint = RFint - 10000 ;
        if (poscursor == 3) RFint = RFint - 1000 ;
        if (poscursor == 5) RFint = RFint - 100 ;
        if (poscursor == 6) RFint = RFint - 10 ;
        if (poscursor == 7) RFint = RFint - 1 ;
        if (poscursor == 15 && msg_no>0) msg_no--;
        if (RFint < 34500) RFint = RFintold;
        if (RFint > 4400000)  RFint = RFintold;
        break;
      }

     if (line == 1)
      { 
        if (poscursor == 2){memory--; 
        if (memory==255)memory=19;
        if (WEE==0){RFint=EEPROMReadlong(memory*4); // lecture EEPROM et Affichage
           if (RFint>4400000) RFint=4400000;
           Serial.print(RFint,DEC);  
           } 
        } // fin poscursor =5 

       if (poscursor==15){ 
       if( PFDRFout==10){PFDRFout=25;} //reglage FREF
       else if ( PFDRFout==25){PFDRFout=10;}
       else PFDRFout=25;// au cas ou PFDRF different de 10 et 25
       modif=1;
       }
                   
       if( (poscursor==0) && (WEE==1))WEE=0;
       else if ((poscursor==0)&&(WEE==0))WEE=1;                          
      
       printAll();
      break; // fin bouton bas
      }

    case btnSELECT:
      do {
        adc_key_in = analogRead(0);      // Test release button
        delay(1); timer2++;        // timer inc toutes les 1 millisecondes
        if (timer2 > 600) { //attente 600 millisecondes
         if (WEE==1 || poscursor==15){ 
         if (line==1 && poscursor==15){ EEPROMWritelong(20*4,PFDRFout);EEPROM.write(100,55);} // ecriture FREF
         else if (WEE==1) {EEPROMWritelong(memory*4,RFint);EEPROM.write(101,55);}// ecriture RF en EEPROM à adresse (memoire*4)
          lcd.setCursor(0,1); lcd.print("  MEMORISATION  ");}
          lcd.setCursor(poscursor,line);
          delay(500);timer2=0;
          printAll();
        }; // mes

        } 
      while (adc_key_in < 900); // attente relachement
      break;  // Fin bouton Select

      
    case btnTX:
        adc_key_in = analogRead(0);
        //cw_tx(cw_msg[msg_no]);

Serial.println(registers[0], HEX);
Serial.println(registers[1], HEX);
//Serial.println(registers[2], HEX);
//Serial.println(registers[3], HEX);
//Serial.println(registers[4], HEX);
//Serial.println(registers[5], HEX);


        switch_rxtx(0);
        Serial.println("o");
        delay(3000);
        switch_rxtx(1);
        delay(3000);
//8004e21
    INTA = 138.4;//(432.5005 * 8) / 25;//138.4
    FRACF = 1000;//(((432.5005 * 8) / 25) - 138) * 2500;
    //FRAC = round(FRACF);

    registers[0] = 0;
    registers[0] = 4521984;//INTA << 15; // OK
    //FRAC = FRAC << 3;
    registers[0] = registers[0] + 999<<3;//8000;//1000<<3

    registers[1] = 0;
    registers[1] = 2500 << 3;
    registers[1] = registers[1] + 1 ; // ajout de l'adresse "001"
    bitSet (registers[1], 27); // Prescaler sur 8/9

    registers[0] = 0x00452BC0;
    registers[1] = 0x08004E21;//0x80080C9


    WriteRegister32(registers[0]);
    WriteRegister32(registers[1]);

        while (0)
        {
          if (mdf <= 100000000) mdf+=100000; else mdf = 0;
          registers[0] = 0x00452BC0;
          registers[1] = 0x08004E21;//0x80080C9
          //registers[1] |= mdf << 15;
          //registers[1] = 0;
          //registers[1] |= 0 << 15; //Phase shift
          //registers[1] |= MOD << 3;
          //registers[1] |= registers[1] + 1 ; // set address to Register "001"
          //bitSet (registers[1], 27); // Prescaler sur 8/9  
          //TODO: send only register 1
          //SetADF4351(); //send all registers to ADF4531 
          delay(1000);
          WriteRegister32(registers[0]);
          WriteRegister32(registers[1]);
        }
        delay(1000);
        Serial.print("TX :");Serial.println(rtx_state,DEC);
      break;

         
      while (adc_key_in < 900);
      break;

     case btnNONE: {
        break;
      };
      break;
  }

   do { adc_key_in = analogRead(0); delay(1);} while (adc_key_in < 900);
   delay (10);timer++;
   //Serial.println(timer,DEC);
   if (timer>1000){lcd.noBlink();timer=0;}





/*
    #ifdef VOX_ENABLE
      if((vox) && ((mode == LSB) || (mode == USB))){  // If VOX enabled (and in LSB/USB mode), then take mic samples and feed ssb processing function, to derive amplitude, and potentially detect cross vox_threshold to detect a TX or RX event: this is expressed in tx variable
        if(!vox_tx){ // VOX not active
    #ifdef MULTI_ADC
          if(vox_sample++ == 16){  // take N sample, then process
            ssb(((int16_t)(vox_adc/16) - (512 - AF_BIAS)) >> MIC_ATTEN);   // sampling mic
            vox_sample = 0;
            vox_adc = 0;
          } else {
            vox_adc += analogSampleMic();
          }
    #else
*/    
      func_ptr = dsp_tx;
      
      //dsp_tx();
          
/*    #endif
          //if(tx){  // TX triggered by audio -> TX
            //vox_tx = 1;
            //switch_rxtx(255);
            //for(;(tx);) wdt_reset();  // while in tx (workaround for RFI feedback related issue)
            //delay(100); tx = 255;
          //}
/*        } 
        else if(!tx){  // VOX activated, no audio detected -> RX
          switch_rxtx(0);
          vox_tx = 0;
          delay(32); //delay(10);
          //vox_adc = 0; for(i = 0; i != 32; i++) ssb(0); //clean buffers
          //for(int i = 0; i != 32; i++) ssb((analogSampleMic() - 512) >> MIC_ATTEN); // clear internal buffer
          //tx = 0; // make sure tx is off (could have been triggered by rubbish in above statement)
        }
        
      }
    #endif //VOX_ENABLE
*/
 }  //LOOP