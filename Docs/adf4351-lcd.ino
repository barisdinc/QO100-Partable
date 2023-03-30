#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <SPI.h>

#define ADF4351_LE 6
#define LOCK_DETECT 8
LiquidCrystal lcd(9, 7, 5, 4, 3, 2 );

byte poscursor = 0;
byte line = 0;
byte memory,RWtemp, msg_no;
byte rtx_state = 0;

uint32_t registers[6] =  {0x4580A8, 0x80080C9, 0x4E42, 0x4B3, 0xBC803C, 0x580005} ; // 437 MHz with REF 25 MHz
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

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnTX     5
#define btnNONE   6

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
  Serial.println(RFint/1000);
  RFcalc=RFint-((RFint/1000)*1000);
  if (RFcalc<100)lcd.print("0");
  if (RFcalc<10)lcd.print("0");
  lcd.print(RFcalc);
  Serial.println(RFcalc);
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
{ for (int i = 5; i >= 0; i--)  // programmation ADF4351 en commencant par R5
    WriteRegister32(registers[i]);
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
//************************************ Setup ****************************************
void setup() {
  lcd.begin(16, 2);
  lcd.display();
  analogWrite(10,5);

  Serial.begin (19200);
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

  if (EEPROM.read(101)==55){RFint=EEPROMReadlong(memory*4);}
  else {RFint=700000;}

  RFintold=12345;
  RFout = RFint/100 ;
  OutputChannelSpacing = 0.01;

  WEE=0;  address=0;
  lcd.blink();
  printAll(); delay(500);


} // Fin setup

//*************************************Loop***********************************
void loop()
{
  
  RFout=RFint;
  RFout=RFout/1000;
  if ((RFint != RFintold)|| (modif==1)) {
    Serial.print(RFout,DEC);Serial.print("\r\n");
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
    MOD = (PFDRFout / OutputChannelSpacing);
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

    bitWrite (registers[4], 5, 0);
    SetADF4351();
   
    SetADF4351();
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
        cw_tx(cw_msg[msg_no]);
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

 }  //LOOP
