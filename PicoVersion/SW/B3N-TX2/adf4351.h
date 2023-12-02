#ifndef _ADF4351_H
#define _ADF4351_H
/*
 * adf4351.h
 *
 * Driver for adf4351 chip.
 *
 */ 


#define ADF_FREQ_MAX 4294967295        ///< Maximum Generated Frequency
#define ADF_FREQ_MIN  34385000         ///< Minimum Generated Frequency
#define ADF_PFD_MAX   32000000.0       ///< Maximum Frequency for Phase Detector
#define ADF_PFD_MIN   125000.0         ///< Minimum Frequency for Phase Detector
#define ADF_REFIN_MIN    10000000      ///< Maximum Reference Frequency
#define ADF_REFIN_MAX    32000000      ///< Maximum Reference Frequency
#define REF_FREQ_DEFAULT 25000000      ///< Default Reference Frequency


typedef struct 
{
    uint8_t gpio_sclk; // CLK pin
    uint8_t gpio_miso; // data pin
    uint8_t gpio_mosi; // dummy pin
    uint8_t gpio_cs; // dummy pin
    uint8_t gpio_ce; // chip enable
    uint8_t gpio_le; // load enable
    uint8_t gpio_ld; // lock detect
} pin_settings;

// you should not access variables that starts with an underscore, these are reserved for internal calculations
typedef struct
{
    uint32_t _reffreq; // always use ADF4351_set_ref_freq to access this variable, only set this directly if you know that you are doing
    double _cfreq; // this is the calculated frequency (useful for debugging when output frequency is wrong). This value is overwritten each time AD5941_set_freq() is called.
    int _outdiv; // the PLL output divider value for the current frequency. This value is overwritten each time AD5941_set_freq() is called.
    uint8_t RD2refdouble; // the PLL reference frequencer doubler flag. It is used to double the input reference frequency. Set to 0 or 1.
    int RCounter; // the PLL R counter value for the current frequency 10bit counter used to divide the ref freq for the PFD
    uint8_t RD1Rdiv2; // the PLL reference freuqency divider by 2. Sets a divide-by-2 relationship between Rcounter and PFD.
    uint8_t BandSelClock; // the PLL Band Select Clock Value
    int ClkDiv; // The PLL ref freq prescaler (divider) flag. Set to 0 or 1
    uint8_t Prescaler; // The PLL ref freq prescaler (divider) flag. Set to 0 or 1
    uint32_t _registers[6]; // The registers for PLL operation
    double _PFDFreq; // The PLL Phase Detect Freq  value for the current frequency. Can be changed if a new reference frequency is used.
    uint32_t ChanStep; // The channel step value. Can be directly changed to set a new frequency step from the defined steps[] array. You should not set an arbitrary value.
    uint32_t _Mod; // The PLL Mod value for the current frequency. this value is overwritten each time AD5941_set_freq() is called.
    int _Frac; //  The PLL Frac value for the current frequency. This value is overwritten each time sef() is called.
    uint16_t _N_Int; //  The PLL INT value for the current frequency. This value is overwritten each time sef() is called.
    uint8_t pwrlevel; // The power output level settings. Allowed values 0-4
    pin_settings pins; // pin settings for SPI and GPIO
    bool _spi_initialised; // SPI initalisation flag
    bool _ce_initialised; // CE initalisation flag
    bool _le_initialised; // LE initalisation flag
    bool _ld_initialised; // LD initalisation flag
    bool _enabled; // Device enabled flag
} ADF4351_cfg;

/*!
    gets the current register bitfield value, based on the start and length mask
    @param preg pointer to register
    @param start index of the bit of where to start
    @param len length number of bits to get
    @return bitfield value
*/
uint32_t ADF4351_get_register_bf(uint32_t *preg, uint8_t start, uint8_t len);

/*!
    modifies the register value based on a value and bitfield (mask)
    @param preg pointer to register
    @param start index of the bit to start the modification
    @param len length number of bits to modify
    @param value value to modify (value is truncated if larger than len bits)
*/
void ADF4351_set_register_bf(uint32_t *preg, uint8_t start, uint8_t len, uint32_t value);

/*!
    Writes the 4 byte register to ADF4351 via SPIs
    @param pcfg pointer to ADF4351 configuration struct
    @param reg 4 byte value to be written
    @return None
*/
void ADF4351_write_register(ADF4351_cfg *pcfg, uint32_t reg);

/*!
    calculates the greatest common denominator for two values
    helper function to calculate PLL values
    @param u value 1
    @param v value 2
    @return the greatest common denominator
*/
uint32_t ADF4351_gcd_iter(uint32_t u, uint32_t v);

/*!
    Enable ADF4351 by pulling CE pin high
    @param pcfg pointer to ADF4351 configuration struct
    @return None
*/
void ADF4351_enable(ADF4351_cfg *pcfg);

/*!
    Disable ADF4351 output by pulling CE pin low
    The chip is still powered up, so all settings are
    maintained in this mode.
    @param pcfg pointer to ADF4351 configuration struct
    @return None
*/
void ADF4351_disable(ADF4351_cfg *pcfg);

/*!
    Initialise the ADF4351 according to the configuration struct
    @param pcfg Pointer to the ADF4351 configuration struct
    @return None
*/
void ADF4351_initialise(ADF4351_cfg *pcfg);

/*!
    sets the incoming reference frequency to the ADF4351 chip,
    based on your OXCO or other freqeuncy reference source value.
    Checks for min and max settings
    @param pcfg Pointer to the ADF4351 configuration struct
    @param ref_freq new reference frequency
    @return success (0) or fail (-1)
*/
int ADF4351_set_ref_freq(ADF4351_cfg *pcfg, uint32_t ref_freq);

/*!
    sets the output frequency
    automatically calculates the PLL and other parameters
    returns false if the desired frequency does not match the
    calculated frequency or is out of range.
    @param pcfg pointer to ADF4351 configuration struct
    @param freq target frequency
    @return success (0) or fail (-1)
*/
int ADF4351_set_freq(ADF4351_cfg *pcfg, uint32_t freq);

#endif /* _ADF4351_H */