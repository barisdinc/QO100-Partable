This folder contains Kicad schematic,PCB and gerber files for the qo-100 sdr transceiver

Hardware development steps will be as follows;
- Original uSDX schematic will be used as a baseline for the transceiver part
- uSDX band pass filters will not be used
- An additional signal generator gor GHZ frequency will be used (AD4530 or similar)
- A mixer will be user do downconvert/upconvert usdx HF capability fo 739 for RX and 2400 MHz for TX

The major plan is to replace ATMEL328p based design to a more powerfull mcu....
