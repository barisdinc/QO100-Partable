# QO100-Partable
A portable SDR QO-100 ground station design and implementation

My design basics are as follows;Will use as much as I can from QCX/uSDX designDown/Upconverters will be used, if possible with selection of best frequencies where harmonics will be usable as wellWill have 2 detectors, one will follow the QO1-00 beacon to make frequency stabilisation as in most SDR softwares, but the bandwidth of the receiver is not enough to listen both the target signal and beacon at the same time, so a double tayloe and mcu is preferred (this will eliminate LNB OSC drifts and local OSC drifts)Instead of 2 atmega328 the code can be ported to a more powerful mcu to have just one mcuNow I will start with receiver then move to transmitter partNo PC or similar hardware will be requiredCan be handheld or manpack for emergency operations


<IMG SRC=https://raw.githubusercontent.com/barisdinc/QO100-Partable/main/Docs/qo100-at-sdr_basic_design.png>
