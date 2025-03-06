# OAM Time Machine for Eurorack

This repo is a fork of the original Time Machine module by Olivia Artz Modular.

This one will soon contain the free production files for my slim version of the Time Machine. You can order the boards at your favorite pcb house, bom and pnp files are also available to have the smd components pre-populated. Please be aware that the original license applies, you cannot make and sell the module or pcbs for profit.

And it also contains the firmware and info for the expander that I developed, see below. This expander can be bought from [**me**](https://www.timorozendal.com/tm "me"), either in built (80 euro) or kit (50 euro) form.

## Expander

The active expander adds:
- 4 knobs and cv inputs that are summed and control various things
- stereo offset mode A: odd inputs are panned left, even inputs are panned right
- stereo offset mode B: right delayline has an offset  (also works with clocked mode)
- filters per delay tap, you can choose lowpass, highpass, bandpass and a second bandpass based on the [4MS SMR](https://github.com/4ms/smr/ "4MS SMR") resonator code. You can set the frequency for the first tap and the last tap and it interpolates in between. You can also change the resonance, this is shared for all taps.

Notes:
- short press on the button changes from lpf, hpf, bandpass (both filter leds on), filters off
- when filters are on the delaybuffer becomes mono (needed to save mcu cycles), but the processing and output is still stereo. 
- this also means that stereo mode B is not possible when filters are on.
- with filters off: button long press selects stereo mode B (mode led turns on)
- with bandpass selected: button long press selects the 4ms SMR bandpass mode (mode led turns on)

