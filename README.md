# OAM Time Machine for Eurorack

This repo is a fork of the original Time Machine module by Olivia Artz Modular.

This contains the free production files for my slim version of the Time Machine. You can order the boards at your favorite pcb house, bom and pnp files are also available to have the smd components pre-populated. Please be aware that the original license applies, you cannot make and sell the module or pcbs for profit.

And it also contains the firmware and info for the expander that I developed, see below. This expander can be bought from [**me**](https://www.timorozendal.com/tm "me"), either in built (80 euro) or kit (50 euro) form.

**For the expander to work you need to update the firmware of the Time Machine,[ see  here for the latest version and instructions](https://github.com/timorozendal/time-machine/releases " see  here for the latest version and instructions")**

Connecting the expansion cable:  the G on the expansion should match the G on the Time Machine itself. I use the red line on the cable to indicate that. (see also [last pictures in the Build Guide](https://docs.google.com/document/d/1udGyIOVuIUEM14i1okJZ_7YuLo60VQ1YnfsVe5B10nA/edit?tab=t.0 "last pictures in the Build Guide"))


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
- cv input range is 0..5V, but it is summed with the knob that also goes from 0..5V. So if the knob is max (5V) you need a negative voltage 0..-5V to cover the whole range

About Feedback and Resonance:
- resonance is basically the feedback of a filter
- the way feedback works with the time machine currently means that the echo that feedbacks is added at the same tap
- this means that increasing the feedback of the time machine with filters with some resonance will increase the resonance even more
- conclusion: be very careful with the feedback when you are using filters with resonance

> I dont have a demo yet, but there are a few previews here in this [instagram highlight](https://www.instagram.com/stories/highlights/18056328476151964/ "instagram highlight")

## Build Guides

- [build guide for the Time Machine](https://docs.google.com/document/d/1G_HOQXmf1HoXMm5cSpaifYvIs_M4wRFkRdWh4EDtrFA/edit?usp=sharing "build guide for the Time Machine")
- [build guide for the Expander](https://docs.google.com/document/d/1udGyIOVuIUEM14i1okJZ_7YuLo60VQ1YnfsVe5B10nA/edit?usp=sharing "build guide for the Expander")
