

//#define REVERSESLIDERS 


//task build_and_program_dfu
#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "dsp.h"
#include "time_machine_hardware.h"

using namespace daisy;
using namespace oam;
using namespace time_machine;
using namespace std;

#define TIME_SECONDS 150
#define BUFFER_WIGGLE_ROOM_SAMPLES 1000

#define LINEAR_TIME false

//Setting Struct containing parameters we want to save to flash
struct CalibrationData {
	float timeCvOffset = 0.0;
	float skewCvOffset = 0.0;
	float feedbackCvOffset = 0.0;
	int calibrated = false;

	//Overloading the != operator
	//This is necessary as this operator is used in the PersistentStorage source code
	bool operator!=(const CalibrationData& a) const {
        return !(
				a.timeCvOffset==skewCvOffset && \
				a.skewCvOffset==skewCvOffset && \
				a.feedbackCvOffset==feedbackCvOffset && \
				a.calibrated==calibrated
			);
    }
};

// init buffers - add an extra second just in case we somehow end up slightly beyond max time
// due to precision loss in floating point arithmetic (maybe use doubles for time values???)
float DSY_SDRAM_BSS bufferLeft[48000 * TIME_SECONDS + BUFFER_WIGGLE_ROOM_SAMPLES];
float DSY_SDRAM_BSS bufferRight[48000 * TIME_SECONDS + BUFFER_WIGGLE_ROOM_SAMPLES];

TimeMachineHardware hw;


PersistentStorage<CalibrationData> CalibrationDataStorage(hw.qspi);
GateIn gate;
Led leds[9];

StereoTimeMachine timeMachine;
ClockRateDetector clockRateDetector;
ContSchmidt timeKnobSchmidt;
ContSchmidt timeKnob2Schmidt;///Timo add: for stereo offset mode
ContSchmidt timeCvSchmidt;

Slew timeKnobSlew;
Slew feedbackKnobSlew;
Slew distributionKnobSlew;
Slew timeCvSlew;
Slew feedbackCvSlew;
Slew distributionCvSlew;


Slew timeKnobSlew2; //Timo add: for stereo offset mode
Slew timeCvSlew2;  //Timo add: for stereo offset mode


// global storage for CV/knobs so we don't get them twice to print diagnostics
float timeCv = 0.0;
float feedbackCv = 0.0;
float skewCv = 0.0;
float timeKnob = 0.0;
float feedbackKnob = 0.0;
float skewKnob = 0.0;
float drySlider = 0.0;
float delaySliders = 0.0;



// Timo add start
float exp2CV = 0.0;
float exp3CV = 0.0;
float exp4CV = 0.0;
float exp5CV = 0.0;
bool  exp1Gate = false;
Slew  exp2CVSlew;
Slew  exp3CVSlew;
Slew  exp4CVSlew;
Slew  exp5CVSlew;
#define LEDVOLTAGE 1.9

bool exp1GatePrev=false;
bool expChangeFilterTypeAction=false;
bool expChangeStereoTypeAction=false;
bool exp1GateHoldAction=false;
#define expFiltStateOff 0
#define expFiltStateLow 1
#define expFiltStateHigh 2
#define expFiltStateBand 3

bool expFiltSMR=false;
int expFiltState=expFiltStateOff;
#define expStereoPanMode false
#define expStereoOffsetMode true
bool  expStereoMode=expStereoPanMode;
float expStereoValLeft=0;
float expStereoValRight=0;
bool expLedFlip=false;
int debugPrintCounter=0;
uint32_t now=0;
uint32_t elapsed=0;
float ledvalues[9];
// Timo add end

// calibration offsets for CV
float timeCvOffset = 0.0;
float feedbackCvOffset = 0.0;
float skewCvOffset = 0.0;

float finalTimeValue = 0.0;
float finalTime2Value = 0.0;
float finalDistributionValue = 0.0;
float finalFeedbackValue = 0.0;

// delay setting LEDs for startup sequences
bool setLeds = false;

CpuLoadMeter cpuMeter;

int droppedFrames = 0;
uint32_t lastButtonChangeTime=0;
uint32_t buttonHoldStartTime=0;
#define BUTTONCHECKINTERVAL 40
#define BUTTONHOLDVALUE 2000
// called every N samples (search for SetAudioBlockSize)
void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	// cpu meter measurements start
	cpuMeter.OnBlockStart();
	elapsed= size;
	now=System::GetNow();
	droppedFrames++;

	// process controls
	hw.ProcessAllControls();



	// Timo add start
	expChangeFilterTypeAction=false;
	expChangeStereoTypeAction=false;
	uint32_t timeSinceLastButtonChange=now-lastButtonChangeTime;//(do in separate var to prevent integer promotion)
	if (timeSinceLastButtonChange>BUTTONCHECKINTERVAL)
	{	
		exp1Gate= hw.gate_in_1.State();
		if (exp1Gate!=exp1GatePrev)
		{
			if (exp1Gate) buttonHoldStartTime=now;
			else
			{
				if (exp1GateHoldAction) exp1GateHoldAction=false;
				else expChangeFilterTypeAction=true;
			}


			exp1GatePrev=exp1Gate;
			lastButtonChangeTime=now;
		}

	}

	//changing stereo mode
	if (exp1Gate && (expFiltState==expFiltStateOff) && (!exp1GateHoldAction))//holding button
	{
		uint32_t holdTime=now-buttonHoldStartTime;//(do in separate var to prevent integer promotion)
		if (holdTime>BUTTONHOLDVALUE)
		{
			expChangeStereoTypeAction=true;
			exp1GateHoldAction=true;
		}
	}

	//enabling extra SMR filter mode
	if (exp1Gate && (expFiltState==expFiltStateBand) && (!exp1GateHoldAction))//holding button
	{
		uint32_t holdTime=now-buttonHoldStartTime;//(do in separate var to prevent integer promotion)
		if (holdTime>BUTTONHOLDVALUE)
		{
			expFiltSMR=!expFiltSMR;
			exp1GateHoldAction=true;
			timeMachine.resetSmrFilter();//TODO: only for smr!
		}
	}

	
	exp2CV = clamp( 1.-exp2CVSlew.Process(minMaxKnob(hw.GetAdcValue(EXP2_CV), 0.01)),0.,1.); 
	exp3CV = 1.-exp3CVSlew.Process(minMaxKnob(hw.GetAdcValue(EXP3_CV), 0.0008));
	exp4CV = 1.-exp4CVSlew.Process(minMaxKnob(hw.GetAdcValue(EXP4_CV), 0.0008));
	exp5CV = 1.-exp5CVSlew.Process(minMaxKnob(hw.GetAdcValue(EXP5_CV), 0.0008));

	//exp2CV = 1;


	
    
//TODO: do action on button release, measure time of button press (system::getnow())
//devide in buttonpressshort and buttonpresslong
//or rather short button press is on release, long buttonpress is on hold

	if(expChangeFilterTypeAction)
	{
		expFiltState++;
		if (expFiltState==4) expFiltState=expFiltStateOff;
		expStereoMode=expStereoPanMode;
		expFiltSMR=false;
		
		
	}

	if(expChangeStereoTypeAction)
	{
		expStereoMode=!expStereoMode;
	}

   
   
	// Timo add end

	// populate/update global CV/knob vars (time is slewed to reduce noise at large time values)
	timeKnob = minMaxKnob(1.0 - hw.GetAdcValue(TIME_KNOB), 0.0008);
	feedbackKnob = fourPointWarp(1.0 - minMaxKnob(hw.GetAdcValue(FEEDBACK_KNOB), 0.028));
	skewKnob = fourPointWarp(1.0 - minMaxKnob(hw.GetAdcValue(SKEW_KNOB), 0.0008));

	timeCv = clamp(hw.GetAdcValue(TIME_CV) - timeCvOffset, -1, 1);
	feedbackCv = clamp(hw.GetAdcValue(FEEDBACK_CV) - feedbackCvOffset, -1, 1);
	skewCv = clamp(hw.GetAdcValue(SKEW_CV) - skewCvOffset, -1, 1);

	#ifdef REVERSESLIDERS
	drySlider = minMaxSlider(hw.GetAdcValue(DRY_SLIDER));
	#else
	drySlider = minMaxSlider(1.0 - hw.GetAdcValue(DRY_SLIDER));
	#endif

	//TIMO TODO: calculate two time values in time offset case L and R

	// calculate time based on clock if present, otherwise simple time
	float time = 0.0; 
	float time2=0.0;//Timo add, for stereo offset mode
	if(clockRateDetector.GetInterval() > 0.0) {
		// 12 quantized steps for knob, 10 for CV (idk what these quanta should actually be)
		// time doubles and halves with each step, they are additive/subtractive
		float timeCoef = pow(2.0, (timeKnobSchmidt.Process((1.0-timeKnob)*12)) + (timeCvSchmidt.Process(timeCv*10))) / pow(2.0, 6.0);
		time = clockRateDetector.GetInterval() / timeCoef;
		// make sure time is a power of two less than the max time available in the buffer
		while(time > TIME_SECONDS) time *= 0.5;
	} else {
		// time linear with knob, scaled v/oct style with CV
		time = pow(timeKnobSlew.Process(timeKnob), 2.0) * 8.0 / pow(2.0, timeCvSlew.Process(timeCv) * 5.0);
	}
	// force time down to a max value (taking whichever is lesser, the max or the time)
	time = std::min((float)TIME_SECONDS, time);



	if (expStereoMode==expStereoOffsetMode)
	{
		
		if(clockRateDetector.GetInterval() > 0.0) {
			// 12 quantized steps for knob, 10 for CV (idk what these quanta should actually be)
			// time doubles and halves with each step, they are additive/subtractive
			float timeCoef = pow(2.0, (timeKnob2Schmidt.Process((1.0-timeKnob-exp2CV/2.0)*12)) + (timeCvSchmidt.Process(timeCv*10))) / pow(2.0, 6.0);
			time2 = clockRateDetector.GetInterval() / timeCoef;
			// make sure time is a power of two less than the max time available in the buffer
			while(time2 > TIME_SECONDS) time2 *= 0.5;
		} else {
			// time linear with knob, scaled v/oct style with CV
			time2 = pow(timeKnobSlew2.Process(timeKnob+0.25*exp2CV), 2.0) * 8.0 / pow(2.0, timeCvSlew2.Process(timeCv) * 5.0);
		}
		// force time down to a max value (taking whichever is lesser, the max or the time)
		time2 = std::min((float)TIME_SECONDS, time2);
	}
	else time2=time;



	// condition feedback knob to have deadzone in the middle, add CV
	float feedback = clamp(fourPointWarp(feedbackKnobSlew.Process(feedbackKnob)) * 2.0 + feedbackCvSlew.Process(feedbackCv), 0, 3); 
	// condition distribution knob value to have deadzone in the middle, add CV
	float distribution = fourPointWarp(distributionKnobSlew.Process(skewKnob)) + distributionCvSlew.Process(skewCv);

	finalTimeValue = time;
	finalTime2Value = time2;
	finalFeedbackValue = feedback;
	finalDistributionValue = distribution;


	for(int i=0; i<9; i++) {
		if(i<8) {
			// set LEDs based on loudness for last 8 sliders
			float loudness = timeMachine.timeMachineLeft.readHeads[i].loudness.Get();
			if (expFiltState==expFiltStateOff) loudness = max(loudness, timeMachine.timeMachineRight.readHeads[i].loudness.Get());
			if(setLeds) {
				ledvalues[i+1]=loudness;
				
				//leds[i+1].Update();
			}
		} else {
			// set LEDs based on loudness for first slider
			float loudness = timeMachine.timeMachineLeft.loudness.Get();
			if (expFiltState==expFiltStateOff) loudness = max(loudness, timeMachine.timeMachineRight.loudness.Get());
			if(setLeds) {
				//leds[0].Set(loudness);
				ledvalues[0]=loudness;
				//leds[0].Update();
			}
		}
	}
	
    
	timeMachine.SetFilters(exp3CV,exp4CV,exp5CV,expFiltState+expFiltSMR);//Timo add
	//timeMachine.SetFilters(0.9,0.05,0.2,expFiltState);//Timo TEST add
      
	// set time machine dry slider value, feedback, "blur" which is semi-deprecated
	timeMachine.Set(drySlider, feedback, feedback,exp2CV); // controlling "blur" with feedback now???
    
	

	if(expFiltState==expFiltStateOff) //stereo mode
	{
		for(int i=1; i<9; i++) {
			// let last 8 slider time/amp/blur values for left channel time machine instance
			if(expStereoMode==expStereoPanMode) expStereoValLeft=(i%2)*exp2CV; //Timo add
			else expStereoValLeft=0;
			#ifdef REVERSESLIDERS
			timeMachine.timeMachineLeft.readHeads[i-1].Set(
				spread((i / 8.0), distribution) * time,
				max(0.0f, minMaxSlider(hw.GetSliderValue(i))),  
				max(0., feedback-1.0),expStereoValLeft //Timo add expStereoValLeft
			);
			#else
			timeMachine.timeMachineLeft.readHeads[i-1].Set(
				spread((i / 8.0), distribution) * time,
				max(0.0f, minMaxSlider(1.0f - hw.GetSliderValue(i))),  
				max(0., feedback-1.0),expStereoValLeft //Timo add expStereoValLeft
			);
			#endif
			// let last 8 slider time/amp/blur values for right channel time machine instance
			if(expStereoMode==expStereoPanMode)expStereoValRight=((i+1)%2)*exp2CV; //Timo add
			else expStereoValRight=0;
			
			#ifdef REVERSESLIDERS
			timeMachine.timeMachineRight.readHeads[i-1].Set(
				spread((i / 8.0), distribution) * time2,  //Timo add: time->time2 to allow stereo offset mode
				max(0.0f, minMaxSlider(hw.GetSliderValue(i))),
				max(0., feedback-1.0),expStereoValRight  //Timo add expStereoValRight
			);
			#else
			timeMachine.timeMachineRight.readHeads[i-1].Set(
				spread((i / 8.0), distribution) * time2,  //Timo add: time->time2 to allow stereo offset mode
				max(0.0f, minMaxSlider(1.0f - hw.GetSliderValue(i))),
				max(0., feedback-1.0),expStereoValRight  //Timo add expStereoValRight
			);
			#endif	
			
		}
	}
	else //mono/pseudostereo mode  (if filters are ON)
	{
		for(int i=1; i<9; i++) {
			// let last 8 slider time/amp/blur values for left channel time machine instance
			#ifdef REVERSESLIDERS
			timeMachine.timeMachineLeft.readHeads[i-1].Set(
				spread((i / 8.0), distribution) * time,
				max(0.0f, minMaxSlider(hw.GetSliderValue(i))), 
				max(0., feedback-1.0)
			);
			#else
			timeMachine.timeMachineLeft.readHeads[i-1].Set(
				spread((i / 8.0), distribution) * time,
				max(0.0f, minMaxSlider(1.0f - hw.GetSliderValue(i))), 
				max(0., feedback-1.0)
			);
			#endif
		}
	}
	

	for (size_t i = 0; i < size; i++)
	{
		// process gate for clock rate detector at audio rate (per-sample) so it calculates clock correctly
		clockRateDetector.Process(hw.gate_in_2.State());
		// process input into time machine
		float* output = timeMachine.Process(in[0][i], in[1][i]);
		// set hardware output to time machine output
		out[0][i] = output[0];
		out[1][i] = output[1];
	}

	//cpu meter measurement stop
	cpuMeter.OnBlockEnd();
	droppedFrames--;
}

bool shouldCalibrate() {
		bool shouldCalibrate = \
			(hw.GetAdcValue(SKEW_CV) < 0.01) && \
			(hw.GetAdcValue(TIME_CV) < 0.01) && \
			(hw.GetAdcValue(FEEDBACK_CV) < 0.01) && \
			hw.gate_in_2.State();
		for(int i=0; i<9; i++) {
			shouldCalibrate &= hw.GetSliderValue(i) < 0.01;
		}
		shouldCalibrate &= minMaxKnob(1.0 - hw.GetAdcValue(TIME_KNOB)) > 0.95;
		shouldCalibrate &= minMaxKnob(1.0 - hw.GetAdcValue(SKEW_KNOB)) > 0.95;
		shouldCalibrate &= minMaxKnob(1.0 - hw.GetAdcValue(FEEDBACK_KNOB)) > 0.95;
		return shouldCalibrate;
}

int main(void)
{
	// init time machine hardware
    hw.Init();
	hw.SetAudioBlockSize(4); // number of samples handled per callback

	dsy_gpio_pin gatePin = DaisyPatchSM::B9;
	gate.Init(&gatePin);

	// initialize LEDs
	leds[0].Init(DaisyPatchSM::D1, false);
	leds[1].Init(DaisyPatchSM::D2, false);
	leds[2].Init(DaisyPatchSM::D3, false);
	leds[3].Init(DaisyPatchSM::D4, false);
	leds[4].Init(DaisyPatchSM::D5, false);
	// hacky selector for my weird dev version (Eris)
	#if BODGE
	leds[5].Init(DaisyPatchSM::B8, false);
	#else
	leds[5].Init(DaisyPatchSM::A9, false);
	#endif
	leds[6].Init(DaisyPatchSM::D10, false);
	leds[7].Init(DaisyPatchSM::D7, false);
	leds[8].Init(DaisyPatchSM::D6, false);

	// set sample rate
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

	// init slew limiter for time (we should tune this more delibrately)
	timeKnobSlew.Init(0.5, 0.0005);
	feedbackKnobSlew.Init(0.5, 0.0005);
	distributionKnobSlew.Init(0.5, 0.0005);
	timeCvSlew.Init(0.5, 0.0005);
	feedbackCvSlew.Init(0.5, 0.0005);
	distributionCvSlew.Init(0.5, 0.0005);


	// Timo add start
	timeKnobSlew2.Init(0.5, 0.0005);
	timeCvSlew2.Init(0.5, 0.0005);
  	//exp2CVSlew.Init(0.5, 0.0005);
	exp2CVSlew.Init(0.5, 0.01);
  	exp3CVSlew.Init(0.5, 0.0005);
  	exp4CVSlew.Init(0.5, 0.0005);
  	exp5CVSlew.Init(0.5, 0.0005);
	// Timo add end

	// init clock rate detector
	clockRateDetector.Init(hw.AudioSampleRate());

	// init time machine
    timeMachine.Init(hw.AudioSampleRate(), TIME_SECONDS + (((float)BUFFER_WIGGLE_ROOM_SAMPLES) * 0.5 / hw.AudioSampleRate()), bufferLeft, bufferRight);

	// load calibration data, using sensible defaults
	CalibrationDataStorage.Init({0.0f, 0.0f, 0.0f, false});
	CalibrationDataStorage.GetSettings();
	CalibrationData &savedCalibrationData = CalibrationDataStorage.GetSettings();

	hw.SetAudioBlockSize(8);//TIMO test add
	// init cpu meter
	cpuMeter.Init(hw.AudioSampleRate(), hw.AudioBlockSize());

	// start time machine hardware audio and logging
    hw.StartAudio(AudioCallback);

	// LED startup sequence
	int ledSeqDelay = 100;
	for(int i=0; i<9; i++) {
		for(int j=0; j<9; j++) {
			leds[j].Set(j == i ? 1.0 : 0.0);
			leds[j].Update();
		}
		hw.PrintLine("%d", i);
		System::Delay(ledSeqDelay);
	}

	if(shouldCalibrate()) {

		bool calibrationReady = true;

		// do reverse LED startup sequence while
		// checking that we definitely want to calibrate
		for(int i=0; i<(5000/ledSeqDelay); i++) {
			for(int j=0; j<9; j++) {
				leds[j].Set(j == (8 - (i%9)) ? 1.0 : 0.0);
				leds[j].Update();
			}
			System::Delay(ledSeqDelay);
			calibrationReady &= shouldCalibrate();
			if(!calibrationReady) break;
		}
		
		if(calibrationReady) {
			// perform calibration routine
			int numSamples = 128;
			for(int i = 0; i < numSamples; i++) {
				// accumulate cv values
				savedCalibrationData.timeCvOffset += timeCv;
				savedCalibrationData.skewCvOffset += skewCv;
				savedCalibrationData.feedbackCvOffset += feedbackCv;
				// wait 10ms
				System::Delay(10);
				// set LEDs
				for(int ledIndex=0; ledIndex<9; ledIndex++) {
					leds[ledIndex].Set(i % 8 < 4 ? 1.0f : 0.0f);
					leds[ledIndex].Update();
				}
			}
			
			// divide CVs by number of samples taken to get average
			savedCalibrationData.timeCvOffset = savedCalibrationData.timeCvOffset / ((float)numSamples);
			savedCalibrationData.skewCvOffset = savedCalibrationData.skewCvOffset / ((float)numSamples);
			savedCalibrationData.feedbackCvOffset = savedCalibrationData.feedbackCvOffset / ((float)numSamples);
			
			// set calibrated value to true
			savedCalibrationData.calibrated = true;
			
			// save calibration data
			CalibrationDataStorage.Save();
		}
	}

	timeCvOffset = savedCalibrationData.timeCvOffset;
	skewCvOffset = savedCalibrationData.skewCvOffset;
	feedbackCvOffset = savedCalibrationData.feedbackCvOffset;

	hw.StartLog();

	setLeds = true;

	while(1) {

		
		if (debugPrintCounter>50000)
		{
			debugPrintCounter=0;
		
		// print diagnostics
		hw.PrintLine("TIME_CV: " FLT_FMT(6), FLT_VAR(6, timeCv));
		hw.PrintLine("FEEDBACK_CV: " FLT_FMT(6), FLT_VAR(6, feedbackCv));
		hw.PrintLine("SKEW_CV: " FLT_FMT(6), FLT_VAR(6, skewCv));
		hw.PrintLine("TIME_KNOB: " FLT_FMT(6), FLT_VAR(6, timeKnob));
		hw.PrintLine("FEEDBACK_KNOB: " FLT_FMT(6), FLT_VAR(6, feedbackKnob));
		hw.PrintLine("SKEW_KNOB: " FLT_FMT(6), FLT_VAR(6, skewKnob));
		hw.PrintLine("GATE 2 IN: %d", hw.gate_in_2.State());
		hw.PrintLine("GATE 1 IN: %d", hw.gate_in_1.State());
		hw.PrintLine("CV IN 1: " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(CV_4)));
		hw.PrintLine("CV IN 2: " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(CV_5)));
		hw.PrintLine("CV IN 3: " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(CV_6)));
		hw.PrintLine("CV IN 4: " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(CV_7)));
		hw.PrintLine("exp2cv: " FLT_FMT(6), FLT_VAR(6, exp2CV));
		hw.PrintLine("exp3cv: " FLT_FMT(6), FLT_VAR(6, exp3CV));
		hw.PrintLine("TIME_CAL: " FLT_FMT(6), FLT_VAR(6, savedCalibrationData.timeCvOffset));
		hw.PrintLine("FEEDBACK_CAL: " FLT_FMT(6), FLT_VAR(6, savedCalibrationData.feedbackCvOffset));
		hw.PrintLine("SKEW_CAL: " FLT_FMT(6), FLT_VAR(6, savedCalibrationData.skewCvOffset));
		hw.PrintLine("CALIBRATED: %d", savedCalibrationData.calibrated);

		hw.PrintLine("FINAL TIME: " FLT_FMT(6), FLT_VAR(6, finalTimeValue));
		hw.PrintLine("FINAL TIME2: " FLT_FMT(6), FLT_VAR(6, finalTime2Value));
		hw.PrintLine("FINAL DISTRIBUTION: " FLT_FMT(6), FLT_VAR(6, finalDistributionValue));
		hw.PrintLine("FINAL FEEDBACK: " FLT_FMT(6), FLT_VAR(6, finalFeedbackValue));

		hw.PrintLine("CPU AVG: " FLT_FMT(6), FLT_VAR(6, cpuMeter.GetAvgCpuLoad()));
		hw.PrintLine("CPU MIN: " FLT_FMT(6), FLT_VAR(6, cpuMeter.GetMinCpuLoad()));
		hw.PrintLine("CPU MAX: " FLT_FMT(6), FLT_VAR(6, cpuMeter.GetMaxCpuLoad()));

		hw.PrintLine("DROPPED FRAMES: %d", droppedFrames);
		hw.PrintLine("elapsed: %d", elapsed);

		for(int i=0; i<9; i++) {
			hw.PrintLine("%d: " FLT_FMT(6), i, FLT_VAR(6, minMaxSlider(1.0 - hw.GetSliderValue(i))));
		}

		hw.PrintLine("");
		
		}  
		
		//System::Delay(5);
		
		 if(expLedFlip)
		{
			if( (expFiltState== expFiltStateHigh)||(expFiltState== expFiltStateBand))hw.WriteCvOut(2,0);
			else hw.WriteCvOut(2,2.5);
		}
		else
		{
			if( (expFiltState== expFiltStateLow)||(expFiltState== expFiltStateBand))hw.WriteCvOut(2,5);
			else hw.WriteCvOut(2,2.5);
		}
		if ((expStereoMode== expStereoOffsetMode)||((expFiltState==expFiltStateBand)&&expFiltSMR))
		{
			hw.WriteCvOut(1,LEDVOLTAGE);
		}
		else hw.WriteCvOut(1,0);

    	expLedFlip=!expLedFlip;
		
		debugPrintCounter++;
		if(setLeds)
		{	for(int i=0; i<9; i++)
			{ 
				leds[i].Set(ledvalues[i]);
				leds[i].Update();
			}
		}

		
	}
}
