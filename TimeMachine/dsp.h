#include "daisysp.h"
#include "biquad.h"
#include "smrbpf.h"

int wrap_buffer_index(int x, int size) {
    while(x >= size) x -= size;
    while(x < 0) x += size;
    return x;
}

int seconds_to_samples(float x, float sampleRate) {
    return (int)(x * sampleRate);
}

float mix(float x, float a, float b) {
    return x*(1-x) + b*x;
}

float clamp(float x, float a, float b) {
    return std::max(a,std::min(b,x));
}

float fourPointWarp(float x,
                    float ai=0.0,
                    float av=0.0,
                    float bi=0.45,
                    float bv=0.5,
                    float ci=0.55,
                    float cv=0.5,
                    float di=1.0,
                    float dv=1.0) {
    if(x < ai) {
        return av;
    } else if(x < bi) {
        x = (x - ai) / (bi - ai);
        return av * (1.0 - x) + bv * x;
    } else if(x < ci) {
        x = (x - bi) / (ci - bi);
        return bv * (1.0 - x) + cv * x;
    } else if(x < di) {
        x = (x - ci) / (di - ci);
        return cv * (1.0 - x) + dv * x;
    } else {
        return dv;
    }
}

float minMaxKnob(float in, float dz=0.002) {
    in = in - dz * 0.5;
    in = in * (1.0 + dz);
    return std::min(1.0f, std::max(0.0f, in));
}

float minMaxSlider(float in, float dz=0.002) {
    return minMaxKnob(in, dz);
}

float softClip(float x, float kneeStart=0.9, float kneeCurve=5.0) {
    float linPart = clamp(x, -kneeStart, kneeStart);
    float clipPart = x - linPart;
    clipPart = atan(clipPart * kneeCurve) /  kneeCurve;
    return linPart + clipPart;
}

float spread(float x, float s, float e=2.5) {
    s = clamp(s, 0.0, 1.0);
    if(s > 0.5) {
        s = (s-0.5)*2.0;
        s = s*e+1.0;
        return 1.0 - pow(1.0-x, s);
    } else if(s < 0.5) {
        s = 1.0-(s*2.0);
        s = s*e+1.0;
        return pow(x, s);
    } else {
        return x;
    }
}

//Timo add start
float mtof(float note)
{
    //range 0..136
    return (440. * exp(.057762265 * (note - 69.)));
  
}

//Timo add end


class PreciseSlew
{
  public:
    PreciseSlew() {}
    ~PreciseSlew() {}
    void Init(float sample_rate, float htime) {
        lastVal  = 0;
        prvhtim_ = -100.0;
        htime_   = htime;

        sample_rate_ = sample_rate;
        onedsr_      = 1.0 / sample_rate_;
    }
    float Process(float in) {
        if(prvhtim_ != htime_)
        {
            c2_      = pow(0.5, onedsr_ / htime_);
            c1_      = 1.0 - c2_;
            prvhtim_ = htime_;
        }
        return lastVal = c1_ * in + c2_ * lastVal;
    }
    inline void SetHtime(float htime) { htime_ = htime; }
    inline float GetHtime() { return htime_; }
    float htime_;
    float c1_, c2_, lastVal, prvhtim_;
    float sample_rate_, onedsr_;
};


class Slew {
public:
    double lastVal = 0.0;
    double coef = 0.001;
    double noiseFloor = 0.0;
    double noiseCoef = 0.0;
    int settleSamples = 0;
    int settleSamplesThreshold = 96;
    void Init(double coef = 0.001, double nf=0.0) {
        this->coef = coef;
        this->noiseFloor = nf;
    }
    float Process(float x) {
        double c = coef;
        double d = (x - lastVal);
        // if we've set a noise floor
        if(noiseFloor > 0.0) {
            // if we're under the noise floor
            if(abs(d) < noiseFloor) {
                // if the input needs to settle
                if(settleSamples < settleSamplesThreshold) {
                    // keep sampling
                    settleSamples++;
                // if the input is done settling
                } else {
                    // don't change the value
                    d = 0.0;
                }
            // if we're over the noise floor
            } else {
                // reset the settle wait
                settleSamples = 0;
            }
        }
        lastVal = lastVal + d * c;
        return lastVal;
    }
};

class ContSchmidt {
public:
    float val = 0.0;
    float Process(float x, float h=0.333) {
        float i, f;
        float sign = x < 0.0 ? -1.0 : 1.0;
        x = abs(x);
	    f = modf(x, &i);
        if(f < h) val = i;
        if(f > 1.0 - h) val = i + 1;
        return val * sign;
    }
};

class UltraSlowDCBlocker {
public:
    Slew slew;
    void Init(float coef = 0.00001) {
        slew.Init(coef);
    }
    float Process(float x) {
        return x - slew.Process(x);
    }
};

class LoudnessDetector {
public:
    Slew slew;
    float lastVal = 0;
    void Init() { slew.Init(); }
    float Get() { return this->lastVal; }
    float Process(float x) {
        lastVal = slew.Process(abs(x));
        return x;
    }
};

class Limiter {
public:
    float gainCoef;
    float attackCoef;
    float releaseCoef;
    void Init(float sampleRate) {
        gainCoef = 1;
        releaseCoef = 16.0 / sampleRate;
    }
    float Process(float in) {
        float targetGainCoef = 1.0 / std::max(abs(in), (float)1.0);
        if(targetGainCoef < gainCoef) {
            gainCoef = targetGainCoef;
        } else {
            gainCoef = gainCoef * (1.0 - releaseCoef) + targetGainCoef*releaseCoef;
        }
        return in * gainCoef;
    }
};

class ReadHead {
public:
    LoudnessDetector loudness;
    // Timo add start
    Biquad filter;
    tr_smrbpf_filter smrbpf;
    int filterType=0;
    // Timo add end
    float* buffer;
    int bufferSize;
    float delayA = 0.0;
    float delayB = 0.0;
    float targetDelay = -1;
    float ampA = 0.0;
    float ampB = 0.0;
    float targetAmp = -1;
	float sampleRate;
    float phase = 1.0;
    float delta;
    float blurAmount;
    float stereoOffset=0;
    void Init(float sampleRate, float* buffer, int bufferSize) {
        this->sampleRate = sampleRate;
        this->delta = 5.0 / sampleRate;
        this->buffer = buffer;
        this->bufferSize = bufferSize;
        this->blurAmount = 0.0;
        
    }
    void Set(float delay, float amp, float blur = 0, float stereo=0) { //Timo add stereo
        this->targetDelay = delay;
        this->targetAmp = amp;
        this->blurAmount = blur;
        this->stereoOffset = stereo; 
    }
    // Timo add start
    void SetFilter(int type, double Freq, double Q) 
    {
        this->filterType=type;
        if ((type>0)&& (type<4))
        {
            this->filter.setBiquad(type-1, Freq/this->sampleRate, Q*10+0.7,0);
        }
        else if (type==4)
        {
            this->smrbpf.set(Freq,Q);
        }
    }
    // Timo add end
    float Process(float writeHeadPosition) {
        if(phase >= 1.0 && (targetDelay >= 0.0 || targetAmp > 0.0)) {
            delayA = delayB;
            delayB = targetDelay;
            targetDelay = -1.0;
            ampA = ampB;
            ampB = std::max(0.f,targetAmp-stereoOffset);//Timo add stereoOffset
            targetAmp = -1.0;
            phase = 0.0;
            delta = (5.0 + daisy::Random::GetFloat(-blurAmount, blurAmount)) / sampleRate;
        }
        float outputA = this->buffer[wrap_buffer_index(writeHeadPosition - seconds_to_samples(this->delayA, this->sampleRate), bufferSize)];
        float outputB = this->buffer[wrap_buffer_index(writeHeadPosition - seconds_to_samples(this->delayB, this->sampleRate), bufferSize)];
        float output = ((1 - phase) * outputA) + (phase * outputB);
        float outputAmp = ((1 - phase) * ampA) + (phase * ampB);
        phase = phase <= 1.0 ? phase + delta : 1.0;
        if ((this->filterType>0) && (this->filterType<4))
        {
            output= this->filter.process(output);// Timo add 
            if (this->filterType==3) output*=4.0;//Timo add, increase volume for bandpass mode
        }
        else if (this->filterType==4) output=this->smrbpf.perform(output);
        
        return loudness.Process(output) * outputAmp;
    }
};

float defaultBlurFunc(float x, float mix) { return x; }
class TimeMachine {
public:
    ReadHead readHeads[8];
    LoudnessDetector loudness;
	float sampleRate;
    float* buffer;
    int bufferSize;
    int writeHeadPosition;
    float dryAmp;
    float feedback;
    float blur;
    Slew dryAmpSlew;
    Slew feedbackSlew;
    Slew ampCoefSlew;
    Slew blurSlew;
    Limiter outputLimiter;
    Limiter feedbackLimiter;
    UltraSlowDCBlocker dcblk;
    daisysp::Compressor compressor;

    float StereoVal=0.;

    UltraSlowDCBlocker dcblk2; //Timo add: for pseudostereo operation
    daisysp::Compressor compressor2;//Timo add: for pseudostereo operation
    Limiter outputLimiter2;//Timo add: for pseudostereo operation

    
    float (*blurFunc)(float, float) = nullptr;
    void Init(float sampleRate, float maxDelay, float* buffer) {
		this->sampleRate = sampleRate;
        this->bufferSize = seconds_to_samples(maxDelay, sampleRate);
        this->buffer = buffer;
        for(int i=0; i<bufferSize; i++) buffer[i] = 0;
        for(int i=0; i<8; i++) readHeads[i].Init(sampleRate, buffer, bufferSize);
        writeHeadPosition = 0;
        dryAmpSlew.Init();
        feedbackSlew.Init(0.01);
        ampCoefSlew.Init(0.0001);
        blurSlew.Init();
        outputLimiter.Init(sampleRate);
        feedbackLimiter.Init(sampleRate);
        blurFunc = defaultBlurFunc;
        loudness.Init();
        
        
        compressor.Init(sampleRate);
        compressor.SetAttack(0.02);
        compressor.SetRelease(0.2);
        compressor.SetRatio(5.0);
        compressor.SetThreshold(0.0);

        //Timo add: for pseudostereo operation
        compressor2.Init(sampleRate);
        compressor2.SetAttack(0.02);
        compressor2.SetRelease(0.2);
        compressor2.SetRatio(5.0);
        compressor2.SetThreshold(0.0);
        outputLimiter2.Init(sampleRate);
    }
    void Set(float dryAmp, float feedback, float blur=0.0, float StereoVal=0.0) {
        this->dryAmp = dryAmp;
        this->feedback = feedback;
        this->blur = blur;
        this->StereoVal=StereoVal;
    }
    void SetBlurFunc(float (*f)(float, float)) {
        blurFunc = f;
    }
    
    float Process(float in) {
        float out = 0;

        float ampCoef = 0.0;
        for(int i=0; i<8; i++) ampCoef += readHeads[i].targetAmp;
        ampCoef = ampCoefSlew.Process(1.0 / std::max(1.0f, ampCoef));

        buffer[writeHeadPosition] = loudness.Process(in);
        
        for(int i=0; i<8; i++) out += readHeads[i].Process(writeHeadPosition);
        out = dcblk.Process(out);
        out = compressor.Process(out, buffer[writeHeadPosition] + out);

        buffer[writeHeadPosition] = -(feedbackLimiter.Process(buffer[writeHeadPosition] + (out * feedbackSlew.Process(feedback) * ampCoef)));
        out = outputLimiter.Process(out + in * dryAmpSlew.Process(dryAmp));
        writeHeadPosition = wrap_buffer_index(writeHeadPosition + 1, bufferSize);

        return out;
    }


    void Process2(float inL, float inR, float *outLptr, float *outRptr) //Timo add: for pseudostereo operation
    {
        float out = 0, outL=0, outR=0, outTap=0,expStereoValLeft,expStereoValRight;

        float ampCoef = 0.0;
        for(int i=0; i<8; i++) ampCoef += readHeads[i].targetAmp;
        ampCoef = ampCoefSlew.Process(1.0 / std::max(1.0f, ampCoef));

        buffer[writeHeadPosition] = loudness.Process((inL+inR)*0.707);
        
        for(int i=0; i<8; i++)
        {
            expStereoValLeft=1-((i%2)*StereoVal); //Timo add
            expStereoValRight=1-(((i+1)%2)*StereoVal); //Timo add

            outTap=readHeads[i].Process(writeHeadPosition);
            outL+=outTap*expStereoValLeft;
            outR+=outTap*expStereoValRight;
        } 


        outL = dcblk.Process(outL);
        outR = dcblk2.Process(outR);
        outL = compressor.Process(outL, buffer[writeHeadPosition] + outL);
        outR = compressor2.Process(outR, buffer[writeHeadPosition] + outR);

        out=(outL+outR)*0.707;
        buffer[writeHeadPosition] = -(feedbackLimiter.Process(buffer[writeHeadPosition] + (out * feedbackSlew.Process(feedback) * ampCoef)));
        outL = outputLimiter.Process(outL + inL * dryAmpSlew.Process(dryAmp));
        outR = outputLimiter2.Process(outR + inR * dryAmpSlew.Process(dryAmp));
        writeHeadPosition = wrap_buffer_index(writeHeadPosition + 1, bufferSize);

        *outLptr=outL;
        *outRptr=outR;
    }

 
};

class StereoTimeMachine {
public:
    TimeMachine timeMachineLeft;
    TimeMachine timeMachineRight;
    int filterType=0;//add 0=off, 1=lpf, 2=hpf,3=bpf
    float outputs[2];
    void Init(float sampleRate, float maxDelay, float* bufferLeft, float* bufferRight) {
        timeMachineLeft.Init(sampleRate, maxDelay, bufferLeft);
        timeMachineRight.Init(sampleRate, maxDelay, bufferRight);
    }
    void Set(float dryAmp, float feedback, float blur=0.0, float stereoval=0.0) {
        timeMachineLeft.Set(dryAmp, feedback, blur,stereoval);
        timeMachineRight.Set(dryAmp, feedback, blur,stereoval);
    }
    // Timo add start
    void resetSmrFilter()
    {
        for(int i=0; i<8; i++)
        {
            timeMachineLeft.readHeads[i].smrbpf.clear();
        }
    }

    void SetFilters(float start, float end, float q, int type)
    {
        float freq=0;
        filterType=type;
        for(int i=0; i<8; i++)
        {
            if(type==0) 
            {
                timeMachineLeft.readHeads[i].SetFilter(type,0,0); //skip expensive mtof if filter is off
            }
            else 
            {
                freq= mtof(( (end-start)/7.*i+start)*136.0);//only calculate once
                timeMachineLeft.readHeads[i].SetFilter(type,freq,q); 
            }
        } 
    }
     // Timo add end

    float* Process(float inLeft, float inRight) {
        if (filterType==0) //Timo add
        {
            outputs[0] = timeMachineLeft.Process(inLeft);
            outputs[1] = timeMachineRight.Process(inRight); 
        }
        else
        {
            timeMachineLeft.Process2(inLeft,inRight,&(outputs[0]),&(outputs[1]));//Timo add
        }
        
        return outputs;
    }
};

class ClockRateDetector {
public:
  int samplesSinceLastClock;
  int lastIntervalInSamples;
  bool lastVal;
  float sampleRate;
  ClockRateDetector() {
    samplesSinceLastClock = 0;
    lastIntervalInSamples = 0;
    lastVal = false;
  }
  void Init(int sr) { sampleRate = sr; }
  bool isStale() {
    return samplesSinceLastClock > sampleRate * 4;//Timo change, was 2
  }
  float GetInterval() {
    float interval = lastIntervalInSamples / sampleRate;
    return isStale() ? 0.0 : interval;
  }
  void Process(bool triggered) {
    if(triggered && lastVal != triggered) {
      if(isStale()) {
        lastIntervalInSamples = samplesSinceLastClock;
      } else {
        lastIntervalInSamples = (lastIntervalInSamples + samplesSinceLastClock) * 0.5;
      }
      samplesSinceLastClock = 0;
    } else {
      samplesSinceLastClock++;
    }
    lastVal = triggered;
  }
};