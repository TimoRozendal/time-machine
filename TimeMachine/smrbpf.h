/*
Timo: 
this code is based on this file: https://github.com/4ms/SMR/blob/master/filter.c
part of the 4ms smr firmware
below is the original copyright notice
*/

/*
 * filter.c - DSP bandpass resonant filter
 *
 * Author: Dan Green (danngreen1@gmail.com), Hugo Paris (hugoplho@gmail.com)
 * Algorithm based on work by Max Matthews and Julius O. Smith III, "Methods for Synthesizing Very High Q Parametrically Well Behaved Two Pole Filters", as published here: https://ccrma.stanford.edu/~jos/smac03maxjos/smac03maxjos.pdf
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * See http://creativecommons.org/licenses/MIT/ for more information.
 *
 * -----------------------------------------------------------------------------
 */


#include <math.h>


//define to use direct expf call to reduce flash size (to prevent making a bootloader hassle)
#define NOLOOKUP


#define M_PI 3.14159265358979323846
#ifndef NOLOOKUP
#include "exp_4096.h"
#endif
#define CLAMP(a, lo, hi) ( (a)>(lo)?( (a)<(hi)?(a):(hi) ):(lo) )

class tr_smrbpf_filter
{
    
private:
    
    double c0,c1,c2;
    double y0,y1,y2;
    double sr;
    

    double qval,freqval;
    double var1;


    
public:
    tr_smrbpf_filter()
    {

        qval =20;
        freqval=1000;
        sr=48000;

        var1 = (2*M_PI)/sr;
        
        y0=0; y1=0; y2=0;
        

       // set(f,q,type,sr);
    }

    void set(double freq, double q)
    {
        freqval=CLAMP(freq,30,0.22*sr);
        #ifndef NOLOOKUP
        qval= CLAMP(q*4096.0,0, 4095);
        #else
        qval=q;
        #endif
        calculate();

    }
    
    void calculate()
    {
        //Q/RESONANCE: c0 = 1 - 2/(decay * samplerate), where decay is around 0.01 to 4.0
        #ifndef NOLOOKUP
        c0 = 1.0 - exp_4096[(__uint32_t)(qval/1.4)+200]/10.0; //exp[200...3125]
        #else
        c0=1.0- 1.31*expf(-39.2301*(qval/7.+0.05))/10.0;
        c0=CLAMP(c0,0.,1.);
        #endif
        
        //FREQ: c1 = 2 * pi * freq / samplerate
        c1 = var1*freqval;
        
        //AMPLITUDE: Boost high freqs and boost low resonance
        c2= (0.003 * c1) - (0.1*c0) + 0.102;
        c2 *= ((4096.0-qval)/1024.0) + 1.04;
    }
    
    void clear()
    {
        y0=0;
        y1=0;
        y2=0;
       
    }
    void perform(double* in, double* out, int n)
    {
        //calculate();
        for (int i=0;i<n;i++)
        {
            y2 = (c0 * y1 + c1 * y0) - c2 * in[i];
            y0 = y0 - (c1 * y2);
            y1 = y2;
            out[i]= y1;
        }
    }
    double perform(double in)
    {

            y2 = (c0 * y1 + c1 * y0) - c2 * in;
            y0 = y0 - (c1 * y2);
            if (isnan(y1)||isinf(y1)) y1=0; //quick prevntion for blowing up
            y1 = y2;
            
            return y1;
        
    }
    
    
};