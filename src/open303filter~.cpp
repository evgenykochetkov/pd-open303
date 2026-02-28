// uses code from https://github.com/RobinSchmidt/Open303
// see ../lib/open303_DSP/LICENSE.txt

#include <m_pd.h>
#include <stdlib.h>

#include "rosic_BiquadFilter.h"
#include "rosic_EllipticQuarterBandFilter.h"
#include "rosic_OnePoleFilter.h"
#include "rosic_TeeBeeFilter.h"

#define OVERSAMPLING 4

static t_class *open303filter_class;

typedef struct _open303filter {
    t_object x_obj;
    t_inlet *x_inlet;
    t_inlet *x_inlet2;
    t_float x_sr;
    rosic::TeeBeeFilter x_filter;
    rosic::OnePoleFilter x_highpass1;
    rosic::OnePoleFilter x_highpass2;
    rosic::OnePoleFilter x_allpass;
    rosic::BiquadFilter x_notch;
    rosic::EllipticQuarterBandFilter x_antiAliasFilter;
} t_open303filter;

static void open303filter_reset(t_open303filter *x) {
    x->x_filter.reset();
    x->x_highpass1.reset();
    x->x_highpass2.reset();
    x->x_allpass.reset();
    x->x_notch.reset();
    x->x_antiAliasFilter.reset();
}

static t_int *open303filter_perform(t_int *w) {
    t_open303filter *x = (t_open303filter *)(w[1]);
    int n_samples = (int)(w[2]);
    t_float *in = (t_float *)(w[3]);
    t_float *cutoff = (t_float *)(w[4]);
    t_float *resonance = (t_float *)(w[5]);
    t_float *out = (t_float *)(w[6]);

    double tmp;
    for (int i = 0; i < n_samples; i++) {
        x->x_filter.setCutoff(cutoff[i], false);
        x->x_filter.setResonance(resonance[i] * 100.0, false);
        x->x_filter.calculateCoefficientsApprox4();

        for (int os = 1; os <= OVERSAMPLING; os++) {
            tmp = -in[i];                                  // the raw input signal
            tmp = x->x_highpass1.getSample(tmp);       // pre-filter highpass
            tmp = x->x_filter.getSample(tmp);          // now it's filtered
            tmp = x->x_antiAliasFilter.getSample(tmp); // anti-aliasing filtered
        }
        tmp = x->x_allpass.getSample(tmp);
        tmp = x->x_highpass2.getSample(tmp);
        tmp = x->x_notch.getSample(tmp);

        out[i] = tmp;
    }

    return w + 7;
}

static void open303filter_dsp(t_open303filter *x, t_signal **sp) {
    if(sp[0]->s_sr != x->x_sr){
        x->x_sr = sp[0]->s_sr;

        x->x_filter.setSampleRate(OVERSAMPLING * x->x_sr);
        x->x_highpass1.setSampleRate(OVERSAMPLING * x->x_sr);

        x->x_highpass2.setSampleRate(x->x_sr);
        x->x_allpass.setSampleRate(x->x_sr);
        x->x_notch.setSampleRate(x->x_sr);
    }
    dsp_add(open303filter_perform, 6, x, sp[0]->s_n, sp[0]->s_vec, sp[1]->s_vec, sp[2]->s_vec, sp[3]->s_vec);
}

static void open303filter_free(t_open303filter *x) {
    inlet_free(x->x_inlet);
    inlet_free(x->x_inlet2);
}

static void *open303filter_new(t_symbol *s, int ac, t_atom *av) {
    t_open303filter *x = (t_open303filter *)pd_new(open303filter_class);

    x->x_inlet = inlet_new((t_object *)x, (t_pd *)x, &s_signal, &s_signal);
    pd_float((t_pd *)x->x_inlet, 0.0);

    x->x_inlet2 = inlet_new((t_object *)x, (t_pd *)x, &s_signal, &s_signal);
    pd_float((t_pd *)x->x_inlet2, 0.0);

    x->x_filter.setMode(rosic::TeeBeeFilter::TB_303);
    x->x_highpass1.setMode(rosic::OnePoleFilter::HIGHPASS);
    x->x_highpass2.setMode(rosic::OnePoleFilter::HIGHPASS);
    x->x_allpass.setMode(rosic::OnePoleFilter::ALLPASS);
    x->x_notch.setMode(rosic::BiquadFilter::BANDREJECT);

    // tweakables:
    x->x_highpass1.setCutoff(44.486);
    x->x_highpass2.setCutoff(24.167);
    x->x_allpass.setCutoff(14.008);
    x->x_notch.setFrequency(7.5164);
    x->x_notch.setBandwidth(4.7);
    x->x_filter.setFeedbackHighpassCutoff(150.0);

    outlet_new(&x->x_obj, gensym("signal"));

    return x;
}


extern "C" void open303filter_tilde_setup(void){
    open303filter_class = class_new(
        gensym("open303filter~"), 
        (t_newmethod)open303filter_new,
        (t_method)open303filter_free,
        sizeof(t_open303filter), 
        0, 
        A_GIMME, 
        0
    );
    class_addmethod(open303filter_class, nullfn, gensym("signal"), A_NULL);
    class_addmethod(open303filter_class, (t_method)open303filter_dsp, gensym("dsp"), A_CANT,  0);
    class_addmethod(open303filter_class, (t_method)open303filter_reset, gensym("bang"), A_NULL, 0);
}
