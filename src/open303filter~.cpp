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
    t_inlet *cutoff_inlet;
    t_inlet *resonance_inlet;
    t_float current_sample_rate;
    rosic::TeeBeeFilter filter;
    rosic::OnePoleFilter highpass1;
    rosic::OnePoleFilter highpass2;
    rosic::OnePoleFilter allpass;
    rosic::BiquadFilter notch;
    rosic::EllipticQuarterBandFilter anti_alias_filter;
} t_open303filter;

static void open303filter_reset(t_open303filter *x) {
    x->filter.reset();
    x->highpass1.reset();
    x->highpass2.reset();
    x->allpass.reset();
    x->notch.reset();
    x->anti_alias_filter.reset();
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
        x->filter.setCutoff(cutoff[i], false);
        x->filter.setResonance(resonance[i] * 100.0, false);
        x->filter.calculateCoefficientsApprox4();

        for (int os = 1; os <= OVERSAMPLING; os++) {
            tmp = -in[i];                                  // the raw input signal
            tmp = x->highpass1.getSample(tmp);       // pre-filter highpass
            tmp = x->filter.getSample(tmp);          // now it's filtered
            tmp = x->anti_alias_filter.getSample(tmp); // anti-aliasing filtered
        }
        tmp = x->allpass.getSample(tmp);
        tmp = x->highpass2.getSample(tmp);
        tmp = x->notch.getSample(tmp);

        out[i] = tmp;
    }

    return w + 7;
}

static void open303filter_dsp(t_open303filter *x, t_signal **sp) {
    if (sp[0]->s_sr != x->current_sample_rate) {
        x->current_sample_rate = sp[0]->s_sr;

        x->filter.setSampleRate(OVERSAMPLING * x->current_sample_rate);
        x->highpass1.setSampleRate(OVERSAMPLING * x->current_sample_rate);

        x->highpass2.setSampleRate(x->current_sample_rate);
        x->allpass.setSampleRate(x->current_sample_rate);
        x->notch.setSampleRate(x->current_sample_rate);
    }
    dsp_add(open303filter_perform, 6, x, sp[0]->s_n, sp[0]->s_vec, sp[1]->s_vec, sp[2]->s_vec, sp[3]->s_vec);
}

static void open303filter_free(t_open303filter *x) {
    inlet_free(x->cutoff_inlet);
    inlet_free(x->resonance_inlet);
}

static void *open303filter_new(t_symbol *s, int ac, t_atom *av) {
    t_open303filter *x = (t_open303filter *)pd_new(open303filter_class);

    x->cutoff_inlet = inlet_new((t_object *)x, (t_pd *)x, &s_signal, &s_signal);
    pd_float((t_pd *)x->cutoff_inlet, 0.0);

    x->resonance_inlet = inlet_new((t_object *)x, (t_pd *)x, &s_signal, &s_signal);
    pd_float((t_pd *)x->resonance_inlet, 0.0);

    x->filter.setMode(rosic::TeeBeeFilter::TB_303);
    x->highpass1.setMode(rosic::OnePoleFilter::HIGHPASS);
    x->highpass2.setMode(rosic::OnePoleFilter::HIGHPASS);
    x->allpass.setMode(rosic::OnePoleFilter::ALLPASS);
    x->notch.setMode(rosic::BiquadFilter::BANDREJECT);

    // tweakables:
    x->highpass1.setCutoff(44.486);
    x->highpass2.setCutoff(24.167);
    x->allpass.setCutoff(14.008);
    x->notch.setFrequency(7.5164);
    x->notch.setBandwidth(4.7);
    x->filter.setFeedbackHighpassCutoff(150.0);

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
