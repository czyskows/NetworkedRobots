/*
 * HOWTO write an External for Pure data
 * (c) 2001-2006 IOhannes m zm�lnig zmoelnig[AT]iem.at
 *
 * this is the source-code for the fourth example in the HOWTO
 * it creates a simple dsp-object:
 * 2 input signals are mixed into 1 output signal
 * the mixing-factor can be set via the 3rd inlet
 *
 * for legal issues please see the file LICENSE.txt
 */


/**
 * include the interface to Pd 
 */
#include "m_pd.h"
#include <math.h>


/**
 * define a new "class" 
 */
static t_class *xcorr_tilde_class;


/**
 * this is the dataspace of our new object
 * the first element is the mandatory "t_object"
 * f_xcorr denotes the mixing-factor
 * "f" is a dummy and is used to be able to send floats AS signals.
 */
typedef struct _xcorr_tilde {
  t_object  x_obj;
  t_sample f_xcorr;
  t_sample f;

  t_inlet *x_in2;
  t_inlet *x_in3;
  t_outlet*x_out;
} t_xcorr_tilde;


/**
 * this is the core of the object
 * this perform-routine is called for each signal block
 * the name of this function is arbitrary and is registered to Pd in the 
 * xcorr_tilde_dsp() function, each time the DSP is turned on
 *
 * the argument to this function is just a pointer within an array
 * we have to know for ourselves how many elements inthis array are
 * reserved for us (hint: we declare the number of used elements in the
 * xcorr_tilde_dsp() at registration
 *
 * since all elements are of type "t_int" we have to cast them to whatever
 * we think is apropriate; "apropriate" is how we registered this function
 * in xcorr_tilde_dsp()
 */
t_int *xcorr_tilde_perform(t_int *w)
{
  /* the first element is a pointer to the dataspace of this object */
  t_xcorr_tilde *x = (t_xcorr_tilde *)(w[1]);
  /* here is a pointer to the t_sample arrays that hold the 2 input signals */
  t_sample  *in1 =    (t_sample *)(w[2]);
  t_sample  *in2 =    (t_sample *)(w[3]);
  /* here comes the signalblock that will hold the output signal */
  t_sample  *out =    (t_sample *)(w[4]);
  /* all signalblocks are of the same length */
  int          n =           (int)(w[5]);
  /* get (and clip) the mixing-factor */
  t_sample f_xcorr = (x->f_xcorr<0)?0.0:(x->f_xcorr>1)?1.0:x->f_xcorr;
  /* just a counter */
  int i, j;

  t_sample in1R[n/2+1];
  t_sample in1I[n/2+1];
  t_sample in2R[n/2+1];
  t_sample in2I[n/2+1];

  t_sample in11[n/2+1];
  t_sample in22[n/2+1];

  t_sample in12R[n/2+1];
  t_sample in12I[n/2+1];

  t_sample in11C[n];
  t_sample in22C[n];
  t_sample in12C[n];

  // Calculate ffts
  mayer_realfft(n, in1);
  mayer_realfft(n, in2);

  // unpack mayer_realfft results into R&I arrays
  for(i=0; i<=n/2; i++) {
    in1R[i] = in1[i];
    in2R[i] = in2[i];
  }
  
  in1I[0]=0;  // DC
  in2I[0]=0;  // DC
		
  for(i=(n-1), j=1; i>n/2; i--, j++) {
    in1I[j] = in1[i];
    in2I[j] = in2[i];
  }
		
  in1I[n/2]=0;  // Nyquist
  in2I[n/2]=0;  // Nyquist

  // Calculate products
  
  for(i=0; i<=n/2; i++) {
    in11[i] = in1R[i]*in1R[i]+in1I[i]*in1I[i];
    in22[i] = in2R[i]*in2R[i]+in2I[i]*in2I[i];

    in12R[i] = in1R[i]*in2R[i]+in1I[i]*in2I[i];
    in12I[i] = in1I[i]*in2R[i]-in1R[i]*in2I[i];
  }

    // pack real and imaginary parts in correct order for mayer_realifft
  for(i=0; i<=n/2; i++) {
    in11C[i] = in11[i];
    in22C[i] = in22[i];
    in12C[i] = in12R[i];
  }
  
  for(i=(n/2+1), j=(n/2-1); i<n; i++, j--) {
    in11C[i] = 0;
    in22C[i] = 0;
    in12C[i] = in12I[j];
  }
			
  // inverse fft
  mayer_realifft(n, in11C);
  mayer_realifft(n, in22C);
  mayer_realifft(n, in12C);

  t_sample AC = sqrt(in11C[0])*sqrt(in22C[0]);
  
  // shuffle xc
  // [xc(n/2 : -1 : 0), xc(n - 1: -1 : n/2)]
  t_sample max = in12C[n/2]/AC;
  int max_index = 0;
  for(i=0, j=n/2; i<=n/2; i++, j--) {
    out[i] = in12C[j]/AC;
    if (out[i] > max) {
      max = out[i];
      max_index = i - n/2;
    }
  }
  for(i=n/2+1, j=n-1; i<n; i++, j--) {
    out[i] = in12C[j]/AC;
    if (out[i] > max) {
      max = out[i];
      max_index = i - n/2;
    }
  }
  
  /* return a pointer to the dataspace for the next dsp-object */
  return (w+6);
}


/**
 * register a special perform-routine at the dsp-engine
 * this function gets called whenever the DSP is turned ON
 * the name of this function is registered in xcorr_tilde_setup()
 */
void xcorr_tilde_dsp(t_xcorr_tilde *x, t_signal **sp)
{
  /* add xcorr_tilde_perform() to the DSP-tree;
   * the xcorr_tilde_perform() will expect "5" arguments (packed into an
   * t_int-array), which are:
   * the objects data-space, 3 signal vectors (which happen to be
   * 2 input signals and 1 output signal) and the length of the
   * signal vectors (all vectors are of the same length)
   */
  dsp_add(xcorr_tilde_perform, 5, x,
          sp[0]->s_vec, sp[1]->s_vec, sp[2]->s_vec, sp[0]->s_n);
}

/**
 * this is the "destructor" of the class;
 * it allows us to free dynamically allocated ressources
 */
void xcorr_tilde_free(t_xcorr_tilde *x)
{
  /* free any ressources associated with the given inlet */
  inlet_free(x->x_in2);
  inlet_free(x->x_in3);

  /* free any ressources associated with the given outlet */
  outlet_free(x->x_out);
}

/**
 * this is the "constructor" of the class
 * the argument is the initial mixing-factor
 */
void *xcorr_tilde_new(t_floatarg f)
{
  t_xcorr_tilde *x = (t_xcorr_tilde *)pd_new(xcorr_tilde_class);

  /* save the mixing factor in our dataspace */
  x->f_xcorr = f;
  
  /* create a new signal-inlet */
  x->x_in2 = inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);
  x->x_in3 = inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);

  /* create a new signal-outlet */
  x->x_out = outlet_new(&x->x_obj, &s_signal);

  return (void *)x;
}


/**
 * define the function-space of the class
 * within a single-object external the name of this function is very special
 */
void xcorr_tilde_setup(void) {
  xcorr_tilde_class = class_new(gensym("xcorr~"),
        (t_newmethod)xcorr_tilde_new,
        (t_method)xcorr_tilde_free,
	sizeof(t_xcorr_tilde),
        CLASS_DEFAULT, 
        A_DEFFLOAT, 0);

  /* whenever the audio-engine is turned on, the "xcorr_tilde_dsp()" 
   * function will get called
   */
  class_addmethod(xcorr_tilde_class,
        (t_method)xcorr_tilde_dsp, gensym("dsp"), 0);
  /* if no signal is connected to the first inlet, we can as well 
   * connect a number box to it and use it as "signal"
   */
  CLASS_MAINSIGNALIN(xcorr_tilde_class, t_xcorr_tilde, f);
}
