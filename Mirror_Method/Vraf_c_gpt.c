#include <math.h>

// Filter parameters
#define ORDER 4
#define MAX_ORDER (ORDER+1)

// VRAF Filter structure
typedef struct {
    float b[MAX_ORDER];   // numerator coefficients
    float a[MAX_ORDER];   // denominator coefficients
    float zR[MAX_ORDER];  // filter state for real part
    float zI[MAX_ORDER];  // filter state for imag part
    float omega;
    float Fs;
    int seq;              // +1: pos, -1: neg
    unsigned long t_idx;
} VRAF;

// Initialize filter structure
void VRAF_init(VRAF *filt, float f_shift, float Fs, int seq, const float *b_coeff, const float *a_coeff) {
    filt->omega = 2.0f * 3.14159265359f * f_shift;
    filt->Fs = Fs;
    filt->seq = seq;
    filt->t_idx = 0;
    for (int i = 0; i < MAX_ORDER; i++) {
        filt->b[i] = b_coeff[i];
        filt->a[i] = a_coeff[i];
        filt->zR[i] = 0.0f;
        filt->zI[i] = 0.0f;
    }
}

// Single-step IIR filter (Direct Form II)
float iir_filter_step(float input, float *b, float *a, float *state) {
    float output = b[0]*input + state[0];
    for (int i = 1; i < ORDER; i++) {
        state[i-1] = b[i]*input + state[i] - a[i]*output;
    }
    state[ORDER-1] = b[ORDER]*input - a[ORDER]*output;
    return output;
}

// Process single sample
void VRAF_process(VRAF *filt, float u_alpha, float u_beta, float *y_alpha, float *y_beta) {
    float t = filt->t_idx / filt->Fs;
    filt->t_idx++;

    // Rotation angle
    float angle = filt->omega * t;

    float cos_ang = cos(angle);
    float sin_ang = sin(angle);

    float u_tilde_real, u_tilde_imag;
    float rot_back_real, rot_back_imag;

    if (filt->seq == 1) {  // positive-sequence
        u_tilde_real = u_alpha*cos_ang + u_beta*sin_ang;
        u_tilde_imag = -u_alpha*sin_ang + u_beta*cos_ang;
        rot_back_real = cos_ang;
        rot_back_imag = sin_ang;
    } else {               // negative-sequence
        u_tilde_real = u_alpha*cos_ang - u_beta*sin_ang;
        u_tilde_imag = u_alpha*sin_ang + u_beta*cos_ang;
        rot_back_real = cos_ang;
        rot_back_imag = -sin_ang;
    }

    // Apply IIR LPF separately
    float xR = iir_filter_step(u_tilde_real, filt->b, filt->a, filt->zR);
    float xI = iir_filter_step(u_tilde_imag, filt->b, filt->a, filt->zI);

    // Rotate back
    *y_alpha = xR * rot_back_real - xI * rot_back_imag;
    *y_beta  = xR * rot_back_imag + xI * rot_back_real;
}
