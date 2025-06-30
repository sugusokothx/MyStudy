/* === 入力ポート ============================================= */
real_T u_alpha = *u_alpha_1;   /* α軸入力 */
real_T u_beta  = *u_beta_2;    /* β軸入力 */
real_T Ts      = *Ts_3;        /* サンプリング周期 [s] */

/* === マスク・パラメータ ===================================== */
const real_T *b = b_coeff;     /* b[0]..b[4] */
const real_T *a = a_coeff;     /* a[0]=1 */
const real_T  w = omega;       /* 回転角速度 */
const real_T  gd = gd_sec;     /* 群遅延 [s] */
const int32_T seq = seqFlag;   /* ±1 */

/* ---------- 1) 回転して中心周波数を原点へ -------------------- */
real_T t  = (real_T)sampleIdx * Ts;
sampleIdx++;

real_T cos_wt = cos(w * t);
real_T sin_wt = sin(w * t);

/* u_tilde = u_p * exp(±j w t) （複素数を実部/虚部で扱う） */
real_T u_t_re, u_t_im;
if (seq > 0) {           /* 正相抽出: multiply by e^{-jωt} */
    u_t_re =  u_alpha * cos_wt + u_beta * sin_wt;
    u_t_im = -u_alpha * sin_wt + u_beta * cos_wt;
} else {                  /* 逆相抽出: multiply by e^{+jωt} */
    u_t_re =  u_alpha * cos_wt - u_beta * sin_wt;
    u_t_im =  u_alpha * sin_wt + u_beta * cos_wt;
}

/* ---------- 2) IIR LPF ― DF-II Transposed (1-sample) -------- */
auto iir_df2t = [](real_T x,
                   const real_T b[], const real_T a[],
                   real_T z[]) -> real_T {
    /* 4 次専用・最適化：状態長 = 4 */
    real_T v = x - a[1]*z[0] - a[2]*z[1] - a[3]*z[2] - a[4]*z[3];
    real_T y = b[0]*v + b[1]*z[0] + b[2]*z[1] + b[3]*z[2] + b[4]*z[3];
    /* 状態シフト */
    z[3] = z[2];
    z[2] = z[1];
    z[1] = z[0];
    z[0] = v;
    return y;
};

real_T x_re = iir_df2t(u_t_re, b, a, zR);
real_T x_im = iir_df2t(u_t_im, b, a, zI);

/* ---------- 3) 群遅延分だけ逆回転 ---------------------------- */
real_T cos_bk = cos(w * (t - gd));
real_T sin_bk = sin(w * (t - gd));

real_T y_alpha = x_re * cos_bk - x_im * sin_bk;
real_T y_beta  = x_re * sin_bk + x_im * cos_bk;

/* === 出力ポート ============================================= */
*y_alpha_1 = y_alpha;
*y_beta_2  = y_beta;
