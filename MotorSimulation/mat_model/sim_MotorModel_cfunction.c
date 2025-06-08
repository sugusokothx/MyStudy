/**********************************************************************
 *  PMSM dq-axis model                                               *
 *  ──────────────────────────────────────────────────────────────── *
 *  Discrete-time (Euler) implementation for the Simulink            *
 *  **C Function** block.                                            *
 *                                                                   *
 *  Inputs  : v_d, v_q  – stator voltage [V]                         *
 *            omega      – electrical speed [rad/s]                  *
 *  Outputs : *i_d, *i_q – stator current  [A]                       *
 *            *Te        – electromagnetic torque [N·m]              *
 *  State   : psi_d, psi_q – stator flux linkage [Wb]                *
 *********************************************************************/

#include <math.h>

/* ── 1)  user-defined constants / tables  ───────────────────────── */
#define NX 201              /* length of ψd axis */
#define NY 201              /* length of ψq axis */

extern const double Rs;           /* stator resistance  [Ω] */
extern const double Pn;           /* pole-pair number   [ –] */
extern const double Ts;           /* sample time        [s] */

extern const double psi_d_axis[NX];      /* 1 × NX          */
extern const double psi_q_axis[NY];      /* 1 × NY          */
extern const double IdTable[NY][NX];     /* NY × NX         */
extern const double IqTable[NY][NX];     /* NY × NX         */

/* ── 2)  forward declaration of helper -- bilinear interpolation  ― */
static double table2D(double x, double y,
                      const double *xAxis,
                      const double *yAxis,
                      const double tbl[NY][NX]);

/* ── 3)  main model function   ──────────────────────────────────── */
void pmsm_dq_model(double v_d, double v_q, double omega,
                   double *i_d, double *i_q, double *Te)
{
    /* persistent (static) flux state */
    static double psi_d = 0.0;
    static double psi_q = 0.0;

    /* 3-1) flux → current (lookup) */
    *i_d = table2D(psi_d, psi_q, psi_d_axis, psi_q_axis, IdTable);
    *i_q = table2D(psi_d, psi_q, psi_d_axis, psi_q_axis, IqTable);

    /* 3-2) flux derivatives  (dψ/dt = v – R·i + ω·J·ψ) */
    double dpsi_d = v_d - Rs * (*i_d) - omega * psi_q;
    double dpsi_q = v_q - Rs * (*i_q) + omega * psi_d;

    /* 3-3) Euler integrate */
    psi_d += Ts * dpsi_d;
    psi_q += Ts * dpsi_q;

    /* 3-4) update currents with new flux */
    *i_d = table2D(psi_d, psi_q, psi_d_axis, psi_q_axis, IdTable);
    *i_q = table2D(psi_d, psi_q, psi_d_axis, psi_q_axis, IqTable);

    /* 3-5) electromagnetic torque */
    *Te = Pn * (psi_d * (*i_q) - psi_q * (*i_d));
}

/* ───────────────────────────────────────────────────────────────── */
/*  bilinear interpolation (no extrapolation)                       */
static double table2D(double x, double y,
                      const double *xAxis,
                      const double *yAxis,
                      const double tbl[NY][NX])
{
    /* --- locate surrounding grid indices ------------------------ */
    int ix = 0;
    while (ix < NX - 2 && xAxis[ix + 1] <= x) ++ix;

    int iy = 0;
    while (iy < NY - 2 && yAxis[iy + 1] <= y) ++iy;

    /* --- fractional distances ----------------------------------- */
    double dx = (x - xAxis[ix]) / (xAxis[ix + 1] - xAxis[ix]);
    double dy = (y - yAxis[iy]) / (yAxis[iy + 1] - yAxis[iy]);

    /* --- four-point blend --------------------------------------- */
    double v11 = tbl[iy    ][ix    ];
    double v21 = tbl[iy    ][ix + 1];
    double v12 = tbl[iy + 1][ix    ];
    double v22 = tbl[iy + 1][ix + 1];

    return (1.0 - dx)*(1.0 - dy)*v11 +
             dx *(1.0 - dy)*v21 +
           (1.0 - dx)*  dy *v12 +
             dx *  dy *v22;
}
