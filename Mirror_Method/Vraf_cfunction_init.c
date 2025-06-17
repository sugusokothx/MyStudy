#include <math.h>
/* ---------- コンフィグ ------------------------------ */
#define ORDER 4               /* Butterworth 次数         */
#define N_STATE (ORDER)       /* = max(len(a),len(b)) - 1 */
static real_T zR[N_STATE];    /* IIR 状態（実部） */
static real_T zI[N_STATE];    /* IIR 状態（虚部） */
static uint32_T sampleIdx;    /* サンプルカウンタ */

/* ---------- 初期化 ---------------------------------- */
for (int i = 0; i < N_STATE; ++i) {
    zR[i] = 0.0;
    zI[i] = 0.0;
}
sampleIdx = 0U;
