#include "Main.h"

extern volatile const double kkk_mg2;
extern double val_mg2 ;

#define kkk kkk_mg2
#define val val_mg2
        
void Main2(double u1, double u2, double* y1, double* y2, double* y3) {
    
    val = myAdd(u1, u2, kkk);  // ← ポインタを介して値を代入
    *y1 = val;
    *y2 = mySub(u1, u2);  // ← ここも同様
    *y3 = val*2;
}


extern volatile const double kkk_mg1;
extern double val_mg1;

#define kkk kkk_mg1
#define val val_mg1
        
void Main1(double u1, double u2, double* y1, double* y2, double* y3) {
    
    val = myAdd(u1, u2, kkk);  // ← ポインタを介して値を代入
    *y1 = val;
    *y2 = mySub(u1, u2);  // ← ここも同様
    *y3 = val*-2;
}