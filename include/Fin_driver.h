#pragma once

typedef struct{
    long double Fin_const;

} Fin_data_t;

typedef struct{
    long double a[0];
} Fin_polynomial_t;

void Fin_init();
float Fin_calculateAlpha(float pres, float temp, float vel, float force);