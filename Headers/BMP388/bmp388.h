#ifndef  BMP388_H
#define BMP388_H

typedef struct {
    double temp;
    double press;
} pt_t;

bool bmp_init(void);
void get_pt(pt_t *out);
double get_po(pt_t *out, double n);
double alt_isothermal(double p, double p0, double temp_c);
double get_initial_alt(pt_t *out, double p0, double n);

#endif
