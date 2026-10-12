#ifndef  BMP388_H
#define BMP388_H

typedef struct {
    double temp;
    double press;
} pt_t;

bool bmp_init(void);
void get_pt(pt_t *out);

#endif
