#ifndef M9N_UBX_H
#define M9N_UBX_H

// decoded data 
typedef struct {
    uint32_t iTOW;              //  0  ms
    uint16_t year;              //  4
    uint8_t  month, day, hour, min, sec;  // 6..10
    uint8_t  valid;             // 11
    uint32_t tAcc;              // 12  ns
    int32_t  nano;              // 16  ns
    uint8_t  fixType;           // 20
    uint8_t  flags;             // 21  bit0 = gnssFixOK
    uint8_t  flags2;            // 22
    uint8_t  numSV;             // 23
    int32_t  lon;               // 24  deg * 1e-7
    int32_t  lat;               // 28  deg * 1e-7
    int32_t  height;            // 32  mm
    int32_t  hMSL;              // 36  mm
    uint32_t hAcc;              // 40  mm
    uint32_t vAcc;              // 44  mm
    int32_t  velN, velE, velD;  // 48,52,56  mm/s
    int32_t  gSpeed;            // 60  mm/s
    int32_t  headMot;           // 64  deg * 1e-5
    uint32_t sAcc;              // 68  mm/s
    uint32_t headAcc;           // 72  deg * 1e-5
    uint16_t pDOP;              // 76  * 0.01
    int16_t  magDec;            // 88  deg * 1e-2
    uint16_t magAcc;            // 90  deg * 1e-2
} nav_pvt_t;

void gps_driver_init(void);
bool gps_update(nav_pvt_t *pvt);
int32_t iabs32(int32_t v);

#endif
