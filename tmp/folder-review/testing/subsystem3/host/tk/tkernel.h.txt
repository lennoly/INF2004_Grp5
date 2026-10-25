/* Host-only kernel boundary. Not included by firmware builds. */
#ifndef S3_TEST_TKERNEL_H
#define S3_TEST_TKERNEL_H
#include <stdint.h>
#include <stddef.h>
typedef int32_t INT;
typedef uint32_t UINT;
typedef uint32_t UW;
typedef uint8_t UB;
typedef int32_t ID;
typedef int32_t ER;
typedef void (*FP)(INT, void *);
typedef struct { uint32_t hi; uint32_t lo; } SYSTIM;
typedef struct { UINT mbfatr; size_t bufsz; size_t maxmsz; } T_CMBF;
typedef struct {
    UINT tskatr; FP task; INT itskpri; size_t stksz;
} T_CTSK;
#define E_OK (0)
#define E_LIMIT (-5)
#define TMO_POL (0)
#define TMO_FEVR (-1)
#define TA_TFIFO (0u)
#define TA_HLNG (1u)
#define TA_RNG3 (0u)
#define DI(mask) do { (mask) = 0u; } while (0)
#define EI(mask) do { (void) (mask); } while (0)
ID tk_cre_mbf(T_CMBF const *p_cfg);
ID tk_cre_tsk(T_CTSK const *p_cfg);
ER tk_sta_tsk(ID task, INT code);
ER tk_del_tsk(ID task);
ER tk_del_mbf(ID buffer);
INT tk_rcv_mbf(ID buffer, void *p_dst, INT timeout);
ER tk_snd_mbf(ID buffer, void const *p_src, INT size, INT timeout);
ER tk_get_tim(SYSTIM *p_time);
ER tk_dly_tsk(uint32_t ms);
#endif
