/* Host-test stand-in for the micro T-Kernel 3.0 API subset used by
   vehicle.c and obstacle.c (types and prototypes as in the template's
   include/tk/typedef.h and syscall.h, with W pinned to 32 bits because long
   is 64-bit on the host).  Implemented by sim/vehicle_sim.c. */
#ifndef TK_TKERNEL_H
#define TK_TKERNEL_H

#include <stddef.h>
#include <stdint.h>

typedef int32_t      W;
typedef uint32_t     UW;
typedef uint8_t      UB;
typedef int          INT;
typedef unsigned int UINT;
typedef W            SZ;
typedef INT          ID;
typedef UW           ATR;
typedef INT          ER;
typedef INT          PRI;
typedef W            TMO;
typedef UW           RELTIM;

/* The kernel's FP is unprototyped; tasks are its only use here. */
typedef void (*FP)(INT stacd, void * p_exinf);

typedef struct
{
    W  hi;
    UW lo;
} SYSTIM;

typedef struct
{
    void * exinf;
    ATR    tskatr;
    FP     task;
    PRI    itskpri;
    SZ     stksz;
    void * bufptr;
} T_CTSK;

typedef struct
{
    void * exinf;
    ATR    mbfatr;
    SZ     bufsz;
    INT    maxmsz;
    void * bufptr;
} T_CMBF;

#define TA_HLNG  (0x00000001u)
#define TA_RNG3  (0x00000300u)
#define TA_TFIFO (0x00000000u)
#define TMO_POL  (0)
#define TMO_FEVR (-1)
#define E_OK     (0)
#define E_LIMIT  (-34)
#define E_TMOUT  (-50)

/* One host thread: interrupt masking is a no-op. */
#define DI(intsts) ((intsts) = 0u)
#define EI(intsts) ((void) (intsts))

ID  tk_cre_tsk(T_CTSK const * pk_ctsk);
ER  tk_sta_tsk(ID tskid, INT stacd);
ER  tk_dly_tsk(RELTIM dlytim);
ID  tk_cre_mbf(T_CMBF const * pk_cmbf);
ER  tk_snd_mbf(ID mbfid, void const * msg, INT msgsz, TMO tmout);
INT tk_rcv_mbf(ID mbfid, void * msg, TMO tmout);
ER  tk_get_tim(SYSTIM * pk_tim);

#endif /* TK_TKERNEL_H */
