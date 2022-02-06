/* 
 * DG Nova
 * =======
 *
*/

#define VCDBG(x)
#define TRACE(x)  if(Trace){x;}

#define VER_VER   0
#define VER_REL   0
#define VER_LVL   1

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <time.h>                               /* nanosleep()    */
#include <sys/file.h>                           /* flock()        */
#include <sys/time.h>                           /* gettimeofday() */
#include <unistd.h>                             /* usleep()       */

#include "curterm.h"

typedef unsigned char Byte;
typedef unsigned short Word;
typedef unsigned int DWord;
typedef enum{false,true} Flag;                  /* Boolean */
typedef unsigned long Tyme;                     /* time measured in 0.1us */

#define FOREVER   ((((unsigned int)-1) >> 1) / 10)


#define ABS(x)    ((x)<0? -(x):(x))

#if 1
#define SWAP(x)   x
#else
Word SWAP(Word x)
{
   return ((x & 0377) << 8) + (x >> 8);
}
#endif

void Assert(char *path,int lno, char *msg, int cond)
{
   if (!cond) {
      fprintf(stderr,"assert at %s:%d: %s\n",path,lno,msg);
      exit(1);
   }
}
#define ASSERT(cond) Assert(__FILE__,__LINE__,""#cond,cond)

Word Trace;          /* trace instr. execution         */
enum {
   H_Run,
   H_Startup,
   H_Halt,
   H_Break,
   H_Undef,
   H_Indir,
   H_MisDev,
   H_IOErr,
   H_ReadOnly,
   H_VCon
} Halt;

char *halts[] = {
   "run",
   "system startup",
   "halt",
   "break",
   "undefined instr/operation",
   "indirection chain",
   "missing device",
   "I/O error",
   "drive read-only",
   "virtual console"
};

void SetHalt(char *path,int lno,int n)
{
   TRACE(printf("%s:%d: Halt = %d\n",path,lno,n));
   Halt = n;
}

#define HALT(x)   SetHalt(__FILE__,__LINE__,x)

typedef struct _Option {
   Byte devno;
   char *key;
   char *value;
} Option;

void devClear(Byte devno);                      /* Forward declarations */
void devStart(Byte devno);
void devPulse(Byte devno);
void doIO(int dt);
void disasm(Word pc,Word IR);
char *io_devs(Byte devno);
Word memReadIO(Word adr);
void memWriteIO(Word adr, Word w);
void showsym(int vc);

Word AC[4], PC;                                 /* registers: ACs, PC */
Flag CRY;                                       /* carry */

Word M[32768];                                  /* memory */
int  T[32768];
Byte* B;                                        /* byte addressed memory */

/* --- D E V I C E S -------------------------------------------------------- */

#define SEC_SIZE  256
#define SEC_BYTES 512
#define NDEV      0100

#define DEV_MDV   (001)
#define DEV_TTI   (010)
#define DEV_TTO   (011)
#define DEV_PTR   (012)
#define DEV_PTP   (013)
#define DEV_RTC   (014)
#define DEV_LPT   (017)
#define DEV_DSK   (020)
#define DEV_DKP   (033)
#define DEV_CPU   (077)

#define HZ        50

#define BUF_A     0
#define BUF_B     1
#define BUF_C     2

#define NO_LINE   ((Byte)255)

typedef enum {IOC_NONE, IOC_START, IOC_CLEAR, IOC_PULSE} CtrlFn;
typedef enum {LPT_IDLE,LPT_XMIT,LPT_FULL,LPT_CTRL} PrinterOp;

#define LPT_NBUF     24
#define LPT_NZONE     6
#define LPT_CHBUF    buf[BUF_A]

#define DSK_DTS      buf[BUF_A]
#define DSK_ACNT     buf[BUF_B] 
#define DSK_E_NODISC 04
#define DSK_E_WRITE  020
#define DSK_E_ERROR  01

#define DKP_CSR      buf[BUF_A]
#define DKP_MAC      buf[BUF_B]
#define DKP_DAR      buf[BUF_C]
#define DKP_NSEC     4872

#define DKP_DRV(x)   ((x) >> 14)
#define DKP_HED(x)   (((x) >> 8) & 037)
#define DKP_SEC(x)   (((x) >> 4) & 017)
#define DKP_SCNT(x)  ((x) & 017)

#define DKP_MOD(x)   (((x) >> 8) & 03)
#define DKP_CYL(x)   ((x) & 0377)

/* status flags */
#define DKP_DONE_RW       0100000
#define DKP_DONE_SEEK0    0040000
#define DKP_DONE_SEEK1    0020000
#define DKP_DONE_SEEK2    0010000
#define DKP_DONE_SEEK3    0004000

#define DKP_DONE_SEEK(u)  (1 << (3 - (u) + 11))

#define DKP_SEEKING0 0002000
#define DKP_SEEKING1 0001000
#define DKP_SEEKING2 0000400
#define DKP_SEEKING3 0000200

#define DKP_SEEKING(u)  (1 << (3 - (u) + 7))

#define DKP_RDY      0000100
#define DKP_E_ERROR  0000001
#define DKP_E_SEEK   (0000040 | DKP_E_ERROR)
#define DKP_E_END    (0000020 | DKP_E_ERROR)
#define DKP_E_UNSAFE (0000010 | DKP_E_ERROR)
#define DKP_E_CHK    (0000004 | DKP_E_ERROR)
#define DKP_E_LATE   (0000002 | DKP_E_ERROR)

/* mode */
#define DKP_MREAD    0
#define DKP_MWRITE   1
#define DKP_MSEEK    2
#define DKP_MRECAL   3

#define DKP_GEO_CC   203
#define DKP_GEO_HH     2
#define DKP_GEO_SS    12
#define DKP_GEO_HHSS (DKP_GEO_HH * DKP_GEO_SS)

typedef struct _Event {
   Byte devno;       /* device number */
   Byte uno;         /* unit number */
   CtrlFn ctrlfn;    /* control fn, which started the operation */
   int  timeout;     /* when reaches 0, done is set */
   struct _Event *que;  /* next event in the queue  */
} Event;

#define NEVENT 256
Event events[NEVENT];

typedef struct _Device {
   Byte devno;        /* device code */
   Word buf[3];
   Flag busy, done;  /* device status: 00 - idle, 10 - working, 01 - done */
   Flag intdis;      /* INT disable flag */
   Flag intreq;      /* INT request */
   Byte line;        /* bus line: 0-15, NO_LINE - missing device */
   int  delay;       /* device delay */
   struct _Device *bus; /* next device on I/O bus */
   struct {
      char  *path;   /* storage file names */
      FILE  *fd;     /* file handles       */
      off_t nsecs;   /* num. of sectors/drive */
      Flag  rdonly;  /* read-only drives   */
      Word  buf[3];  /* saved regs at Start/Pulse */
      Tyme  lastRW;  /* last R/W time, Seek/Recalibrate clears it */
      Word  lastSEC; /* last sector R/W on a surface */
   } units[4];
   Word status;      /* controller status  */
   Flag shared;      /* shared drive, use locking */
   Flag locked;      /* signals drive locked      */
   Tyme unlockTO;    /* unlock timeout for locked devs */
   Word *mem;        /* memory image   */
   union {
      struct {
         Byte zone;     /* current zone: 1-LPT_NZONE */
         Byte pos;      /* buffer position   */
         Byte buf[LPT_NBUF];/* zone buffer  */
         unsigned long lines;
         PrinterOp op;  /* LPT operation */
      } LPT;
      struct {
         Tyme lastIO;   /* last Start/Pulse  */
         Tyme lastDOA;  /* last buffer A write  */
         Word lastDTS;  /* last sector       */
         Byte protect;  /* track protection (1bit for 16 tracks) */
      } DSK;
      struct {
         Word dummy;
      } DKP;
   } u;
} Device;

#define ASC_LF  012
#define ASC_FF  014
#define ASC_CR  015

Tyme elapsedTime; /* elapsed time 0.1us */
Tyme memTime;     /* memory time */
Flag memCycle;    /* cycle exact memory time */
Tyme ioTime;      /* updated at every I/O Start/Pulse */
                  /* all QUE timeouts are relative to this time */
Tyme eventTime;   /* next event time in QUE */
unsigned long numInstr;    /* number of instructions executed */

Tyme usince(Tyme base)
{
   return (elapsedTime - base) / 10;
}

Device *BUS;         /* I/O bus, fastest device(s) first */
Event *QUE;          /* event queue, most recent device event first */
int IREQ;            /* no. of pending INT requests */
Flag delayION;       /* delayed ION */
Device devs[64];
Flag MissingDev;     /* allow/disallow missing devices */

int  nBKPT;  /* breakpoints */
#define MAX_BKPT   8
Word BKPT[MAX_BKPT];
Word BRK[2048];

#define BRAKE(x)  (BRK[(x)>>4] & (1 << ((x) & 15)))

Event *allocEVT(void)
{
   int steps;
   static int pos = 0;                          /* start search from here  */

   steps = 0;                                   /* init steps  */
   while ((steps < NEVENT) &&                   /* 1 cycle should not lapse */
          0 != events[pos & (NEVENT - 1)].devno)/*    AND devno be clear    */
   {
      pos++;                                    /* check next post   */
      steps++;                                  /* count steps       */
   }
   ASSERT(steps != NEVENT);                     /* ALL EVENTS USED!  */

   return &events[pos++ & (NEVENT - 1)];        /* return event ptr  */
}

void clearEVT(Event *evt)
{
   evt->devno = 0;
   evt->que   = NULL;
}

/* return prev.device where t < dev->delay */
Device* insertBUS(int t)
{
   Device *p, *q;

   p = NULL; q = BUS;
   while (q) {
      if (t < q->delay)
         break;
      p = q;
      q = q->bus;
   }
   return p;
}

/* return prev.device where t < dev->timeout */
Event* insertQUE(int t)
{
   Event *p, *q;

   p = NULL; q = QUE;
   while (q) {
      if (t < q->timeout)
         return p;
      p = q;
      q = q->que;
   }
   return p;
}

/* return pointer to previous device in the queue */
Event* removeQUE(Byte devno)
{
   Event *p, *q;

   p = NULL; q = QUE;
   while (q) {
      if (devno == q->devno)
         return p;
      p = q;
      q = q->que;
   }
   
   /* NB. if p is NULL then either QUE is empty, or devno not found! */

   return p;
}

void IORST(void)                                /* IORST */
{
   Device *q;
   int i;

   q = BUS;                                     /* clear ctrl FFs in all devs */
   while (q) {
      devClear(q->devno);                       /* Clear dev (intreq cleared) */
      q->buf[0] = q->buf[1] = q->buf[2] = 0;    /* clear buffers    */
      q->intdis = false;                        /* clear IntDisable */
      q = q->bus;                               /* next device on BUS */
   }

   for (i = 0; i < NEVENT; i++)                 /* clear event QUE   */
      clearEVT(&events[i]);
   QUE = NULL;
}


static char *io_regs[] = {"A", "B", "C", "?"};
static char *ctrlfns[] = {" NONE", "START", "CLEAR", "PULSE"};

Word devIN(Byte devno, int reg)                 /* Device INPUT */
{
   Word w;
   Device *q, *dev;

   dev = &devs[devno];                          /* point to device*/
   w = dev->buf[reg];                           /* default action */

   switch (devno) {
   case DEV_CPU:                                /* --- CPU ------------- */
      switch (reg) {
      case BUF_A:                               /* READS, see def action */
         break;
      case BUF_B:                               /* INTA */
         w = 0; q = BUS;                        /* 1st request on the bus */
         while (q) {
            if (q->intreq) {
               /* "The INT request signal is a level so once synchronized
                * it remains on the bus until the program clears Done or
                * sets Interrupt Disable.
               */
               w = q->devno;
               break;
            }
            q = q->bus;
         }
         break;
      case BUF_C:                               /* IORST */
         IORST();
         break;
      }
      break;
   case DEV_LPT:                                /* --- LPT ------- */
      if (BUF_A == reg)                         /* read status     */
         w = NULL == dev->units[0].fd? 0 : 1;   /* printer online? */
      break;
   case DEV_DSK:
      switch (reg) {
      case BUF_A:
         w = dev->status;                       /* DSK status reg */
         break;
      case BUF_B:
         break;
      case BUF_C:
         if (dev->DSK_DTS & 0100000)
            fprintf(stderr,"%06o/%06o Disc Maintenance!\n",PC,M[PC]);
         break;
      }
      break;
   case DEV_DKP:                                /* DKP?                */
      {  int uno;
         Flag rdy;

         uno = DKP_DRV(dev->DKP_CSR);           /* calc. unit number */
         switch (reg) {
         case BUF_A:
            rdy = true;
            if (dev->units[uno].fd == NULL      /* no disc           */
               || dev->busy                     /* doing Read/Write  */
               || (dev->status & DKP_SEEKING(uno)) /* Seeking on drive  */
            )
               rdy = false;
            if (rdy) dev->status |= DKP_RDY;
            else dev->status &= ~DKP_RDY;
            w = dev->status;                    /* DKP status register */
            break;
         case BUF_B:                            /* read address counter */
            break;
         case BUF_C:
            w = dev->units[uno].DKP_DAR;        /* read unit DAR status */
            break;
         }
      }
      break;
   }

   TRACE(printf("IODATI %s=%06o\n",io_regs[reg],w));
   return w;
}

void devOUT(Byte devno, int reg, Word w)        /* Device OUTPUT */
{
   Device *q, *dev;

   dev = &devs[devno];                          /* point to device   */
   if (NO_LINE != dev->line) {                  /* device installed? */
      dev->buf[reg] = w;                        /* set buffer        */
      TRACE(printf("IODATO %s=%06o\n",io_regs[reg],w));
   }

   switch (devno) {
   case DEV_CPU:                                /* CPU? */
      switch (reg) {
      case BUF_A: break;
      case BUF_B:                               /* MSKO */
         q = BUS;                               /* setup IntDisable flags */
         while (q) {
            q->intdis = w & (1 << q->line)? true : false;   /* set flag */
            if (q->done && false == q->intdis) {/* Done=1 & IntDis=0 ?  */
               if (false == q->intreq) {        /* no pending INT req ? */
                  q->intreq = true; IREQ++;     /* issue INT request    */
               }
            }
            q = q->bus;
         }
         break;
      case BUF_C:                               /* HALT */
         HALT(H_Halt);                          /* Halt the processor. */
         break;
      }
      break;
   case DEV_DSK:                                /* DSK? */
      if (BUF_A == reg)
         dev->u.DSK.lastDOA = elapsedTime;
      if (BUF_B == reg && (0100000 & w))        /* check diag mode */
         fprintf(stderr,"%06o / %06o DSK Diagnostic Mode!\n",PC,M[PC]);
      break;
   case DEV_DKP:
      if (BUF_A == reg)
         dev->status &= ~(0174000 & w);         /* clear status bits */
      break;
   }
}

void enqueIO(Device *dev, int uno,  CtrlFn fn)  /* Enqueue I/O operation */
{
   Event *q, *evt;

   ASSERT(NO_LINE != dev->line);                /* ensure device connected */
   ASSERT(0 < dev->delay);                      /* has delay set         */
   ASSERT(IOC_CLEAR != fn);                     /* only None/Start/Pulse */

   evt = allocEVT();

   evt->devno   = dev->devno;                   /* set device no  */
   evt->uno     = uno;                          /* set unit no    */
   evt->ctrlfn  = fn;                           /* set ctrl function */
   evt->timeout = 10 * dev->delay;              /* set timeout in 0.1us */

   if (QUE)                                     /* QUE not empty?   */ 
      doIO(elapsedTime - ioTime);               /* step all devs[]  */

   q = insertQUE(evt->timeout);                 /* get insert pos */
   if (NULL == q) {                             /* be the first one */
      evt->que = QUE;                           /* insert dev */
      QUE = evt;
   }
   else {
      evt->que = q->que;                        /* insert after `q' */
      q->que = evt;
   }
   ioTime = elapsedTime;                        /* reset ioTime     */
   eventTime = ioTime + QUE->timeout;           /*    and eventTime */
}

int lockDEV(Device *dev)
{
   /* NB. always lock unit0, because the controller is locked!    */

   ASSERT(dev->shared);                         /* ensure shared  */
   ASSERT(false == dev->locked);                /* not locked yet */
   ASSERT(NULL != dev->units[0].fd);            /* ensure fd      */

   if (-1 == flock(fileno(dev->units[0].fd), LOCK_EX)) {/* failed to lock? */
      HALT(H_IOErr);                            /*    I/O error   */
      return 1;                                 /*    HALT        */
   }
   dev->locked = true;                          /* mark locked    */
   return 0;
}

void unlockDEV(Device *dev)
{
   /* NB. always lock unit0, because the controller is locked!    */

   ASSERT(dev->shared);                         /* ensure shared  */
   ASSERT(dev->locked);                         /* ensure actually locked */
   ASSERT(NULL != dev->units[0].fd);            /* ensure fd      */

   dev->locked   = false;                       /* clear locked status  */
   dev->unlockTO = 0;                           /* clear unlock timeout */
   if (-1 == flock(fileno(dev->units[0].fd), LOCK_UN))/* failed to unlock? */
      HALT(H_IOErr);                            /*    HALT     */
}

int initDSK(Device *dev, CtrlFn *pCtrlfn)        /* Init DSK operation */
{
   Tyme dt;
   Word sector, track;

   dev->status = 0;                             /* clear status */

   sector = 017777 & dev->DSK_DTS;              /* calc sector    */
   if (sector >= dev->units[0].nsecs)           /* sector off?    */
      dev->status |= DSK_E_NODISC;              /* report NO DISC */

   track = 0177 & (sector >> 3);                /* calc track  */
   if (dev->u.DSK.protect & (1 << (track >> 4)))/* track protected?  */
      dev->status |= DSK_E_WRITE;               /* report WRITE ERROR */

   if (dev->status) {                           /* any error?     */
      dev->status |= DSK_E_ERROR;               /* set ERROR flag */
      dev->delay = 3 * memTime / 10;            /* report ERROR   */
      *pCtrlfn = IOC_NONE;                      /* no actual I/O  */
      goto Exit;                                /* done           */
   }

   if (dev->shared) {                           /* shared disk? */
      lockDEV(dev);                             /*    lock it   */
      if (Halt) return 1;                       /* I/O failed?, HALT */
   }

   dev->delay = 2084;                           /* actual xfer: 256 * 8us */

   if (dev->shared) {                           /* shared drive?  */
      dev->u.DSK.lastIO = 0;                    /* clear last I/O time */
                                                /* calc w/ half rev.   */
   }

   dt = usince(dev->u.DSK.lastIO);              /* elapsed time since last I/O*/
   if (dt > 2084) {                             /* more than 2084us?    */
      dev->delay += 8334;                       /* avg rotational delay */
   } else if (1 == dev->DSK_DTS 
                   - dev->u.DSK.lastDTS) {      /* next sector? */
      dev->delay += 2084 - dt;                  /* wait until we got there */
      if (3 == (dev->u.DSK.lastDTS & 7))        /* sector 3 -> 4*/
         dev->delay += 2084;
   }
Exit:
   dev->u.DSK.lastIO = elapsedTime;             /* save current time  */
   return 0;
}

void doioDSK(Device *dev, Event *evt)           /* Perform DSK I/O   */
{
   Word buf[SEC_SIZE];
   int  i;
   Word sector;

   if (IOC_PULSE == evt->ctrlfn) {              /* was a WRITE?   */
      for (i = 0; i < SEC_SIZE; i++)            /* read words     */
         buf[i] = memReadIO(dev->DSK_ACNT + i); /* from memory*/
   }
   sector = (017777 & dev->DSK_DTS);            /* calc. sector */
   switch (evt->ctrlfn) {
   case IOC_NONE:                               /* delayed error */
      break;
   case IOC_START:                              /* do READ */
      for (i = 0; i < SEC_SIZE; i++)
         buf[i] = dev->mem[sector + i];
      break;
   case IOC_CLEAR:
      ASSERT(true);                             /* cannot be Clear */
      break;
   case IOC_PULSE:                              /* do WRITE */
      for (i = 0; i < SEC_SIZE; i++)
         dev->mem[sector + i] = buf[i];
      break;
   }
   if (IOC_START == evt->ctrlfn) {              /* was a READ? */
      for (i = 0; i < SEC_SIZE; i++)            /* write words */
         memWriteIO(dev->DSK_ACNT + i, buf[i]); /* to memory*/
   }
   dev->DSK_ACNT += SEC_SIZE;                   /* update address cnt */
   if (IOC_PULSE == evt->ctrlfn)                /* was a WRITE?       */
      dev->DSK_ACNT += 2;                       /*    ahead 2 words   */
   dev->u.DSK.lastDTS = dev->DSK_DTS;           /* save last sector   */
}

int initDKP(Device *dev, int uno, CtrlFn *pCtrlfn)/* Init DKP operation */
{
   int  mode;
   Word cylinder, sector, surface;
   int  delay;
   Tyme dt;
   int  sic, dsic;
   CtrlFn ctrlfn;
   Word csr, dar;

   ctrlfn = *pCtrlfn;

   dev->status &= 0177700;                      /* clear error flags  */
   csr  = dev->DKP_CSR;                         /* Cmd/Cylinder Select Reg. */
   dar  = dev->DKP_DAR;                         /* Disc Address Reg.        */
   mode = DKP_MOD(dev->DKP_CSR);                /* get mode */

   TRACE(printf("INIDKP %06o %02o:%d %s cmd=%d\n",
                  PC,dev->devno,uno,ctrlfns[ctrlfn],mode));

   cylinder = DKP_CYL(csr);                     /* get target cylinder */
   if (cylinder > (DKP_GEO_CC-1))               /* no such cylinder?   */
      dev->status |= DKP_E_SEEK;                /*    SEEK error       */

   sector = DKP_SEC(dar);                       /* get target sector   */
   if (sector > (DKP_GEO_SS-1))                 /* sector off?         */
      dev->status |= DKP_E_UNSAFE;              /*    UNSAFE error     */

   surface = DKP_HED(dar);                      /* get target surface  */
   if (surface > (DKP_GEO_HH-1))                /* surface off?        */
      dev->status |= DKP_E_UNSAFE;              /*    UNSAFE error     */

   if (dev->units[uno].fd == NULL)              /* no such unit?       */
      goto ErrOut;

   if (dev->shared) {                           /* shared disc?      */
      if (!dev->locked) {                       /* not locked yet?   */
         lockDEV(dev);                          /*    lock it        */
         if (Halt) return 1;                    /* I/O failed?, HALT */
         dev->unlockTO = elapsedTime + 60000000;/* unlock in 6sec    */
      }
   }

   delay = 0;

   switch (mode) {
   case DKP_MREAD:
   case DKP_MWRITE:
      if (IOC_PULSE == ctrlfn)                  /* Pulse? no R/W     */
         goto ErrOut;
      if (sector > (DKP_GEO_SS-1)) {            /* no such sector?   */
         TRACE(printf("%06o %02o FOREVER\n",PC,dev->devno));
         dev->delay = FOREVER;                  /*    it takes forever */
      }
      else {
         dev->delay = 3334;                     /* actual xfer: 2840us */
         if (dev->units[uno].lastRW) {          /* no seek since last R/W? */
            dt = usince(dev->units[uno].lastRW);/* time lapsed         */
            sic = (dev->units[uno].lastSEC + dt / 3334) % DKP_GEO_SS;
            if (sector > sic)                   /* calc. delta sectors */
               dsic = sector - sic;
            else
               dsic = DKP_GEO_SS - sic + sector;
            if (dsic < 1)                       /* 1 sector difference?*/
               dev->delay += 40000;             /* full rev. */
            else
               dev->delay += dsic * 3334;
         }
         else
            dev->delay += 20000;                /* half rev. */
      }
      break;
   case  DKP_MSEEK:
      delay = 20000 * abs(DKP_CYL(dev->units[uno].DKP_CSR) - cylinder);
      goto LDelay;
   case DKP_MRECAL:
      delay = 20000 * DKP_CYL(dev->units[uno].DKP_CSR);/* 20ms * cylinder */
LDelay:
      dev->status |= DKP_SEEKING(uno);          /* set seeking flag   */
      if (delay > 135000)                       /* max. 135ms         */
         delay = 135000;
      else if (delay < 20000)
         delay = 20000;
      dev->delay = delay;
      break;
   }
   return 0;

ErrOut:
   if (dev->status & DKP_E_ERROR) {             /* error set? */
      if (IOC_PULSE == ctrlfn)                  /*    Pulse?         */
         dev->status |= DKP_DONE_SEEK(uno);     /*    SEEKu DONE     */
      else
         dev->status |= DKP_DONE_RW;            /*    R/W DONE       */
      dev->delay = 2 * memTime / 10;            /*    minimal delay  */
      *pCtrlfn = IOC_NONE;                      /*    no actual I/O  */
   }

   return 0;
}

void doioDKP(Device *dev, Event *evt)           /* Perform DKP I/O */
{
   Byte uno, scnt;
   off_t offs;
   Word buf[SEC_SIZE];
   Word mac, csr, dar;
   int  i, n, mode, sic;

   ASSERT(dev->devno == evt->devno);            /* ensure same devno */ 

   uno  = evt->uno;                             /* get unit number   */
   csr  = dev->units[uno].DKP_CSR;              /* get Mode/Cylinder */
   dar  = dev->units[uno].DKP_DAR;              /* get Drive/Surf/Sec/SCnt */

   ASSERT(uno == DKP_DRV(dar));                 /* ensure same unit  */

   mode = DKP_MOD(csr);                         /* get unit mode     */
   mac  = 077777 & dev->DKP_MAC;                /* get address counter */
   scnt = DKP_SCNT(dar);                        /* get sector counter  */

   TRACE(printf("IODONE %06o %02o:%d %s/%d mac=%06o scnt=%02o status=%06o (%d,%d,%d)\n",
                  PC,dev->devno,uno,ctrlfns[evt->ctrlfn],
                  mode,mac,scnt,dev->status,
                  DKP_CYL(csr),DKP_HED(dar),DKP_SEC(dar)));

   if (IOC_NONE == evt->ctrlfn)                 /* just delayed error */
      goto Exit;

   switch (mode) {
   case DKP_MREAD:
   case DKP_MWRITE:
      offs =                     DKP_CYL(csr);  /* calc. offset        */
      offs = DKP_GEO_HH * offs + DKP_HED(dar);
      offs = DKP_GEO_SS * offs + DKP_SEC(dar);
      if (-1 == fseeko(dev->units[uno].fd, offs, SEEK_CUR)) {
         HALT(H_IOErr);
         return;
      }
      sic  = DKP_GEO_SS * DKP_HED(dar);         /* sector in cylinder  */
      sic += DKP_SEC(dar);
      do {
         if (DKP_MREAD == mode) {               /* handle READ         */
            n = fread(&buf[0], sizeof(Word), SEC_SIZE, dev->units[uno].fd);
            if (n != SEC_SIZE) {                /* I/O failed?         */
               HALT(H_IOErr);                   /*    HALT             */
               return;
            }
            for (i = 0; i < SEC_SIZE; i++)      /* write buffer to     */
               memWriteIO(mac++, SWAP(buf[i])); /*    memory           */
         }
         else if (DKP_MWRITE == mode) {         /* handle WRITE        */
            for (i = 0; i < SEC_SIZE; i++)      /* read buffer from    */
               buf[i] = SWAP(memReadIO(mac++)); /*    memory           */
            n = fwrite(&buf[0], sizeof(Word), SEC_SIZE, dev->units[uno].fd);
            if (n != SEC_SIZE) {                /* I/O failed?         */
               HALT(H_IOErr);                   /*    HALT             */
               return;
            }
         }
         scnt = 017 & (scnt + 1);               /* incr. sector count   */
         if (scnt) {
            sic++;                              /* incr. sector in cylinder*/
            if (DKP_GEO_HHSS == sic) {          /* end of cylinder? */
               dev->status |= DKP_E_END;        /*    report END error  */
               sic = 0;
               break;
            }
         }
      } while (0 != scnt);

      dev->units[uno].lastRW  = elapsedTime;    /* save lastRW and */
      dev->units[uno].lastSEC = sic % DKP_GEO_SS;  /*    position  */

                                                /* update DAR      */
      dar &= 0177000;                           /* clear Surf/Sec/SCnt*/
      if (sic == DKP_GEO_HHSS)                  /* update Surf/Sec */
         sic = 0;
      if (sic > (DKP_GEO_SS-1)) {
         dar |= 0400; sic -= DKP_GEO_SS;
      }
      dar |= (017 & sic) << 4;
      dar |= scnt;                              /* update SCnt     */
      dev->units[uno].DKP_DAR = dar;
      if (DKP_MWRITE == mode)                   /* adjust adr counter     */
         mac += 2;                              /*    if WRITE            */
      dev->DKP_MAC = mac;                       /* update adr counter     */
                                                /* do NOT clear bit0      */
      dev->status |= DKP_DONE_RW;               /* R/W done               */
      if (scnt)                                 /* non-zero sector count? */
         dev->status |= DKP_E_END;              /*    report END error    */
      break;
   case DKP_MSEEK:
      offs  = SEC_BYTES;
      offs *= DKP_GEO_HHSS * DKP_CYL(dev->units[uno].DKP_CSR);
      goto LSeek;
   case DKP_MRECAL:
      offs = 0;
      dev->units[uno].DKP_CSR = 0;
LSeek:
      ASSERT(0 != (dev->status & DKP_SEEKING(uno)));
      dev->units[uno].lastRW = 0;               /* clear last R/W     */
      if (-1 == fseeko(dev->units[uno].fd, offs, SEEK_SET)) /* I/O error? */
         HALT(H_IOErr);                         /*    HALT             */
      dev->status &= ~DKP_SEEKING(uno);         /* clear Seeking flag  */
      dev->status |= DKP_DONE_SEEK(uno);        /* set Seeking Done    */
      break;
   }
Exit:
   TRACE(printf("IODONE %06o %02o:%d         mac=%06o scnt=%02o status=%06o (%d,%d,%d)\n",
                  PC,dev->devno,uno,mac,scnt,dev->status,
                  DKP_CYL(csr),DKP_HED(dar),DKP_SEC(dar)));
}

void devStart(Byte devno)                       /* Start device `dev' */
{
   int uno;
   Device *dev;
   CtrlFn ctrlfn;

   ctrlfn = IOC_START;                          /* default is Start */

   dev = &devs[devno];                          /* point to device*/
   if (NO_LINE == dev->line)                    /* no such device */
      return;                                   /*    do nothing  */

   uno = 0;
   if (DEV_DKP == devno)                        /* calc. drive for DKP */
      uno = DKP_DRV(dev->DKP_DAR);

   TRACE(printf("IOSTAR %06o %02o:%d Busy=%d,Done=%d\n",
               PC,dev->devno,uno,dev->busy,dev->done));

   if (dev->busy)                               /* device busy?   */
      return;                                   /*    do nothing. */

   switch (devno) {
   case DEV_PTR:
   case DEV_PTP:
   case DEV_LPT:
   case DEV_DSK:
      if (!dev->units[uno].fd) {                /* no file assigned? */
         if (!MissingDev)                       /* no missing devs allowed? */
            HALT(H_MisDev);                     /*    HALT           */
         return;
      }
      break;
   default:
      break;
   }

   if (DEV_CPU == devno)                        /* this is the CPU?  */
      delayION = true;                          /* execute delayed INTEN */
   else {
      switch (devno) {
      case DEV_RTC:                             /* RTC? */ 
         {
            int freq;
            switch (dev->buf[BUF_A] & 3) {      /* set-up frequency */
            case 0: freq =   HZ; break;
            case 1: freq =   10; break;
            case 2: freq =  100; break;
            case 3: freq = 1000; break;
            }
            dev->delay = 1000000 / freq;        /* delay according to freq */
         }
         break;
      case DEV_LPT:                             /* LPT? */
         {
            Flag spacing;
            Byte pos, ch;
            PrinterOp  op;

            op = LPT_XMIT; spacing = false;
            pos = dev->u.LPT.pos++;             /* xmit char buffer to zone */
            dev->u.LPT.buf[pos] = (ch = 0377 & dev->LPT_CHBUF);
            switch (ch) {
            case ASC_CR:
               op = LPT_CTRL;
               break;
            case ASC_LF:                        /* spacing?      */
               op = LPT_CTRL;
               dev->u.LPT.lines++;
               if (0 == pos) {                  /* 1st position? */
                  dev->delay = 13000;           /* 13ms spacing  */
                  spacing = true;
               }
               else
                  dev->delay = 20000;           /* 20ms spacing  */
               break;
            case ASC_FF:                        /* form feed ?   */
               op = LPT_CTRL;                   /* 63 lines/page */
               dev->delay = 13000 * (63 - (dev->u.LPT.lines % 63));
               dev->u.LPT.lines = 0;            /* reset line counter */
               break;
            default:
               dev->delay = 6;                  /* 6us char buf transfer */
               break;
            }
            if (LPT_XMIT == op) {               /* just a xmit?   */
               Byte lastPos;

               lastPos = LPT_NBUF;
               if (LPT_NZONE == dev->u.LPT.zone)/* last zone?      */
                  lastPos >>= 1;                /*    full at half      */
               lastPos--;
               if (pos == lastPos)              /* was last position?   */
                  op = LPT_FULL;                /* change to full       */
            }
            if (LPT_XMIT != op && !spacing)     /* print and no spacing?*/
               dev->delay += 34000;             /*    34ms print time   */
            dev->u.LPT.op = op;
         }
         break;
      case DEV_DSK:                             /* DSK? */
         if (usince(dev->u.DSK.lastDOA) < 1000)
            fprintf(stderr,                     /* check 1ms since last DOA */
                    "%06o/%06o |DSK Start - DOA| < 1ms!\n",
                    PC,M[PC]);
         if (initDSK(dev, &ctrlfn))             /* init I/O operation */
            return;                             /* failed, return     */
         break;
      case DEV_DKP:
         uno = DKP_DRV(dev->DKP_DAR);
         if (initDKP(dev, uno, &ctrlfn))        /* init I/O operation */
            return;                             /* failed, return     */
         dev->units[uno].DKP_CSR = dev->DKP_CSR;/* save regs, to units */
         dev->units[uno].DKP_DAR = dev->DKP_DAR;
         TRACE(printf("REGDKP %06o %02o:%d CSR=%06o DAR=%06o\n",
                        PC,dev->devno,uno,dev->DKP_CSR,dev->DKP_DAR));
         break;
      }

      dev->busy = true;                         /* Busy=1,Done=0 */
      dev->done = false;

      TRACE(printf("IOSTAR %06o %02o:%d %s delay=%d\n",
                     PC,dev->devno,uno,ctrlfns[ctrlfn],dev->delay));
      enqueIO(dev, uno, ctrlfn);                /* enqueue I/O */
   }
}

void devClear(Byte devno)                       /* Clear device `dev' */ 
{
   Device *dev;
   Event *p, *q;
   Flag done;

   dev = &devs[devno];
   if (DEV_DSK == devno || DEV_DKP == devno) {  /* DSK or DKP? */
      Flag working = dev->busy ||               /* either busy */
                     (DEV_DKP == devno && 0 != (003600 & dev->status));
                                                /*    OR DKP Seeking    */
      if (working) {                            /* I/O was in progress? */
         if (dev->locked)                       /*    device locked?    */
            unlockDEV(dev);                     /*    unlock it         */
      }
      dev->status = 0;
   }

   if (DEV_DSK != devno && DEV_DKP != devno) {
      if (DEV_CPU != devno)                     /* clear buffers */
         dev->buf[BUF_A] = 0;                   /* except CPU buffer A (SW) */
      if (DEV_MDV != devno)
         dev->buf[BUF_B] = 0;                   /* MDV just clear buffer A */
      if (DEV_MDV != devno)
         dev->buf[BUF_C] = 0;
   }

   done = false;
   while (!done) {                              /* remove all events for devno*/
      q = removeQUE(devno);                     /* get prev entry       */
      if (NULL == q) {                          /* first OR not found?  */
         if (QUE && devno == QUE->devno) {      /* first entry?         */
            q = QUE;                            /* remove it            */
            QUE = q->que;
         }
         else
            done = true;                        /* empty QUE OR not found  */
      }
      else {
         p = q->que;                            /* point to devno event */
         ASSERT(devno == p->devno);             /* ensure devno event   */
         q->que = p->que;                       /* remove it            */
         q = p;
      }

      if (q)
         clearEVT(q);
   }

   dev->busy = false;                           /* clear control FFs */
   dev->done = false;

   TRACE(printf("IOCLER %06o %02o %s\n",
                  PC,dev->devno,ctrlfns[IOC_CLEAR]));

   if (dev->intreq) {                           /* INT request pending?  */
      dev->intreq = false; IREQ--;              /* clear dev and CPU flag*/
   }
}

void devPulse(Byte devno)                       /* Pulse device `dev' */
{
   Byte uno;
   Device *dev;
   CtrlFn ctrlfn;

   dev = &devs[devno];                          /* point to device   */
   ctrlfn = IOC_PULSE;                          /* default is Pulse  */

   uno = 0;
   if (DEV_DKP == devno)                        /* calc. drive for DKP */
      uno = DKP_DRV(dev->DKP_DAR);

   switch (devno) {
   case DEV_DSK:
      if (!dev->units[uno].fd) {                /* no file assigned? */
         if (!MissingDev)                       /* no missing devs allowed? */
            HALT(H_MisDev);                     /*    HALT           */
         return;
      }
      if (dev->units[uno].rdonly) {             /* drive read-only?  */
         HALT(H_ReadOnly);                      /*    HALT           */
         return;
      }
      break;
   default:
      break;
   }

   switch (devno) {
   case DEV_MDV:                                /* MDV MUL   */
      break;
   case DEV_DSK:                                /* DSK Write */
      if (initDSK(dev, &ctrlfn))                /* init I/O operation */
         return;                                /* failed, return     */
      if (usince(dev->u.DSK.lastDOA) < 1000)    /* check 1ms since last DOA */
         fprintf(stderr,"%06o/%06o |DSK Pulse - DOA| < 1ms!\n",
                  PC,M[PC]);
      break;
   case DEV_DKP:
      uno = DKP_DRV(dev->DKP_DAR);              /* get unit number    */
      if (dev->status & DKP_SEEKING(uno))       /* already Seek or Recal? */
         return;
      if (initDKP(dev, uno, &ctrlfn))           /* init I/O operation */
         return;                                /* failed, return     */
      dev->units[uno].DKP_CSR = dev->DKP_CSR;   /* save regs, to units */
      dev->units[uno].DKP_DAR = dev->DKP_DAR;
      TRACE(printf("REGDKP %06o %02o:%d CSR=%06o DAR=%06o\n",
                     PC,dev->devno,uno,dev->DKP_CSR,dev->DKP_DAR));
      break;
   default:                                     /* default case:       */
      return;                                   /*    do NOT start I/O */
   }

   if (DEV_DSK == devno)
      dev->busy = true;                        /* Busy=1,Done=0 */
   dev->done = false;

   TRACE(printf("IOPULS %06o %02o:%d %s status=%06o delay=%d\n",
                  PC,dev->devno,uno,ctrlfns[ctrlfn],dev->status,dev->delay));

   enqueIO(dev, uno, ctrlfn);

   return;
}

int readChar(Device *dev)
{
   int ch;

   ASSERT(NULL != dev->units[0].fd);

   ch = fgetc(dev->units[0].fd);
   if (EOF == ch) {
      ch = 0;
      if (!feof(dev->units[0].fd))
         HALT(H_IOErr);
   }
   return ch;
}

void writeChar(Device *dev, int ch)
{
   char buf[8];
   int n;

   ASSERT(NULL != dev->units[0].fd);

   buf[0] = ch;
   n = fwrite(buf, sizeof(char), 1, dev->units[0].fd);
   if (1 != n)
      HALT(H_IOErr);
}

void devDone(Event *evt)                       /* Device done, ie timed out */
{
   int ch;
   Device *dev;
   Flag done;

   dev = &devs[evt->devno];                     /* get device data   */

   ASSERT(NULL == evt->que);                    /* removed from event queue */
   ASSERT(NO_LINE != dev->line);                /* ensure device present */

   switch (dev->devno) {
   case DEV_MDV:                                /* MDV has no FFs */
      break;
   case DEV_LPT:
      /* xmit char - does NOT set done, otherwise set done */
      dev->done = LPT_XMIT == dev->u.LPT.op? false : true;
      dev->busy = false;
      break;
   case DEV_DKP:
      if (IOC_PULSE != evt->ctrlfn) {           /* Seek/Recalibrate does */
         dev->done = true;                      /*    NOT set Done       */
         dev->busy = false;
      }
      break;
   default:
      dev->done = true;                         /* Done=1,Busy=0  */
      dev->busy = false;
   }

   switch (dev->devno) {
   case DEV_MDV:
      switch (evt->ctrlfn) {
      case IOC_NONE:
         break;
      case IOC_START:                           /* MDV DIV */
         {
            DWord dw;

            dw = dev->buf[BUF_A];
            dw = (dw << 16) + dev->buf[BUF_B];
            dev->buf[BUF_A] = dw % dev->buf[BUF_C];
            dev->buf[BUF_B] = dw / dev->buf[BUF_C];
         }
         break;
      case IOC_CLEAR:
         break;
      case IOC_PULSE:                           /* MDV MUL */
         {
            DWord result;

            result = dev->buf[BUF_A] +
                     (DWord)dev->buf[BUF_B] * dev->buf[BUF_C];
            dev->buf[BUF_A] = 0177777 & (result >> 16);
            dev->buf[BUF_B] = 0177777 & result;
         }
         break;
      }
      break;
   case DEV_TTO:                                /* TTO? */
      ch = 0377 & dev->buf[BUF_A];
      if (dev->units[0].fd)
         writeChar(dev, ch);
      printf("%c",toupper(ch));                 /* write upper case char */
      break;
   case DEV_TTI:                                /* TTI? */
      if (dev->units[0].fd) {
         ch = readChar(dev);                    /* 8bit char */
         dev->buf[BUF_A] = ch;
      }
      else {
         if (has_key()) {                       /* check TTY */
            ch = getkey(0);
            dev->buf[BUF_A] = toupper(ch);      /* read upper case char */
         }
      }
      break;
   case DEV_LPT:                                /* LPT? */
      {  PrinterOp op;
         Flag resetZone;
         int n;

         ASSERT(NULL != dev->units[0].fd);      /* ensure file opened */

         resetZone = false;
         op = dev->u.LPT.op;                    /* get printer op */
         if (LPT_XMIT != op) {                  /* not xmit char buffer? */
            n = fwrite(&dev->u.LPT.buf[0],      /* write zone buffer out */
                   sizeof(Byte),
                   dev->u.LPT.pos,
                   dev->units[0].fd);
            if (n != dev->u.LPT.pos)            /* check chars written */
               HALT(H_IOErr);                   /*    error if less    */
            dev->u.LPT.pos = 0;                 /* reset position */
            if (LPT_NZONE == dev->u.LPT.zone || LPT_CTRL == op)
                                                /* last zone OR CtrlOp? */
               resetZone = true;                /*    do reset    */
            else
               dev->u.LPT.zone++;               /*    next zone   */
            if (resetZone)
               dev->u.LPT.zone = 1;             /* reset zone     */
         }
      }
      break;
   case DEV_DSK:                                /* DSK? */
      doioDSK(dev, evt);                        /*    perform I/O */
      if (dev->locked)                          /*    unlock it   */
         unlockDEV(dev);
      break;
   case DEV_DKP:                                /* DKP? */
      doioDKP(dev, evt);                        /*    perform I/O */
      if (dev->locked) {
         ASSERT(IOC_NONE != evt->ctrlfn);       /* ensure unlocked if no I/O */
         if (IOC_START == evt->ctrlfn)          /* was a Read/Write? */
            unlockDEV(dev);                     /*    unlock it      */
      }
      break;
   }

   done = dev->done;
   if (DEV_DKP == dev->devno                    /* DKP? */
       && IOC_PULSE == evt->ctrlfn)             /*    Seek/Recalibrate?     */
   {
      if (0 != (074000 & dev->status))          /* any Seek Done flag set?  */
         done = true;                           /*    trigger INT           */
   }

   if (done                                     /* Done set?                */
       && false == dev->intdis                  /*    AND IntDisable clear? */
       && false == dev->intreq)                 /*    AND IntReq clear?     */
   {
      dev->intreq = true; IREQ++;               /* trigger an INT request */
      TRACE(printf("IOIREQ %06o %02o\n",PC,dev->devno));
   }
}

/* process I/O event queue
 * - take into account elapsed time
 * - all devices whose timeout lapsed, fired
*/
void doIO(int dt)                               /* Do Input/Output   */
{
   Event *q;

   q = QUE;                                     /* elapsed dt time   */
   while (q) {
      q->timeout -= dt;
      q = q->que;
   }

   while (QUE && (QUE->timeout <= 0)) {         /* at the front time lapsed */
      q = QUE; QUE = q->que;                    /* remove device from queue */
      q->que = NULL;                            /* ensure ptr cleared       */
      devDone(q);                               /* do actual I/O            */
      clearEVT(q);                              /* clear event record       */
   }

   ioTime = elapsedTime;                        /* reset ioTime            */
   if (QUE)                                     /* if QUE not empty        */
      eventTime = ioTime + QUE->timeout;        /*    reset eventTime also */
}

void finish(void)
{
   int i;
   Byte uno;
   Device *dev;

   dev = &devs[DEV_DSK];                        /* point to DSK   */
   if (NULL != dev->units[0].fd) {              /* DSK opened ?   */
      long nbytes;
      if (-1 == fseek(dev->units[0].fd, 0, SEEK_SET)) {/* go to beginning */
         fprintf(stderr,"fseek(): %s!\n",devs->units[0].path);
         goto LCloseAll;
      }
      for (i = 0; i < dev->units[0].nsecs * SEC_SIZE; i++)
         dev->mem[i] = SWAP(dev->mem[i]);
      nbytes = fwrite(dev->mem,                 /* write out data */
                     SEC_BYTES,
                     dev->units[0].nsecs,
                     dev->units[0].fd);
      if (nbytes != dev->units[0].nsecs) {      /* check bytes written */
         fprintf(stderr,"fwrite(): %s!\n",dev->units[0].path);
         goto LCloseAll;
      }
   }
LCloseAll:
   for (i = 1; i < NDEV; i++) {                 /* close all open storage */
      dev = &devs[i];
      for (uno = 0; uno < 4; uno++)             /* for each drive */
         if (NULL != dev->units[uno].fd) {
            if (EOF == fclose(dev->units[uno].fd))/* close, check status */
               fprintf(stderr,"fclose(): failed to close %s!\n",dev->units[uno].path);
         }
   }
}

int keyIndex(char *key, char *pfx)
{
   int idx;
   char *tmp;
   
   idx = 0;                                     /* assume 0    */
   tmp = key + strlen(pfx);                     /* skip prefix */
   if (*tmp) {                                  /* any chars after prefix? */
      if (!isdigit(*tmp))                       /* not digit? */
         return -1;                             /*    error   */
      idx = atoi(tmp);                          /* convert to int */
   }
   return idx;
}

void initDevs(int nopts, Option *opts)
{
   int i;
   Byte devno, uno;
   Device *q, *dev;
   Option *opt;

   static struct {
      Byte devno;
      Byte line;
      int  delay;
   } inits[] = {
      {DEV_MDV, 16,     7},  /* 6.8us MUL, 7.2us DIV */
      {DEV_TTI, 14, 100000},  /*  10cps */
      {DEV_TTO, 15, 100000},  /*  10cps */
      {DEV_PTR, 11,   2500},  /* 400cps */
      {DEV_PTP, 11,  15873},  /*  63cps */
      {DEV_RTC, 13,  20000},  /*  50Hz  */
      {DEV_LPT, 12,  20000},  /* 6us char buf xmit, 20ms/zone print */
      {DEV_DSK,  9,   2084},  /* 2084us/sector */
      {DEV_DKP,  7,   3334},  /* 3334us/sector */
      {DEV_CPU, 16,      0},  /* dummy line */
      {      0,  0,      0}
   };

   for (i = 0; i < NEVENT; i++)                 /* clear event storage */
      clearEVT(&events[i]);

   for (i = 0; i < NDEV; i++) {
      dev = &devs[i];
      dev->devno   = i;
      dev->intdis = false;
      dev->intreq = false;
      dev->line   = NO_LINE;
      dev->delay  = 0;
      dev->bus    = NULL;
      for (uno = 0; uno < 4; uno++) {
         dev->units[uno].path  = NULL;
         dev->units[uno].fd    = NULL;
         dev->units[uno].nsecs = 0;
         dev->units[uno].rdonly=false;
         dev->units[uno].buf[BUF_A] = 0;
         dev->units[uno].buf[BUF_B] = 0;
         dev->units[uno].buf[BUF_C] = 0;
         dev->units[uno].lastRW  = 0;
      }
      dev->mem    = NULL;
      dev->shared = false;
      dev->locked = false;
      dev->unlockTO = 0;
   }

   IREQ  = 0;  /* no. of INT requests */

   BUS = NULL;
   for (i = 0; inits[i].devno; i++) {
      devno = inits[i].devno;
      dev = &devs[devno];
      dev->line  = inits[i].line;
      dev->delay = inits[i].delay;
      devClear(devno);

      /* don't insert CPU in I/O bus */
      if (DEV_CPU == devno)
         continue;

      q = insertBUS(dev->delay);
      if (NULL == q) {
         dev->bus = BUS;
         BUS = dev;
      }
      else {
         dev->bus = q->bus;
         q->bus = dev;
      }
   }

   for (i = 0; i < nopts; i++) {
      char *mode = NULL;
      char *errmsg = NULL;

      opt   = &opts[i];
      devno = opts[i].devno;
      dev   = &devs[devno];
      if (0 == strncmp(opt->key, "file", 4)) {  /* storage file   */
         uno = 0;
         switch (opt->devno) {
         case DEV_TTI:
            mode = "rb"; break;
         case DEV_TTO:
            mode = "wb"; break;
         case DEV_PTP:
            mode = "wb"; break;
         case DEV_PTR:
            mode = "rb"; break;
         case DEV_LPT:
            mode = "ab"; break;
         case DEV_DSK:
            mode = "r+b"; break;
         case DEV_DKP:
            uno = keyIndex(opt->key, "file");   /* get drive no.  */
            if (uno < 0 || uno > 3) goto ErrOut;/* max 4 drives   */
            mode = "r+b"; break;
         default: break;
         }
         if (!mode) goto ErrOut;
         dev->units[uno].fd = fopen(opt->value,mode);
         if (NULL == dev->units[uno].fd) {
            fprintf(stderr,"%s: cannot open %s!\n",
                     io_devs(opt->devno),
                     opt->value);
            exit(1);
         }
         dev->units[uno].path = opt->value;
         if (DEV_DSK == opt->devno || DEV_DKP == opt->devno) {
            off_t nsecs;
            if (-1 == fseek(dev->units[uno].fd, 0, SEEK_END)) {
               fprintf(stderr,"fseek(): SEEK_END (%s)\n",opt->value);
               exit(1);
            }
            nsecs = ftello(dev->units[uno].fd);
            if ((off_t)-1 == nsecs) {
               fprintf(stderr,"ftello(): %s!\n",opt->value);
               exit(1);
            }
            nsecs = nsecs / SEC_BYTES;
            errmsg = NULL;
            if (DEV_DSK == opt->devno) {
               if (nsecs < 256 || nsecs > 65536)
                  errmsg = "[64K - 2048K]!\n";
            }
            else if (DEV_DKP == opt->devno) {
               /* Diablo 31 style */
               if (DKP_NSEC != nsecs)
                  errmsg = "[1218K]!\n";
            }
            if (errmsg) {
               fprintf(stderr,"%s: %s unit%d has invalid size %s!\n",
                        opt->value,
                        io_devs(opt->devno),
                        uno,
                        errmsg);
            }
            dev->units[uno].nsecs = nsecs;
            if (-1 == fseek(dev->units[uno].fd, 0, SEEK_SET)) {
               fprintf(stderr,"fseek(): SEEK_SET (%s)!\n",opt->value);
               exit(1);
            }
         }
         if (DEV_DSK == devno) {
            long nsecs;
            dev->mem = (Word*)malloc(dev->units[uno].nsecs * SEC_BYTES);
            if (NULL == dev->mem) {
               fprintf(stderr,"malloc(): not enough memory (%s)!\n",opt->value);
               exit(1);
            }
            nsecs = fread(dev->mem, SEC_BYTES, dev->units[uno].nsecs, dev->units[uno].fd);
            if (nsecs != dev->units[uno].nsecs) {
               fprintf(stderr,"fdread(): %s!\n",opt->value);
               exit(1);
            }
            for (i = 0; i < nsecs * SEC_SIZE; i++)
               dev->mem[i] = SWAP(dev->mem[i]);
         }
         continue;
      }
      else if (0 == strcmp(opt->key, "protect")) {
         if (DEV_DSK == devno) {
            dev->u.DSK.protect = atoi(opt->value);
            continue;
         }
      }
      else if (0 == strncmp(opt->key, "rdonly", 6)) {
         if (DEV_DSK == devno || DEV_DKP == devno) {
            uno = keyIndex(opt->key, "rdonly");
            if (uno < 0) goto ErrOut;
            dev->units[uno].rdonly = atoi(opt->value)? true : false;
         }
      }
      else if (0 == strcmp(opt->key, "shared")) {
         if (DEV_DSK == devno || DEV_DKP == devno) {
            dev->shared = atoi(opt->value)? true : false;
            continue;
         }
      }
      goto ErrOut;
   }

   dev = &devs[DEV_LPT];
   dev->u.LPT.lines = 0;
   dev->u.LPT.op    = LPT_IDLE;

   dev = &devs[DEV_DSK];
   dev->u.DSK.lastIO  = 0;
   dev->u.DSK.lastDOA = 0;
   dev->u.DSK.lastDTS = 0;

   atexit(finish);

   return;

ErrOut:
   fprintf(stderr,"%s: unknown option %s!\n",io_devs(opt->devno),opt->key);
   exit(1);

   /* 
      Code  Device   Line
        10  TTI      14
        11  TTO      15     10 cps
        12  PTR      11    150 cps
        13  PTP      13     63 cps
        14  RTC      13     HZ on power on
        17  LPT      12    230 lpm (Potter HSP3502)
        33  DKP       7    moving head disk (4047 Diablo 31-style)
        73  DKP1      7    2nd moving head disk

        cartridges: Model 31 - 1247KW, Model 33 - 2494KW
        4046 controller:
             4047 adapter (2x Model31/1x Model33)
             4049 adapter (4x Model31/2x Model33)

        SuperNova: 66 DIAS instr. placed from memory 0, execute instr @40

        20  DSK       9
        60  DSK2
        DG disk 256KW, 256W sectors, 128 tracks w/ 8 sectors
            1 word transferred ever 8us
            1 control w/ 8 disks
            3600rpm - sector time 2083.3us, data xfer time 2048us
   */
}


/* --- M E M O R Y ---------------------------------------------------------- */

void cycle(void)
{
   struct timespec dork, unslept;

   
   if (memCycle) {
      dork.tv_sec  = 0;
      dork.tv_nsec = memTime * 100;
      while (nanosleep(&dork, &unslept)) {
         dork = unslept;
      }
   }
   elapsedTime += memTime;
}

void memWrite(Word adr, Word w)
{
   ASSERT(adr < 0100000);

   M[adr] = w;
   if (Trace) T[adr] = 0;
   cycle();
}

Word memReadIO(Word adr)
{
   adr &= 077777;
/*fprintf(stderr,"R:%06o/%06o\n",adr,M[adr]);*/
   /* I/O memory read when time alread lapsed ! */
   return M[adr];
}

void memWriteIO(Word adr, Word w)
{
   adr &= 077777;
/*fprintf(stderr,"W:%06o/%06o\n",adr,w);*/
   /* I/O memory write, when time already lapsed ! */
   M[adr] = w;
   if (Trace) T[adr] = 0;
}

Word memRead(Word adr)
{
   ASSERT(adr < 0100000);

   cycle();
   return M[adr];
}

Word memReadIndir(Word adr)
{
   ASSERT(adr < 0100000);

   if (020 <= adr && adr <= 027)
      M[adr] += 1;
   else if (030 <= adr && adr <= 037)
      M[adr] -= 1;
   return memRead(adr);
}

/* --- P R O C E S S O R ---------------------------------------------------- */

#define SW     (devs[DEV_CPU].buf[BUF_A])
#define ION    (devs[DEV_CPU].busy)
#define PWFAIL (devs[DEV_CPU].done)
#define MSKO   (devs[DEV_CPU].buf[BUF_B])

typedef struct _HE {
   Word PC, IR;
   Word AC[4];
   Flag CRY, _ION;
   Word _MSKO;
} HE;

HE *H;
int nH, posH;

void saveH(Word IR)
{
   HE *h;
   int pos;

   pos = posH++ % nH;
   h = &H[pos]; 
   h->PC = PC;
   h->IR = IR;
   h->AC[0] = AC[0];
   h->AC[1] = AC[1];
   h->AC[2] = AC[2];
   h->AC[3] = AC[3];
   h->CRY   = CRY;
   h->_ION  = ION;
   h->_MSKO = MSKO;
}

void showHE(HE *h)
{
   printf("%06o/%06o  ACs: %06o %06o %06o %06o  %c%c  MSKO: %06o\n",
         h->PC, h->IR,
         h->AC[0],h->AC[1],h->AC[2],h->AC[3],
         h->CRY? 'C':' ',
         h->_ION? 'I':' ',
         h->_MSKO
   );
   disasm(h->PC,h->IR);
}

void showH(int from)
{
   int i, pos, ch;

Loop:
   pos = from;
   if (pos < 0) pos = nH + pos;
   pos = pos % nH;
   for (i = 0; i < 20; i++) {
      showHE(&H[(pos + i) % nH]);
   }

   printf("Back/Next? ");
   ch = toupper(getchar());
   if ('B' == ch) {
      from -= 20; goto Loop;
   }
   else if ('N' == ch) {
      from += 20; goto Loop;
   }
}

Word indirectChain(Word E, int indir)
{
   int cnt;

   cnt = 0;
   while (indir) {
      E = memReadIndir(E);
      if (0100000 == E) /* bit0 0 => 1 */
         indir = 0;
      if (0 == E)       /* bit0 1 => 0 */
         indir = 1;
      else              /* no change in sign */
         indir = 0100000 & E? 1 : 0;
      E &= 077777;
      if (++cnt > 16) {
         HALT(H_Indir);
         return 0;
      }
   }
   return E;
}

Word KIPS;                                      /* instr/ms           */
unsigned long syncInstr;                        /* sync instr. stream */
struct timeval tv0;                             /* last timestamp     */

unsigned long elapsedus(void)                   /* Elapsed us since tv0 */
{
   struct timeval tv;
   unsigned long dt;

   gettimeofday(&tv, NULL);
   dt = (tv.tv_sec - tv0.tv_sec) * 1000000      /* calc. delta us */
            + (tv.tv_usec - tv0.tv_usec);
   tv0 = tv;                                    /* update timestamp */
   return dt;                                   /* return delta us */
}

void syncKIPS(void)                             /* Synchronize KIPS */
{
   unsigned long t;

   t = elapsedus();                             /* get elapsed us */

   if (t < 1000)                                /* if less than 1000us (1ms) */
      usleep(1000 - t);                         /* wait the delta */

   syncInstr = numInstr + KIPS;                 /* update next instr. sync */
}

void waitIO(void)
{
   if (QUE)
      doIO(QUE->timeout);
}

void execute(Word IR)
{
   if (Trace)                                   /* trace execution?      */
      showsym(0);                               /*    show symbolic inst.*/

   if (KIPS && numInstr >= syncInstr)           /* if KIPS set,          */
      syncKIPS();                               /* sync instruction time */

   if (H)                                       /* has history buffer ? */
      saveH(IR);                                /*    save instr.       */

   if (QUE && elapsedTime >= eventTime)         /* do Input/Output */
      doIO(elapsedTime - ioTime);

   if (devs[DEV_DKP].unlockTO                   /* unlock timeout present? */
      && elapsedTime >= devs[DEV_DKP].unlockTO) /*    time passed?         */
   {
      ASSERT(devs[DEV_DKP].locked);             /* ensure locked  */
      unlockDEV(&devs[DEV_DKP]);
   }

   /* DCH has priority */

   if (IREQ && ION) {                           /* any IntReqs and IntOn ? */
      Word E;
      ION = false;                              /* clear IntOn */
      memWrite(0, PC);                          /* save PC @ 0 */
      E = indirectChain(1, 1);                  /* get handler address @ 1 */
      if (Halt)                                 /* check Halt */
         return;
      PC = E;                                   /* continue in handler */
   }

   if (delayION) {                              /* delayed IntOn ? */
      ION = true;                               /* set IntOn */
      delayION = false;
   }

   numInstr++;
   if (IR > 077777) {   /* arithmetic/logic */
      Word result;
      Flag ncy, cy;
      int cond;
      int acs, acd, fn, shift, carry, noload, skip;

      skip   = IR & 7; IR >>= 3;
      noload = IR & 1; IR >>= 1;
      carry  = IR & 3; IR >>= 2;
      shift  = IR & 3; IR >>= 2;
      fn     = IR & 7; IR >>= 3;
      acd    = IR & 3; IR >>= 2;
      acs    = IR & 3; IR >>= 2;

      switch (carry) {
      case 0: /* leave CRY */
         cy = CRY; break;
      case 1: /* Z force CRY to zero */
         cy = 0; break;
      case 2: /* O force CRY to one */
         cy = 1; break;
      case 3: /* C complement CRY */
         cy = 1 - CRY; break;
      }

      switch (fn) {
      case 0: /* COM */
         result = ~AC[acs];
         break;
      case 1: /* NEG */
         result = -AC[acs];
         if (0 == AC[acs]) cy = 1 - cy;
         break;
      case 2: /* MOV */
         result = AC[acs];
         break;
      case 3: /* INC */
         result = AC[acs] + 1;
         if (0177777 == AC[acs]) cy = 1 - cy;
         break;
      case 4: /* ADC */
         result = AC[acd] + ~AC[acs];
         if (AC[acd] > AC[acs]) cy = 1 - cy;
         break;
      case 5: /* SUB */
         result = AC[acd] - AC[acs];
         if (AC[acd] >= AC[acs]) cy = 1 - cy;
         break;
      case 6: /* ADD */
         result = AC[acd] + AC[acs];
         if (result < AC[acd]) cy = 1 - cy;
         break;
      case 7: /* AND */
         result = AC[acd] & AC[acs];
         break;
      }

      switch (shift) {
      case 0: /* no shift/swap */
         break;
      case 1: /* L rotate left */
         ncy = 0100000 & result? 1 : 0;
         result = (result << 1) + cy;
         cy = ncy;
         break;
      case 2: /* R rotate right */
         ncy = 1 & result? 1 : 0;
         result = (result >> 1);
         if (cy) result += 0100000;
         cy = ncy;
         break;
      case 3: /* S swap halves */
         result = ((result & 0377) << 8) + (result >> 8);
         break;
      }

      if (0 == noload) { /* "#" */
         AC[acd] = result; CRY = cy;
      }

      cond = 0;
      switch (skip >> 1) {
      case 0: /* Never "SKP" */
         cond = 0; break;
      case 1: /* "SZC" "SNC" */
         cond = 0 == cy; break;
      case 2: /* "SZR" "SNR" */
         cond = 0 == result; break;
      case 3: /* "SEZ" "SBN" */
         cond = 0 == cy || 0 == result; break;
      }
      if (1 & skip) cond = !cond;
      if (cond) PC = 077777 & (PC + 1);
   }
   else if (IR > 057777) { /* Input/Output */
      int ctrl, tfer, ac;
      Byte devno;

      devno = IR & 077; IR >>= 6;
      ctrl  = IR &   3; IR >>= 2;
      tfer  = IR &   7; IR >>= 3;
      ac    = IR &   3; IR >>= 2;

      if (tfer && tfer != 7) {
         if (tfer & 1)
            AC[ac] = devIN(devno,(tfer-1)/2);
         else
            devOUT(devno,(tfer-1)/2,AC[ac]);
      }

      if (7 != tfer) {
         switch (ctrl) {
         case 0: /* Do not activate */
            break;
         case 1: /* S start dev */
            devStart(devno);
            break;
         case 2: /* C clear dev */
            devClear(devno);
            break;
         case 3: /* P pulse dev */
            devPulse(devno);
            break;
         }
      }
      else {
         int cond;

         cond = 0;
         switch (ctrl) {
         case 0: /* SKPBN */
            cond = devs[devno].busy != false;
            break;
         case 1: /* SKPBZ */
            cond = devs[devno].busy == false;
            break;
         case 2: /* SKPDN */
            cond =  devs[devno].done != false;
            break;
         case 3: /* SKPDZ */
            cond = devs[devno].done == false;
            break;
         }
         if (cond) PC = 077777 & (PC + 1);
      }
   }
   else { /* Memory/Control */
      int disp, mode, indir, ac, fn;
      Word result, E;

      disp  = IR & 0377; IR >>= 8;
      mode  = IR &    3; IR >>= 2;
      indir = IR &    1; IR >>= 1;
      ac    = IR &    3; IR >>= 2;
      fn    = IR &    3;

      if (mode) {
         if (disp > 127) disp -= 256;
      }

      switch (mode) {
      case 0: /* page zero */
         E = disp;
         break;
      case 1: /* PC-relative */
         E = disp + PC;
         break;
      case 2: /* AC2 indexed */
         E = disp + AC[2];
         break;
      case 3: /* AC3 indexed */
         E = disp + AC[3];
         break;
      }

      E &= 077777;
      E  = indirectChain(E, indir);
      if (Halt)
         return;

      switch (fn) {
      case 0: /* program flow */
         switch (ac) {
         case 0: /* JMP */
            PC = E;
            return; /* don't increment PC */
         case 1: /* JSR */
            AC[3] = 077777 & (PC + 1);
            PC = E;
            return; /* don't increment PC */
         case 2: /* ISZ */
            result = memRead(E) + 1;
            if (0 == result)
               PC = 077777 & (PC + 1);
            memWrite(E, result);
            break;
         case 3: /* DSZ */
            result = memRead(E) - 1;
            if (0 == result)
               PC = 077777 & (PC + 1);
            memWrite(E, result);
            break;
         }
         break;
      case 1: /* LDA */
         AC[ac] = memRead(E);
         break;
      case 2: /* STA */
         memWrite(E, AC[ac]);
         break;
      }
   }
   PC = 077777 & (PC + 1);
}

void step(void)
{
   Word IR;

   IR = memRead(PC);
   execute(IR);

   if (nBKPT && BRAKE(PC)) {
      HALT(H_Break); return;
   }
}

#define CTRL_VC   (05)

void run(void)
{
   int ch;

   prepterm(1);                                 /* install terminal handler */
   while (!Halt) {
      if (has_key()) {                          /* key pressed?   */
         ch = getkey(0);                        /*    get it      */
         if (CTRL_VC == ch)                     /*    Ctrl-E?     */
            Halt = H_VCon;                      /*       request VC */
      }
      step();
   }
   prepterm(0);                                 /* remove terminal handler */
}

Word bootstrap[] = {    /* standard 32 word bootstrap loader */
062677,  /* BEG:     IORST          ;Reset all IO                             */
060477,  /*          READS 0        ;Read switches into AC0                   */
024026,  /*          LDA   1,C77    ;Get device mask (000077)                 */
0107400, /*          AND   0,1      ;Isolate device code                      */
0124000, /*          COM   1,1      ;-device code - 1                         */

010014,  /* LOOP:    ISZ   OP1      ;Count device code into all               */
010030,  /*          ISZ   OP2      ;IO instructions                          */
010032,  /*          ISZ   OP3                                                */
0125404, /*          INC   1,1,SZR  ;Done?                                    */
000005,  /*          JMP   LOOP     ;No, increment again                      */

030016,  /*          LDA   2,C377   ;Yes, put JMP 377 into location 377       */
050377,  /*          STA   2,377                                              */
060077,  /* OP1:     060077         ;Start device; (NIOS 0) - 1               */
0101102, /*          MOVL  0,0,SZC  ;Low speed device? (test switch 0)        */
000377,  /* C377:    JMP   377      ;No go to 377 and wait for channel        */

004030,  /* LOOP2:   JSR   GET+1    ;Get a frame                              */
0101065, /*          MOVC  0,0,SNR  ;Is it nonzero?                           */
000017,  /*          JMP   LOOP2    ;No, ignore and get another               */

004027,  /* LOOP4:   JSR   GET      ;Yes, get full word */
046026,  /*          STA   1,@C77   ;Store starting at 100 (autoincrement)    */
010100,  /*          ISZ   100      ;Count word - done?                       */
000022,  /*          JMP   LOOP4    ;No, get another                          */
000077,  /* C77:     JMP   77       ;Yes - location counter and jump to last  */
         /*                         ;word                                     */
0126420, /* GET:     SUBZ  1,1      ;Clear AC1, set Carry                     */
         /* OP2:                                                              */
063577,  /* LOOP3:   063577         ;Done?: (SKPDN 0) - 1                     */
000030,  /*          JMP   LOOP3    ;No, wait                                 */
060477,  /* OP3:     060477         ;Yes, read in AC0: (DIAS 0,0) - 1         */
0107363, /*          ADDCS 0,1,SNC  ;Add 2 frames swapped - got second?       */
000030,  /*          JMP   LOOP3    ;No, go back after it                     */
0125300, /*          MOVS  1,1      ;Yes, swap them                           */
001400,  /*          JMP   0,3      ;Return with full word                    */
000000,  /*          0              ;Padding                                  */
};

void load(void)
{
   Word w, DIAS;
   int i;

   w = 0;
   DIAS = (077 & SW) + 060500;
   for (i = 0; i < 32; i++)
      M[i] = bootstrap[i];

   PC = 0;
   AC[0] = (077 & SW) << 1;
   run();

#if 0

   /* SuperNova-style loader */
   if (0100000 & SW) {
      execute(DIAS);
      memWrite(0377, 0377);
      PC = 0377; AC[0] = (077 & SW) << 1;
      run();
      return;
   }

   execute(DIAS);
   while (0 == AC[0]) {
      waitIO();
      execute(DIAS);
   }

   PC = 0; w = 0;
   for (i = 0; i < 66; i++) {
      if (i) {
         waitIO();
         execute(DIAS);
      }
      w = (w << 8) + (077 & AC[0]);
      if (i & 1)
         memWrite(PC++, w);
   }
   PC = 040; AC[0] = (077 & SW) << 1;
   run();
#endif

}

/* --- V I R T U A L  C O N S O L E ----------------------------------------- */

char* alu_carrys[]  = {"", "Z", "O", "C"};
char* alu_fns[]     = {"COM", "NEG", "MOV", "INC", "ADC", "SUB", "ADD", "AND"};
char* alu_shifts[]  = {"", "L", "R", "S"};
char* alu_noloads[] = {"", "#"};
char* alu_skips[]   = {"", "SKP", "SZC", "SNC", "SZR", "SNR", "SEZ", "SBN"};

char* io_ctrls[]    = {"", "S", "C", "P"};
char* io_tfers[]    = {"NIO", "DIA", "DOA", "DIB", "DOB", "DIC", "DOC", "SKP"};
char* io_skips[]    = {"BN", "BZ", "DN", "DZ"};

char* mem_indirs[]  = {"", "@"};
char* mem_acs[]     = {"JMP", "JSR", "ISZ", "DSZ"};
char* mem_fns[]     = {"jmp", "LDA", "STA", "i/o"};

struct {
   Byte devno;
   char *name;
} devnames[] = {
   {DEV_MDV, "MDV"},
   {DEV_TTI, "TTI"},
   {DEV_TTO, "TTO"},
   {DEV_PTR, "PTR"},
   {DEV_PTP, "PTP"},
   {DEV_RTC, "RTC"},
   {DEV_LPT, "LPT"},
   {DEV_DSK, "DSK"},
   {DEV_DKP, "DKP"},
   {DEV_CPU, "CPU"},
   {      0, "NULL"}
};

Byte str2dev(char *nm)
{
   int i;

   for (i = 0; devnames[i].name; i++) {
      if (0 == strcmp(devnames[i].name, nm))
         return devnames[i].devno;
   }

   return 0;
}

char *io_devs(Byte devno)
{
   static char buf[8];
   int i;
   
   for (i = 0; NULL != devnames[i].name; i++)
      if (devno == devnames[i].devno)
         return devnames[i].name;
   sprintf(buf, "%02o", devno);
   return &buf[0];
}

char *special(Word IR)
{
   static struct {
      Word code;
      char *mnemo;
   } specials[] = {
      {060477, "READS"},
      {062677, "IORST"},
      {063077, "HALT"},
      {060177, "INTEN"},
      {060277, "INTDS"},
      {061477, "INTA"},
      {062077, "MSKO"},
      {073301, "MUL"},
      {073101, "DIV"},
      {0, NULL}
   };
   int i;

   for (i = 0; NULL != specials[i].mnemo; i++) {
      if (IR == specials[i].code)
         return specials[i].mnemo;
   }
   return NULL;
}

void disasm(Word pc,Word IR)
{
   char mnemo[8];

   if (IR > 077777) {   /* arithmetic/logic */
      int acs, acd, fn, shift, carry, noload, skip;

      skip   = IR & 7; IR >>= 3;
      noload = IR & 1; IR >>= 1;
      carry  = IR & 3; IR >>= 2;
      shift  = IR & 3; IR >>= 2;
      fn     = IR & 7; IR >>= 3;
      acd    = IR & 3; IR >>= 2;
      acs    = IR & 3; IR >>= 2;

      sprintf(mnemo,"%s%s%s%s",
         alu_fns[fn],alu_carrys[carry],alu_shifts[shift],alu_noloads[noload]);
      printf("%-6s %d,%d",mnemo,acs,acd);
      if (skip)
         printf(",%s",alu_skips[skip]);
   }
   else if (IR > 057777) { /* input/output */
      int dev, ctrl, tfer, ac;
      char *devname;
      Flag ispec;
      char *tmp;

      ispec = false;
      if ((tmp = special(IR))) {
         strcpy(mnemo, tmp);
         ispec = true;
      }

      dev  = IR & 077; IR >>= 6;
      ctrl = IR &   3; IR >>= 2;
      tfer = IR &   7; IR >>= 3;
      ac   = IR &   3; IR >>= 2;

      devname = io_devs(dev);

      if (!ispec)
         sprintf(mnemo,"%s%s",
            io_tfers[tfer],
            7 == tfer? io_skips[ctrl] : io_ctrls[ctrl]);
      if (7 == tfer)
         printf("%-6s %s",mnemo,devname);
      else {
         if (ispec)
            printf("%-6s %d",mnemo,ac);
         else
            printf("%-6s %d,%s",mnemo,ac,devname);
      }
   }
   else { /* memory/program flow */
      int disp, mode, indir, ac, fn;

      disp  = IR & 0377; IR >>= 8;
      mode  = IR &    3; IR >>= 2;
      indir = IR &    1; IR >>= 1;
      ac    = IR &    3; IR >>= 2;
      fn    = IR &    3;

      if (mode) {
         if (disp > 127) disp -= 256;
      }

      sprintf(mnemo,"%s",0 == fn? mem_acs[ac] : mem_fns[fn]);
      printf("%-6s ",mnemo);
      if (fn)
         printf("%d,",ac);
      printf("%s",mem_indirs[indir]);
      if (0 == mode)
         printf("%03o",disp);
      else if (1 == mode)
         printf("%s%03o ;%06o",disp<0? "-" : "+",ABS(disp),077777 & (pc+disp));
      else
         printf("%s%03o",disp<0? "-" : "+",ABS(disp));
      if (mode > 1)
         printf(",%d",mode);
   }
}

#define FF(ff,on) (ff?on:' ')

void showQUE(void)                              /* Show event QUE */
{
   Event *evt;
   Device *dev;

   evt = QUE;
   while (evt) {
      ASSERT(0 != evt->devno);
      dev = &devs[evt->devno];
      printf("%s BUFs: %06o %06o %06o %c%c%c%c %02d %6d %6d %s\n",
         io_devs(dev->devno),
         dev->buf[BUF_A], dev->buf[BUF_B], dev->buf[BUF_C],
           dev->busy? 'B' : ' ',
           dev->done? 'D' : ' ',
         dev->intdis? 'I' : ' ',
         dev->intreq? 'R' : ' ',
         dev->line,
         dev->delay, evt->timeout,
         ctrlfns[evt->ctrlfn]);
      evt = evt->que;
   }
}

#define NIC 18
Word IC[NIC]; /* internal cells */

#define IC_AC0 0
#define IC_AC1 1
#define IC_AC2 2
#define IC_AC3 3
#define IC_PC  4
#define IC_CRY 5
#define IC_ION 10
#define IC_SW  12
#define IC_MSKO 17

#define LO(n)  ((n) & 0377)
#define HI(n)  LO((n) >> 8)

void regs2ic(void)
{
   int i;

   for (i = 0; i < NIC; i++) IC[i] = 0;

   IC[IC_AC0] = AC[0];
   IC[IC_AC1] = AC[1];
   IC[IC_AC2] = AC[2];
   IC[IC_AC3] = AC[3];
   IC[IC_PC]  = PC;
   IC[IC_CRY] = CRY? 1 : 0;
   IC[IC_ION] = ION? 1 : 0;
   IC[IC_SW]  = SW;
   IC[IC_MSKO]= MSKO;
}


void ic2regs(void)
{
   AC[0] = IC[IC_AC0] = AC[0];
   AC[1] = IC[IC_AC1] = AC[1];
   AC[2] = IC[IC_AC2] = AC[2];
   AC[3] = IC[IC_AC3] = AC[3];
      PC = IC[IC_PC]  & 077777;
     CRY = IC[IC_CRY] & 1? true : false;  /* transfer bit15 */
     ION = IC[IC_ION] & 1? true : false;
      SW = IC[IC_SW];
    MSKO = IC[IC_MSKO];
}

int cellFmt;

void dispcell(Word adr,Word w)
{
   int i;

   switch (cellFmt) {
   case '!':
      printf("!");
      for (i = 0; i < 16; i++) {
         printf("%s",w & 0100000? "1" : "0");
         w <<= 1;
      }
      break;
   case '=': printf("=%06o",w); break;
   case '%': printf("%%%05d",w); break;
   case '$': printf("$%04x",w); break;
   case '_': printf("%03o_%03o",HI(w),LO(w)); break;
   case '|': printf("%03d|%03d",HI(w),LO(w)); break;
   case ':': printf("%02x:%02x",HI(w),LO(w)); break;
   case '\'':printf("%c'%c",HI(w),LO(w)); break;
   case '&': printf("&%06o %d",w >> 1,w & 1); break;
   case ';': printf("%06o  ; ",w); disasm(adr,w); break;
   }
}

void showregs(int vc,int nl)
{
   if (vc)
      printf("PC: %06o   ACs: %06o %06o %06o %06o  %s%s",
            IC[IC_PC],
            IC[IC_AC0],IC[IC_AC1],IC[IC_AC2],IC[IC_AC3],
            IC[IC_CRY]? "C":" ",
            IC[IC_ION]? "I":" "
      );
   else
      printf("PC: %06o   ACs: %06o %06o %06o %06o  %s%s",
            PC,
            AC[0],AC[1],AC[2],AC[3],
            CRY? "C":" ",
            ION? "I":" "
      );
   if (nl) printf("\n");
}

void show(int cello,int adr)
{
   adr &= 077777;
   printf("%06o %s ",adr,cello<0? "A" : "/");
   dispcell(adr,cello<0? IC[adr] : M[adr]);
   printf("  ");
}

void showsym(int vc)
{
   Word IR;
   Word adr;

   adr = 077777 & PC;
   if (!vc) {
      if (T[adr] >= Trace)
         return;
      T[adr]++;
   }
   IR = M[adr];
   showregs(vc,0);
   printf("  %06o  ",IR);
   disasm(adr,IR);
   printf("\n");
}

char* getexpr(char *p, int *pxpr, int* px)      /* Get expression value */
{
   int ret, x, d, base;
   int ndig;
   int sign;
   int op;

   ret = 0; op = ' ';                           /* retval is zero, op is SP */
Loop:
   x = 0; *pxpr = 0; base = 8; sign = 0;        /* init:positive oct, no expr */
   if ('-' == *p) {                             /* negative?               */
      sign = 1; p++;                            /* remember, skip char     */
   }
   switch (*p) {                                /* check number base       */
   case '=': base =  8; p++; break;             /* =octal                  */
   case '%': base = 10; p++; break;             /* %decimal                */
   case '$': base = 16; p++; break;             /* $hexadecimal, 4digit    */
   case '"':                                    /* "cc two ASCII           */
      *pxpr = 1; p++;                           /* have expr, skip char    */
      x = *p++;                                 /* 1st ASCII, skip char    */
      x = (x << 8) + *p++;                      /* 2nd ASCII, skip char    */
      goto Out;                                 /* done                    */
   }
   ndig = 0;                                    /* number of digits zero   */
   d = toupper(*p);                             /* make upper 1st char     */
   while (isdigit(d) ||                         /* digit?                  */
          (16 == base && 'A' <= d && d <= 'F')) /*    OR hexadecimal char? */
   {
      *pxpr = 1; ndig++; p++;                   /* have expr, count digit, */
                                                /*    skip char            */
      if (d > '9') d = 10 + d - 'A';            /* convert alpha           */
      else d = d - '0';                         /*    else convert digit   */
      if (d > base - 1)                         /* digit gt than allowed?  */
         return NULL;                           /*    return error         */

      x = base * x + d;                         /* add digit               */
      if (16 == base && 4 == ndig)              /* 4 hex digits?           */
         break;                                 /*    quit loop            */
      d = toupper(*p);                          /* get next upper char     */
   }

Out:
   x = sign? -x : x;                            /* use sign                */
        if (' ' == op) ret  = x;                /* 1st op, set value       */
   else if ('+' == op) ret += x;                /* increment by value      */
   else if ('-' == op) ret -= x;                /* decrement by value      */
   if (*p == '+' || *p == '-') {                /* next char is op?        */
      op = *p++; goto Loop;                     /* save op, next number    */
   }
   *px = ret;                                   /* return value            */
   return p;                                    /* return ptr              */
}

void usage(void)
{
   printf("Commands:\n");
   printf("---------\n");
   printf("   [expr]A      open internal cell/show regs\n");
   printf("   [expr]B      set/display breakpoint\n");
   printf("   [expr]D      delete [exp] breakpoint\n");
   printf("   E            show event QUE\n");
   printf("   F<char>      set cell fmt\n");
   printf("   [expr]H      set/display instr. history, 0 - clear\n");
   printf("   I            I/O reset\n");
   printf("   K            cancel line\n");
   printf("   <expr>L      program load, bit0=1 - data channel load\n");
   printf("   [<expr>]M    set/display memory cycle [300..2600] nS\n");
   printf("                0< - exact timing\n");
   printf("   [expr]O      step\n");
   printf("   P            proceed\n");
   printf("   [expr]Q      quit YaNOVA\n");
   printf("   <expr>R      run\n");
   printf("   <expr>Sfile  slurp <expr> sectors from file\n");
   printf("   [expr]T      set/clear instr. trace\n");
   printf("   <expr>X      modify device delay [1..100000] uS\n");
   printf("   [expr]Z      set/display KIPS (#instr/mS)\n");
   printf("   ?            display help\n");
   printf("   !cmd         execute shell command\n");

   printf("\nCell navigation:\n");
   printf("----------------\n");
   printf("   <expr>/    open memory cell\n");
   printf("   <CR>       next cell\n");
   printf("   ^          previous cell\n");
   printf("   @          indirect cell\n");
   printf("   !          close cell\n");

   printf("\nInternal cells:\n");
   printf("---------------\n");
   printf("   0-3       AC0-AC3\n");
   printf("     4       PC\n");
   printf("     5       Carry in bit15\n");
   printf("    10       Interrupt On in bit15\n");
   printf("    12       Console switch\n");
   printf("    17       Interrupt Mask (last MSKO)\n");

   printf("\n   <expr> := <expr> [+ | - <expr>]\n");
   printf("   <expr> := [=]oct | %%dec | $hex | \"cc\n");

   printf("\nCell format:\n");
   printf("------------\n");
   printf("   ! binary           ' two ASCII\n");
   printf("   = octal            _ two octal\n");
   printf("   %% decimal          | two decimal\n");
   printf("   $ hexadecimal      : two hexadecimal\n");
   printf("   & byte pointer     ; symbolic\n\n"); 
}

void strupper(char *s)
{
   if (!s)
      return;

   while (*s) {
      *s = toupper(*s);
      s++;
   }
}

void vconsole(void)
{
   int  i;
   int  err;   /* err flag */
   int  cello; /* cell open: 0< - memory, 0> - internal */
   int  dly;   /* delay, adr contains dev */
   int  xpr;   /* cmd has <expr> prefix */
   Word adr,x; /* value of <expr>, cell address, 16bit value of <expr> */
   int  xx;    /* value of <expr> */
   char line[80], *cmd;

   H = NULL; Trace = 0;
   dly = cello = 0; cellFmt = '=';
   err = 0;
   for(;;) {
      if (Halt) {
         printf("<%s>\n",halts[Halt]);
         regs2ic();
         showsym(1);
         HALT(H_Run);
      }
      if (err) {
         printf("?\n");
         cello = dly = 0;
         err = 0;
      }
      printf("! ");
      if (cello) show(cello, adr);

      if (NULL == fgets(line,80,stdin))
         exit(0);
      x = strlen(line);
      if (x && line[x-1] == '\n')
         line[x-1] = '\0';

      cmd = line;
      VCDBG(printf("cmd = [%s]\n",cmd));
      while (isspace(*cmd)) cmd++;
      VCDBG(printf("isspace cmd = [%s]\n",cmd));
      cmd = getexpr(cmd, &xpr, &xx);
      VCDBG(printf("getnum  cmd = [%s] xpr=%d xx=%d\n",cmd, xpr, xx));
      if (dly) {
         if (xpr) {
            if (0 < xx && xx <= 100000)
               devs[adr].delay = xx;
            printf("%s delay = %d\n",io_devs(adr),devs[adr].delay);
         }
         dly = 0;
         continue;
      }

      x = xx & 0177777;

      if (cello) {
         if (xpr) {
            if (cello < 0) IC[adr] = x;
            else M[adr] = x;
         }
         VCDBG(printf("cello cmd=[%s]\n",cmd));
         switch (*cmd) {
         case  '!': cello = 0; break;
         case ASC_LF:
         case '\0': adr = (adr + 1); break;       /* CR */
         case  '^': adr = (adr - 1); break;
         case  '@': adr = cello<0? IC[adr] : M[adr];
                    cello = 1;
                    break;
         default:  err = 1;
         }
         if (cello < 0) adr = adr % NIC;
         else adr = adr & 077777;
         continue;
      }

      if (!*cmd)
         continue;

      *cmd = toupper(*cmd);
      if ('!' != *cmd && 'S' != *cmd)
         strupper(cmd);
      if (strchr(cmd,'K')) continue;

      if (NULL != strchr("/LRSX",*cmd) && !xpr) { /* arg required */
         err = 1; continue;
      }

      VCDBG(printf("switch cmd = [%s]\n",cmd));
      switch (*cmd++) {
      case 'A': /* open internal cell */
         if (xpr) { cello = -1; adr = x; }
         else showregs(1,1);
         break;
      case '/': /* open memory cell */
         cello = 1; adr = x;
         break;
      case 'B': /* set/display breakpoints */
         if (xpr) {
            BKPT[nBKPT++] = x;
            BRK[x >> 4] |= 1 << (x & 15);
         }
         else
            for (i = 0; i < nBKPT; i++)
               printf("%d %06o",i,BKPT[i]);
         break;
      case 'D': /* delete/all breakpoints */
         if (xpr) {
            x &= MAX_BKPT-1;
            BRK[x >> 4] &= ~(1 << (x & 15));
            for (i = x; i < MAX_BKPT; i++)
               BKPT[i] = BKPT[i+1];
            nBKPT--;
         }
         else {
            for (i = 0; i < sizeof(BRK) / sizeof(Word); i++)
               BRK[i] = 0;
            nBKPT = 0;
         }
         break;
      case 'E': /* show event QUE */
         showQUE();
         break;
      case 'F': /* change cell format */
         if (strchr("!=%$_|:'&;",*cmd)) cellFmt = *cmd;
         else err = 1;
         break;
      case 'H': /* show/set history */
         if (xpr) {
            if (H) {
               free(H); H = NULL;
            }
            if (x) {
               if (x < 32) x = 32;
               x &= 0177777;
               H = (HE*)calloc(x, sizeof(HE));
               if (H) {
                  nH = x; posH = 0;
               }
               else printf("not enough memory!\n");
            }
         }
         else if (H) showH(posH-21);
         break;
      case 'I': /* I/O reset */
         IORST();
         break;
      case 'K': /* kill line */
         break;
      case 'L': /* program load */
         IC[IC_SW] = x;
         ic2regs(); load(); regs2ic();
         break;
      case 'M': /* display/set memory cycle time */
         if (xpr) {
            if (300 <= x && x <= 2600) {
               memTime  = x / 100;
               memCycle = xx < 0? true : false;
            }
         }
         printf("memory cycle time = %ldns%s\n",
                  100 * memTime,
                  memCycle? " (exact)" : ""
         );
         break;
      case 'O': /* step */
         if (!xpr) x = 1;
         ic2regs();
         for (i = 0; (i < x) && !Halt; i++) {
            step(); showsym(0);
         }
         regs2ic();
         break;
      case 'P': /* proceed */
         ic2regs(); run(); regs2ic();
         break;
      case 'Q': /* quit */
         exit(xpr ? x : 1); break;
      case 'R': /* run */
         IC[IC_PC] = 077777 & x;
         ic2regs(); run(); regs2ic();
         break;
      case 'S': /* slurp n sectors at PC */
         {
            FILE *fd;
            fd = fopen(cmd,"rb");
            if (NULL != fd) {
               int i;
               Word w, adr = IC[IC_PC];
               if (x > 128) x = 128;
               for (i = 0; i < x * SEC_SIZE; i++) {
                  fread(&w, sizeof(Word), 1, fd);
                  memWriteIO(adr++, SWAP(w));
               }
               fclose(fd);
            }
            else printf("cannot open %s!\n",cmd);
         }
         break;
      case 'T': /* set/clear instr. trace */
         if (xpr) { Trace = xx; }
         printf("trace = %o\n", Trace);
         break;
      case 'X': /* modify device delay */
         adr = 077 & x;
         if (DEV_MDV == adr || DEV_CPU == adr || NO_LINE == devs[adr].line)
            printf("%s cannot set delay",io_devs(adr));
         else {
            printf("%s delay=%d ",io_devs(adr),devs[adr].delay);
            dly = 1;
         }
         break;
      case 'Z': /* set/display KIPS */
         if (xpr && (0 <= x))
            KIPS = x;
         printf("KIPS = %d\n", KIPS);
         break;
      case '?': /* display help */
         usage();
         break;
      case '!': /* exec shell command line */
         system(cmd);
         break;
      default: err = 1;
      }
   }
}

#define MAX_OPTS  64

int parseOpt(char *kv, Option *opt)             /* Parse option: key=value */
{
   char *asgn;
   char tmp[16];

   asgn = strchr(kv,'=');                       /* find equal sign   */

   if ( 0 == asgn) return 1;                    /* no equal? error   */
   if (kv == asgn) return 2;                    /* no key? error     */
   if ( 0 == strlen(asgn + 1)) return 3;        /* no value? error   */
   if (asgn - kv > 15) return 4;                /* key too long? error */
   opt->value = asgn + 1;                       /* get value         */
   strncpy(tmp, kv, asgn-kv);                   /* get key           */
   tmp[asgn-kv] = '\0';
   opt->key   = strdup(tmp);

   return 0;                                    /* done */
}

void initOpts(int argc, char *argv[])
{
   int i;
   char buf[8];
   Byte devno;
   int  nopts;
   Option opts[MAX_OPTS];

   nopts = 0;

   for (i = 1; i < argc; i++) {
      char *arg = argv[i];
      if ('-' != *arg)
         continue;
      arg++;
     
      if (strlen(arg) < 3) {
         fprintf(stderr,"%s unknown option!",arg);
         exit(1);
      }
      strncpy(buf,arg,3);
      buf[3] = '\0'; 

      devno = str2dev(buf);
      if (0 == devno) {
         fprintf(stderr,"%s device not found!",buf);
         exit(1);
      }
      if (nopts == MAX_OPTS) {
         fprintf(stderr,"too many options!");
         exit(1);
      }
      opts[nopts].devno = devno;
      if (parseOpt(arg + 3, &opts[nopts])) {
         fprintf(stderr,"%s invalid option!",arg+3);
         exit(1);
      }
      nopts++;
   }

   initDevs(nopts, &opts[0]);
}

int main(int argc, char*argv[])
{
   int i;

   B = (unsigned char*)&M[0];

   printf("YaNOVA V%d.%d",VER_VER,VER_REL);
   if (VER_LVL)
      printf("P%d",VER_LVL);
   printf("\n");

   /*
    * Nova        2600ns core memory
    * Nova        2400ns read-only memory
    * Nova 1200   1200ns
    * Nova 800     800ns
    * Supernova    800ns
    * Supernova SC 300ns
   */

   memTime     = 26; memCycle = false;
   elapsedTime = 0;
   QUE = NULL; ioTime = 0;
   MissingDev = true;

   gettimeofday(&tv0, NULL);
   KIPS = 192;
   numInstr  = 0;
   syncInstr = numInstr + KIPS;

   initOpts(argc, argv);

   srandom(1969);
   for (i = 0; i < sizeof(M) / sizeof(Word); i++) {
      M[i] = random() & 0177777;
      T[i] = 0;
   }

   HALT(H_Startup);
   vconsole();

   return 0;
}
