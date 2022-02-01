/* 
 * DG Nova
 * =======
 *
*/

int DBG=0;

#define VER_VER   0
#define VER_REL   0
#define VER_LVL   1

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <time.h>                               /* nanosleep()    */
#include <sys/time.h>                           /* gettimeofday() */
#include <unistd.h>                             /* usleep()       */

typedef unsigned char Byte;
typedef unsigned short Word;
typedef unsigned int DWord;
typedef enum{false,true} Flag;

#define ABS(x)    ((x)<0? -(x):(x))

void Assert(char *path,int lno, char *msg, int cond)
{
   if (!cond) {
      fprintf(stderr,"assert at %s:%d: %s\n",path,lno,msg);
      exit(1);
   }
}
#define ASSERT(cond) Assert(__FILE__,__LINE__,""#cond,cond)

/* --- D E V I C E S -------------------------------------------------------- */

#define NDEV      0100

#define DEV_MDV   (001)
#define DEV_TTI   (010)
#define DEV_TTO   (011)
#define DEV_PTR   (012)
#define DEV_PTP   (013)
#define DEV_RTC   (014)
#define DEV_LPT   (017)
#define DEV_DKP   (033)
#define DEV_CPU   (077)

#define HZ        50

#define BUF_A     0
#define BUF_B     1
#define BUF_C     2

#define NO_LINE   ((Byte)255)

typedef enum {IOC_NONE, IOC_START, IOC_CLEAR, IOC_PULSE} CtrlFn;

typedef struct _Device {
   Byte code;        /* device code */
   Word buf[3];
   Flag busy, done;  /* device status: 00 - idle, 10 - working, 01 - done */
   Flag intdis;      /* INT disable flag */
   Flag intreq;      /* INT request */
   Byte line;        /* bus line: 0-15, NO_LINE - missing device */
   int  delay;       /* device delay */
   int  timeout;     /* when reaches 0, done is set */
   CtrlFn ctrlfn;    /* control function, which started the operation */
   struct _Device *bus; /* next device on I/O bus */
   struct _Device *que; /* next device on event queue */
} Device;


/* time measured in 0.1us */
unsigned long elapsedTime; /* elapsed time 0.1us */
unsigned long memTime;     /* memory time */
Flag memCycle;             /* cycle exact memory time */
unsigned long ioTime;      /* updated at every I/O Start/Pulse */
                           /* all QUE timeouts are relative to this time */
unsigned long eventTime;   /* next event time in QUE */
unsigned long numInstr;    /* number of instructions executed */

Device *BUS;         /* I/O bus, fastest device(s) first */
Device *QUE;         /* event queue, most recent device event first */
int IREQ;            /* no. of pending INT requests */
Flag delayION;       /* delayed ION */
Device devs[64];

enum {H_Run,H_Startup,H_Halt,H_Break,H_Undef,H_Indir} Halt;
char *halts[] = {
   "run",
   "system startup",
   "halt",
   "break",
   "undefined instruction",
   "indirection chain"
};

int  nBKPT;  /* breakpoints */
#define MAX_BKPT   8
Word BKPT[MAX_BKPT];
Word BRK[2048];

#define BRAKE(x)  (BRK[(x)>>4] & (1 << ((x) & 15)))

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
Device* insertQUE(int t)
{
   Device *p, *q;

   p = NULL; q = QUE;
   while (q) {
      if (t < q->timeout)
         return p;
      p = q;
      q = q->que;
   }
   return p;
}

void devClear(int dev);                         /* Forward declarations */
void devStart(int dev);
void devPulse(int dev);
void doIO(int dt);
void disasm(Word IR);

void IORST(void)                                /* IORST */
{
   Device *q;

   q = BUS;                                     /* clear ctrl FFs in all devs */
   while (q) {
      devClear(q->code);                        /* Clear dev (intreq cleared) */
      q->intdis = false;                        /* clear IntDisable */
      q->que = NULL;                            /* clear event QUE ptr */
      q = q->bus;                               /* next device on BUS */
   }
   QUE = NULL;                                  /* clear the event QUE */
}

Word devIN(int dev, int reg)                    /* Device INPUT */
{
   Word w;
   Device *q;

   w = devs[dev].buf[reg];                      /* default action */

   switch (dev) {
   case DEV_CPU:
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
               w = q->code;
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
   }

   return w;
}

void devOUT(int dev, int reg, Word w)           /* Device OUTPUT */
{
   Device *q;

   if (NO_LINE != devs[dev].line)               /* device installed? */
      devs[dev].buf[reg] = w;                   /* set buffer        */

   switch (dev) {
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
         Halt = H_Halt;                         /* Halt the processor. */
         break;
      }
      break;
   }
}

void enqueIO(int dev, CtrlFn fn)                /* Enqueue I/O operation */
{
   Device *q;

   ASSERT(NO_LINE != devs[dev].line);           /* ensure device connected */
   ASSERT(0 < devs[dev].delay);                 /* has delay set        */
   ASSERT(IOC_START == fn || IOC_PULSE == fn);  /* only Start or Pulse  */

   devs[dev].ctrlfn  = fn;                      /* set control function */
   devs[dev].timeout = 10 * devs[dev].delay;    /* set timeout in 0.1us */

   if (QUE)                                     /* QUE not empty?   */ 
      doIO(elapsedTime - ioTime);               /* step all devs[]  */

   q = insertQUE(devs[dev].timeout);            /* get insert pos */
   if (NULL == q) {                             /* be the first one */
      devs[dev].que = QUE;                      /* insert dev */
      QUE = &devs[dev];
   }
   else {
      devs[dev].que = q->que;                   /* insert after `q' */
      q->que = &devs[dev];
   }
   ioTime = elapsedTime;                        /* reset ioTime     */
   eventTime = ioTime + QUE->timeout;           /*    and eventTime */
}

void devStart(int dev)                          /* Start device `dev' */
{
   if (NO_LINE == devs[dev].line)               /* no such device */
      return;                                   /*    do nothing  */

   if (devs[dev].busy)                          /* device busy?   */
      return;                                   /*    do nothing. */

   if (DEV_CPU == dev)                          /* this is the CPU?  */
      delayION = true;                          /* execute delayed INTEN */
   else {
      switch (dev) {
      case DEV_RTC:                             /* RTC? */ 
         {
            int freq;
            switch (devs[dev].buf[BUF_A] & 3) { /* set-up frequency */
            case 0: freq =   HZ; break;
            case 1: freq =   10; break;
            case 2: freq =  100; break;
            case 3: freq = 1000; break;
            }
            devs[dev].delay = 1000000 / freq;   /* delay according to freq */
         }
         break;
      }

      devs[dev].busy = true;                    /* Busy=1,Done=0 */
      devs[dev].done = false;

      enqueIO(dev, IOC_START);                  /* enqueue I/O */
   }
}

void devClear(int dev)                          /* Clear device `dev' */ 
{
   if (DEV_CPU != dev)                          /* clear buffers */
      devs[dev].buf[BUF_A] = 0;                 /* except CPU buffer A (SW) */
   if (DEV_MDV != dev)
      devs[dev].buf[BUF_B] = 0;                 /* MDV just clear buffer A */
   if (DEV_MDV != dev)
      devs[dev].buf[BUF_C] = 0;

   devs[dev].busy = false;                      /* clear control FFs */
   devs[dev].done = false;

   if (devs[dev].intreq) {                      /* INT request pending?  */
      devs[dev].intreq = false; IREQ--;         /* clear dev and CPU flag*/
   }
}

void devPulse(int dev)                          /* Pulse device `dev' */
{
   switch (dev) {
   case DEV_MDV:                                /* MDV MUL */
      enqueIO(dev, IOC_PULSE);
      break;
   }
   return;
}

void devDone(int dev)                           /* Device done, ie timed out */
{
   int ch;

   ASSERT(NO_LINE != devs[dev].line);           /* ensure device present */
   ASSERT(NULL == devs[dev].que);               /* removed from event queue */

   if (DEV_MDV != dev) {
      devs[dev].done = true;                    /* Done=1,Busy=0 */
      devs[dev].busy = false;
   }

   switch (dev) {
   case DEV_MDV:
      switch (devs[dev].ctrlfn) {
      case IOC_NONE:
         break;
      case IOC_START:                           /* MDV DIV */
         {
            DWord dw;

            dw = devs[dev].buf[BUF_A];
            dw = (dw << 16) + devs[dev].buf[BUF_B];
            devs[dev].buf[BUF_A] = dw % devs[dev].buf[BUF_C];
            devs[dev].buf[BUF_B] = dw / devs[dev].buf[BUF_C];
         }
         break;
      case IOC_CLEAR:
         break;
      case IOC_PULSE:                           /* MDV MUL */
         {
            DWord result;

            result = devs[dev].buf[BUF_A] +
                     (DWord)devs[dev].buf[BUF_B] * devs[dev].buf[BUF_C];
            devs[dev].buf[BUF_A] = 0177777 & (result >> 16);
            devs[dev].buf[BUF_B] = 0177777 & result;
         }
         break;
      }
      break;
   case DEV_TTO:                                /* TTO? */
      ch = toupper(0377 & devs[dev].buf[BUF_A]);/* write upper case char */
      printf("%c",ch);
      break;
   case DEV_TTI:                                /* TTI? */
      ch = toupper(0377 & getchar());
      devs[dev].buf[BUF_A] = ch;                /* read upper case char */
      break;
   }

   devs[dev].ctrlfn = IOC_NONE;                 /* clear control function */

   if (true == devs[dev].done                   /* Done set?                */
       && false == devs[dev].intdis)            /*    and IntDisable clear? */
   {
      devs[dev].intreq = true; IREQ++;          /* trigger an INT request */
   }
}

/* process I/O event queue
 * - take into account elapsed time
 * - all devices whose timeout lapsed, fired
*/
void doIO(int dt)                               /* Do Input/Output   */
{
   Device *q;

   q = QUE;                                     /* elapsed dt time   */
   while (q) {
      q->timeout -= dt;
      q = q->que;
   }

   while (QUE && (QUE->timeout <= 0)) {         /* at the front time lapsed */
      q = QUE; QUE = q->que;                    /* remove device from queue */
      q->que = NULL;                            /* ensure ptr cleared       */
      devDone(q->code);                         /* do actual I/O            */
   }

   ioTime = elapsedTime;                        /* reset ioTime            */
   if (QUE)                                     /* if QUE not empty        */
      eventTime = ioTime + QUE->timeout;        /*    reset eventTime also */
}

void initDevs(void)
{
   int i;
   Device *q;

   static struct {
      int dev;
      int line;
      int delay;
   } inits[] = {
      {DEV_MDV, 16,     7},  /* 6.8us MUL, 7.2us DIV */
      {DEV_TTI, 14, 100000},
      {DEV_TTO, 15, 100000},
      {DEV_PTR, 11,   6667},
      {DEV_PTP, 11,  15873},
      {DEV_RTC, 13,  20000},  /* 50Hz */
      {DEV_CPU, 16,      0},  /* dummy line */
      {      0,  0,      0}
   };

   for (i = 0; i < NDEV; i++) {
      devs[i].code    = i;
      devs[i].intdis  = false;
      devs[i].intreq  = false;
      devs[i].line    = NO_LINE;
      devs[i].delay   = 0;
      devs[i].timeout = 0;
      devs[i].ctrlfn  = IOC_NONE;
      devs[i].bus = devs[i].que = NULL;
   }

   IREQ = 0;

   BUS = NULL;
   for (i = 0; inits[i].dev; i++) {
      int dev = inits[i].dev;
      devs[dev].line  = inits[i].line;
      devs[dev].delay = inits[i].delay;
      devClear(dev);

      /* don't insert CPU in I/O bus */
      if (DEV_CPU == dev)
         continue;

      q = insertBUS(devs[dev].delay);
      if (NULL == q) {
         devs[dev].bus = BUS;
         BUS = &devs[dev];
      }
      else {
         devs[dev].bus = q->bus;
         q->bus = &devs[dev];
      }
   }

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
   */
}


/* --- M E M O R Y ---------------------------------------------------------- */

Word M[32768];    /* memory */
unsigned char* B;

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
   cycle();
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

Word AC[4], PC;
Flag CRY;

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
   disasm(h->IR);
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
         Halt = H_Indir;
         return 0;
      }
   }
   return E;
}

unsigned int KIPS;                              /* instr/ms           */
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
   if (KIPS && numInstr >= syncInstr)           /* if KIPS set,          */
      syncKIPS();                               /* sync instruction time */

   if (H)                                       /* has history buffer ? */
      saveH(IR);                                /*    save instr.       */

   if (QUE && elapsedTime >= eventTime)         /* do Input/Output */
      doIO(elapsedTime - ioTime);

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
      Word aluA, result;
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

      aluA = 4 & fn? AC[acd] : 0;
      switch (fn & 3) {
      case 0: /* COM ADC */
         result = aluA + ~AC[acs];
         if (result < AC[acs]) cy = 1 - cy;
         break;
      case 1: /* NEG SUB */
         result = aluA - AC[acs];
         if (result < AC[acs]) cy = 1 - cy;
         break;
      case 2: /* MOV ADD */
         result = aluA + AC[acs];
         if (result < AC[acs]) cy = 1 - cy;
         break;
      case 3: /* INC AND */
         if (7 == fn)
            result = aluA & AC[acs];
         else {
            result = AC[acs] + 1;
            if (result < AC[acs]) cy = 1 - cy;
         }
         break;
      }

      switch (shift) {
      case 0: /* no shift/swap */
         break;
      case 1: /* L rotate left */
         ncy = 0177777 & result? 1 : 0;
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
      int dev, ctrl, tfer, ac;

      dev  = IR & 077; IR >>= 6;
      ctrl = IR &   3; IR >>= 2;
      tfer = IR &   7; IR >>= 3;
      ac   = IR &   3; IR >>= 2;

      if (tfer && tfer != 7) {
         if (tfer & 1)
            AC[ac] = devIN(dev,(tfer-1)/2);
         else
            devOUT(dev,(tfer-1)/2,AC[ac]);
      }

      switch (ctrl) {
      case 0: /* Do not activate */
         break;
      case 1: /* S start dev */
         devStart(dev);
         break;
      case 2: /* C clear dev */
         devClear(dev);
         break;
      case 3: /* P pulse dev */
         devPulse(dev);
         break;
      }

      if (0 == tfer || 7 == tfer) {
         int cond;

         cond = 0;
         switch (ctrl) {
         case 0: /* SKPBN */
            cond = devs[dev].busy != false; 
            break;
         case 1: /* SKPBZ */
            cond = devs[dev].busy == false;
            break;
         case 2: /* SKPDN */
            cond =  devs[dev].done != false;
            break;
         case 3: /* SKPDZ */
            cond = devs[dev].done == false;
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
         E = 077777 & (disp + PC);
         break;
      case 2: /* AC2 indexed */
         E = 077777 & (disp + AC[2]);
         break;
      case 3: /* AC3 indexed */
         E = 077777 & (disp + AC[3]);
         break;
      }

      E = indirectChain(E, indir);
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
   PC = 07777 & (PC + 1);
}

void step(void)
{
   Word IR;

   IR = memRead(PC);
   execute(IR);

   if (nBKPT && BRAKE(PC)) {
      Halt = H_Break; return;
   }
}

void run(void)
{
   while (!Halt)
      step();
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

   for (i = 0; i < 32; i++)
      M[i] = bootstrap[i];
   return;

   /* SuperNova-style loader */
   DIAS = (077 & SW) + 060500;

   execute(062677); /* IORST */
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

char *io_devs(int dev)
{
   static char buf[8];
   int i;
   static struct {
      int code;
      char *name;
   } devs[] = {
      {DEV_MDV, "MDV"},
      {DEV_TTI, "TTI"},
      {DEV_TTO, "TTO"},
      {DEV_PTR, "PTR"},
      {DEV_PTP, "PTP"},
      {DEV_RTC, "RTC"},
      {DEV_LPT, "LPT"},
      {DEV_DKP, "DKP"},
      {DEV_CPU, "CPU"},
      {      0, "NULL"}
   };
   
   for (i = 0; NULL != devs[i].name; i++)
      if (dev == devs[i].code)
         return devs[i].name;
   sprintf(buf, "%02o", dev);
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

void disasm(Word IR)
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
      else
         printf("%s%03o",disp<0? "-" : "+",ABS(disp));
      if (mode > 1)
         printf(",%d",mode);
   }
}

#define FF(ff,on) (ff?on:' ')

void showQUE(void)                              /* Show event QUE */
{
   static char* ctrlfns[] = {"NONE", "START", "CLEAR", "PULSE"};
   Device *q;

   q = QUE;
   while (q) {
      printf("%s BUFs: %06o %06o %06o %c%c%c%c %02d %6d %6d %s\n",
         io_devs(q->code),
         q->buf[BUF_A], q->buf[BUF_B], q->buf[BUF_C],
           q->busy? 'B' : ' ',
           q->done? 'D' : ' ',
         q->intdis? 'I' : ' ',
         q->intreq? 'R' : ' ',
         q->line,
         q->delay, q->timeout,
         ctrlfns[q->ctrlfn]);
      q = q->que;
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

void dispcell(Word w)
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
   case ';': printf("%06o  ; ",w); disasm(w); break;
   }
}

void showregs(int nl)
{
   printf("PC: %06o   ACs: %06o %06o %06o %06o  %s%s",
         IC[IC_PC],
         IC[IC_AC0],IC[IC_AC1],IC[IC_AC2],IC[IC_AC3],
         IC[IC_CRY]? "C":" ",
         IC[IC_ION]? "I":" "
   );
   if (nl) printf("\n");
}

void show(int cello,int adr)
{
   adr &= 077777;
   printf("%06o %s ",adr,cello<0? "A" : "/");
   dispcell(cello<0? IC[adr] : M[adr]);
   printf("  ");
}

void showsym(void)
{
   Word IR;

   IR = M[077777 & PC];
   showregs(0);
   printf("  %06o  ",IR);
   disasm(IR);
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
   printf("   [expr]A    open internal cell/show regs\n");
   printf("   [expr]B    set/display breakpoint\n");
   printf("   [expr]D    delete [exp] breakpoint\n");
   printf("   E          show event QUE\n");
   printf("   F<char>    set cell fmt\n");
   printf("   [expr]H    set/display instr. history, 0 - clear\n");
   printf("   I          I/O reset\n");
   printf("   K          cancel line\n");
   printf("   <expr>L    program load, bit0=1 - data channel load\n");
   printf("   [<expr>]M  set/display memory cycle [300..2600] nS\n");
   printf("              0< - exact timing\n");
   printf("   [expr]O    step\n");
   printf("   P          proceed\n");
   printf("   [expr]Q    quit YaNOVA\n");
   printf("   <expr>R    run\n");
   printf("   <expr>X    modify device delay [1..100000] uS\n");
   printf("   [expr]Z    set/display KIPS (#instr/mS)\n");
   printf("   ?          display help\n");
   printf("   !cmd       execute shell command\n");

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

#define CR  015
#define LF  012

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

   H = NULL;
   dly = cello = 0; cellFmt = '=';
   err = 0;
   for(;;) {
      if (Halt) {
         printf("<%s>\n",halts[Halt]);
         showsym();
         Halt = H_Run;
      }
      if (err) {
         printf("?\n");
         cello = dly = 0;
      }
      printf("! ");
      if (cello) show(cello, adr);

      if (NULL == fgets(line,80,stdin))
         exit(0);

      cmd = line;
if (DBG) fprintf(stderr,"cmd = [%s]\n",cmd);
      while (isspace(*cmd)) cmd++;
if (DBG) fprintf(stderr,"isspace cmd = [%s]\n",cmd);
      cmd = getexpr(cmd, &xpr, &xx);
if (DBG) fprintf(stderr,"getnum  cmd = [%s] xpr=%d xx=%d\n",cmd, xpr, xx);
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
if (DBG) fprintf(stderr,"cello cmd=[%s]\n",cmd);
         switch (*cmd) {
         case  '!': cello = 0; break;
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

      strupper(line);
      if (strchr(line,'K')) continue;

      if (NULL != strchr("/LRX",*cmd) && !xpr) { /* arg required */
         err = 1; continue;
      }

if (DBG) fprintf(stderr,"switch cmd = [%s]\n",cmd);
      switch (*cmd++) {
      case 'A': /* open internal cell */
         if (xpr) { cello = -1; adr = x; }
         else showregs(1);
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
            step(); showsym();
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
         if (xpr && (0 < x))
            KIPS = x;
         printf("KIPS = %d\n", KIPS);
         break;
      case '?': /* display help */
         usage();
         break;
      case '!': /* exec shell command line */
         system(++cmd);
         break;
      default: err = 1;
      }
   }
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

   gettimeofday(&tv0, NULL);
   KIPS = 192;
   numInstr  = 0;
   syncInstr = numInstr + KIPS;

   srandom(1969);
   for (i = 0; i < sizeof(M) / sizeof(Word); i++)
      M[i] = random() & 0177777;

   initDevs();
   Halt = H_Startup;
   vconsole();

   return 0;
}
