# yaNOVA
This is the beginnings of a DG NOVA simulator.
The project goal is to run Mapped RDOS.

Options:
  * set storage file for device, -DEVfile[n]=path
  * set DSK write-protect, -DSKprotect=num
  * set disc read-only, -DEVrdonly=0|1
  * set disc controller shared, -DEVshared=0|1

Changes:
  * fixed some DKP behavior, CPU Carry generation, got RDOS "DISK ERR=000000"
  * Ctrl-E break into virtual console
  * sector load into memory
  * instruction trace
  * DKP controller (not debugged), unlock timeout
  * standalone event queue, each device can fire multiple events
  * shared disc controllers
  * read-only drives
  * DSK simulation (not debugged), cached in memory, write protection
  * LPT simulation (not debugged), file I/O
  * inital revision, somewhat working virtual console, not debugged.

**NOTE**: Good luck!
