# yaNOVA
This is the beginnings of a DG NOVA simulator.
The project goal is to run Mapped RDOS.

Options:
  * set storage file for device, -DEVfile[n]=path
  * set DSK write-protect, -DSKprotect=num
  * set drive read-only, -DEVrdonly=0|1

Changes:
  * read-only drives
  * DSK simulation (not debugged), cached in memory, write protection
  * LPT simulation (not debugged), file I/O
  * inital revision, somewhat working virtual console, not debugged.

**NOTE**: Good luck!
