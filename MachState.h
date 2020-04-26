#ifndef _H_MACHSTATE
#define _H_MACHSTATE

#include <stddef.h>
#include <functional>

typedef uint8_t machinestates_t;

static const machinestates_t BOOTING = 0,                  /* Startup state */
                             OUTOFORDER = 1,               /* device not functional.  */
                             REBOOT = 2,                   /* forcefull reboot  */
                             TRANSIENTERROR = 3,           /* hopefully goes away level error  */
                             NOCONN = 4,                   /* sort of fairly hopless (though we can cache RFIDs!)  */
                             WAITINGFORCARD = 5,           /* waiting for card. */
                             CHECKINGCARD = 6,             /* checking card. with server */
                             SWITCHEDOFF = 7,              /* unit is switched off completely */
							 POWERED = 8,                  /* unit is powered on */
							 RUNNING = 9;                 /* unit is running (opto sees light). */


#endif