#ifndef _H_MACHSTATE
#define _H_MACHSTATE

#include <stddef.h>
#include <functional>

typedef uint8_t machinestates_t;

static const machinestates_t BOOTING = 1,                  /* Startup state */
                             OUTOFORDER = 2,               /* device not functional.  */
                             REBOOT = 3,                   /* forcefull reboot  */
                             TRANSIENTERROR = 4,           /* hopefully goes away level error  */
                             NOCONN = 5,                   /* sort of fairly hopless (though we can cache RFIDs!)  */
                             WAITINGFORCARD = 6,           /* waiting for card. */
                             CHECKINGCARD = 7,             /* checking card. with server */
                             SWITCHEDOFF = 8,              /* unit is switched off completely */
							 POWERED = 9,                  /* unit is powered on */
							 RUNNING = 10;                 /* unit is running (opto sees light). */


#endif