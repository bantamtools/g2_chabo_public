/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "main.h"//
#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
/* this results in hard fault when sent ovr from desktop sw on the USB
 {"sr":{"mpox":t,"mpoy":t,"mpoz":t,"mpoa":t,"posx":t,"posy":t,"posz":t,"posa":t,"unit":t,"stat":t,"coor":t,"momo":t,"dist":t,"home":t,"mots":t,"plan":t,"line":t,"path":t,"frmo":t,"prbe":t,"safe":t,"estp":t,"spc":t,"hold":t,"macs":t,"cycs":t,"sps":t,"feed":t, "fro":t, "spo":t, "mark":t}}

 */
// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************
#ifdef DEPLOY_HARD_FAULT_REPORTING
volatile HardFaultInfoS hard_fault_info __attribute__ ((persistent));
#endif

int main ( void )
{
  // Initialize all modules
  SYS_Initialize ( NULL );
 
#if 1//substitute bantam architecture 
  user_main();
#else
  while ( true )
  {
    // Maintain state machines of all polled MPLAB Harmony modules
    SYS_Tasks ( );
  }
#endif
  // Execution should not come here during normal operation
  return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

