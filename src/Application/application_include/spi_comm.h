/*******************************************************************************
 * File:   spi_comm.h
 * Description:
 *
 ******************************************************************************/

#ifndef _SPI_COMM_H
#define	_SPI_COMM_H
 
#include "user_types.h"
#include "globdefs.h"

void spi_comm_init(void);
void spi_comm_task(void);
void pwr_brd_spi_comm(void);

 
#endif	/* _SPI_COMM_H */

