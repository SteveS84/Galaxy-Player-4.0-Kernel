/* ============================================================================================



   G D M     H O S T     A D A P T A T I O N     L A Y E R






   This file includes HAL(Host Adatation Layer) function prototypes for S/T-DMB chipset GDM700X.

   All the clients who use GCT's host API solution should make HAL functions in accordance with

   the function definition.







   GCT Semiconductor Inc. All Rights Reserved.



   ============================================================================================ */



/* ============================================================================================
   C O D E     H I S T O R Y
   ===============================================================================================

   -----------------------------------------------------------------------------------------------
   When              Who              What                        (in reverse chronological order)
   -----------------------------------------------------------------------------------------------

   Apr.19.2007       summerj           burst mode bug in gdm_hal_mm_busrt_read/write fixd
   Jan.19.2007       james Shin       delete memory access type in MM function
   Jan.15.2007	      Summerj          mm func moved to hal
   Jan.05.2007       James Shin       Created

   -------------------------------------------------------------------------------------------- */

#define GDM_API_FILE // definition indicating this file is API file

/* ============================================================================================
   I N C L U D E     F I L E S
   ============================================================================================ */

#include <linux/kernel.h>
#include <linux/string.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>

#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/stat.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/mach/map.h>
#include <linux/timer.h>
//#include <mach/irq.h>
#include <asm/mach/time.h>
#include <linux/dma-mapping.h>
#include "gdm_headers.h"
/*
#include "gdm_types.h"
#include "gdm_memory_map.h"
#include "gdm_hal.h"
 */


/* ============================================================================================
   D E F I N I T I O N S   and   V A R I A B L E S
   ============================================================================================ */
gdm_hal_func_s_type gdm_hal_func = 
{
	NULL, /* read_reg */
	NULL, /* burst_read_reg */
	NULL, /* write_reg */
	NULL, /* cr_read */
	NULL, /* cr_write */
	NULL, /* mm_read */
	NULL, /* mm_write */
	NULL, /* mm_burst_read */
	NULL , /* mm_burst_write */
	NULL   /* debug_msg */
};

static hal_endian_e_type gdm_hal_endian = HAL_BIG_ENDIAN;

struct spi_device *spi_dmb;

#define GDM_SPI_SINGLE_WRITE	0x80000000
#define GDM_SPI_SINGLE_READ		0x00000000
#define GDM_SPI_BURST_READ		0x40000000
#define GDM_SPI_BURST_WRITE 	0xC0000000
#define GDM_SPI_BURST_WRITE_S	0xC000
#define GDM_SPI_ADDR_OFFSET		23



/* ============================================================================================
   F U N C T I O N S
   ============================================================================================ */

/* --------------------------------------------------------------------------------------------

   FUNCTION    
   GDM_HAL_INIT_FUNC



   DESCRIPTION

   This function initialize Function Pointer. You should make function appropriately
   to the target system. Keep it in mind that all host API functions are based on this
   function.


   PARAMETER

   None.

   RETURN VALUE

   None

   -------------------------------------------------------------------------------------------- */
void gdm_hal_init_func(void)
{
	gdm_hal_func.read_reg       = &gdm_hal_read_reg;
	gdm_hal_func.burst_read_reg = &gdm_hal_burst_read_reg;
	gdm_hal_func.write_reg      = &gdm_hal_write_reg;
	gdm_hal_func.cr_read        = &gdm_hal_cr_read;
	gdm_hal_func.cr_write       = &gdm_hal_cr_write;
	gdm_hal_func.mm_read        = &gdm_hal_mm_read;
	gdm_hal_func.mm_write       = &gdm_hal_mm_write;
	gdm_hal_func.mm_burst_read  = &gdm_hal_mm_burst_read;
	gdm_hal_func.mm_burst_write = &gdm_hal_mm_burst_write;

	gdm_hal_func.debug_msg 			= NULL;
}


/* --------------------------------------------------------------------------------------------

   FUNCTION    
   GDM_HAL_GET_ENDIAN



   DESCRIPTION

   This function gets the information of host system endianness.



   PARAMETER

   None.



   RETURN VALUE

   The host system endianness.

   -------------------------------------------------------------------------------------------- */
hal_endian_e_type gdm_hal_get_endian(void)
{
	return gdm_hal_endian;
} /* gdm_hal_get_endian() */

/* --------------------------------------------------------------------------------------------

   FUNCTION    
   GDM_HAL_SET_ENDIAN



   DESCRIPTION

   This function sets host system endianness to big or little endian.



   PARAMETER

   [in] endian - the host system endianness.



   RETURN VALUE

   G_SUCCESS - Operation succeeded.
   G_ERROR - Operation failed.

   -------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_set_endian(hal_endian_e_type endian)
{
	gdm_hal_func.read_reg(0,0);// klaatu
	if(endian >= HAL_ENDIAN_MAX)
	{
		return G_ERROR;
	}

	gdm_hal_endian = endian;

	return G_SUCCESS;
} /* gdm_hal_set_endian() */

/* --------------------------------------------------------------------------------------------

   FUNCTION    
   GDM_HAL_READ_REG



   DESCRIPTION

   This function reads 16bit data of GDM700X GHOST register through host interface
   such as 6bit address - 16bit data parallel bus, I2C or SPI. You should make
   function appropriately to the target system. Keep it in mind that all host API
   functions are based on this function.



   PARAMETER

   [in] addr - the 6bit address of GDM700X GHOST register to be read
   [out] read_buf_ptr - the buffer for saving the 16bit data of GDM700X GHOST register




   RETURN VALUE

   G_SUCCESS - Operation succeeded.
   G_ERROR - Operation failed.

   -------------------------------------------------------------------------------------------- */

G_Int8 gdm_hal_read_reg(G_Uint8 addr, G_Uint16* read_buf_ptr)
{
	G_Uint32 ghost_addr, tx_data ;
	G_Uint8* uMRxBufAddr;

	struct spi_message              msg;
	struct spi_transfer             transfer[2];
	unsigned char                   status;
	//unsigned char *tx_tmp = kmalloc(2,GFP_KERNEL | GFP_DMA);//{0,}; //mkh
	unsigned char *tx_tmp = kmalloc(2,GFP_KERNEL );//{0,}; //mkh

	//uMRxBufAddr = kmalloc(2, GFP_KERNEL | GFP_DMA);
	uMRxBufAddr = kmalloc(2, GFP_KERNEL );

	ghost_addr = (((G_Uint32)(addr&0x3F))<<GDM_SPI_ADDR_OFFSET);
	tx_data = GDM_SPI_SINGLE_READ|ghost_addr;	
	//	printk("tx_data = [%x]\r\n", tx_data);		

	tx_tmp[0] = ((tx_data) >> 24) & 0xff;
	tx_tmp[1] = ((tx_data) >> 16) & 0xff;

	// printk("tx_tmp[] = [%x][%x]\r\n", tx_tmp[0], tx_tmp[1]);			

	memset( &msg, 0, sizeof( msg ) );
	memset( transfer, 0, sizeof( transfer ) );
	spi_message_init( &msg );

	msg.spi=spi_dmb;

	transfer[0].tx_buf = (u8 *) tx_tmp;
	transfer[0].rx_buf = NULL;
	transfer[0].len = 2;
	transfer[0].bits_per_word = 8;
	transfer[0].delay_usecs = 0;
	spi_message_add_tail( &(transfer[0]), &msg );

	transfer[1].tx_buf = NULL;
	transfer[1].rx_buf = (u8 *) uMRxBufAddr;
	transfer[1].len = (unsigned) 2;
	transfer[1].bits_per_word = 8;
	transfer[1].delay_usecs = 0;
	spi_message_add_tail( &(transfer[1]), &msg );

	/* Setting dma address for dma transfer */
	//msg.is_dma_mapped=1;
	//transfer[0].tx_dma = dma_map_single(&(spi_dmb->dev),tx_tmp,2,DMA_TO_DEVICE);  //mkh
	//transfer[1].rx_dma = dma_map_single(&(spi_dmb->dev),uMRxBufAddr,2,DMA_FROM_DEVICE); //mkh

	status = spi_sync(spi_dmb, &msg);

	//dma_unmap_single(&(spi_dmb->dev),(dma_addr_t)tx_tmp,2,DMA_TO_DEVICE); //mkh
	//dma_unmap_single(&(spi_dmb->dev),(dma_addr_t)uMRxBufAddr,2,DMA_FROM_DEVICE); //mkh

	// printk("status=%x, gdm_hal_read_reg rd_data = [%x][%x]\r\n", status, uMRxBufAddr[0], uMRxBufAddr[1]);

	*read_buf_ptr = (G_Int16)(uMRxBufAddr[0] << 8)|(G_Int16)uMRxBufAddr[1];

	kfree(tx_tmp);
	kfree(uMRxBufAddr);

	return G_SUCCESS;
}

/* --------------------------------------------------------------------------------------------

   FUNCTION    
   GDM_HAL_BURST_READ_REG



   DESCRIPTION

   This function reads array of 16bit data of GDM700X GHOST register through host interface
   such as 6bit address - array of 16bit data parallel bus, I2C or SPI. This function
   should be made appropriately to the target system. 


   PARAMETER

   [in] addr - the 6bit address of GDM700X GHOST register to be read
   [out] read_buf_ptr - the buffer for saving the array of 8bit data of GDM700X GHOST register
   [in] size - the size of data to be read in unit of 8bit byte.



   RETURN VALUE

   G_SUCCESS - Operation succeeded.
   G_ERROR - Operation failed.

   -------------------------------------------------------------------------------------------- */
#if 0
G_Int8 gdm_hal_burst_read_reg(G_Uint8 addr, G_Uint8* read_buf_ptr, G_Uint32 size)
{
	int i = 0;
	G_Uint32 ghost_addr, tx_data;
	G_Uint8* uMRxBufAddr;

	struct spi_message              msg;
	struct spi_transfer             transfer[2];
	unsigned char                   status;
	//unsigned char *tx_tmp = kmalloc(2,GFP_KERNEL | GFP_DMA);//{0,}; //mkh
	unsigned char *tx_tmp = kmalloc(2,GFP_KERNEL );//{0,}; //mkh

	//uMRxBufAddr = kmalloc(size, GFP_KERNEL | GFP_DMA);
	uMRxBufAddr = kmalloc(size + 32, GFP_KERNEL );

	ghost_addr = (((G_Uint32)(addr&0x3F))<<GDM_SPI_ADDR_OFFSET);
	tx_data = GDM_SPI_BURST_READ|ghost_addr;

	tx_tmp[0] = ((tx_data) >> 24) & 0xff;
	tx_tmp[1] = ((tx_data) >> 16) & 0xff;

	//	printk("tx_tmp[] = [%x][%x]\r\n", tx_tmp[0], tx_tmp[1]);			

	memset( &msg, 0, sizeof( msg ) );
	memset( transfer, 0, sizeof( transfer ) );
	spi_message_init( &msg );

	msg.spi=spi_dmb;

	transfer[0].tx_buf = (u8 *) tx_tmp;
	transfer[0].rx_buf = (u8 *) uMRxBufAddr;
	transfer[0].len = 2 + size;
	transfer[0].bits_per_word = 8;
	transfer[0].delay_usecs = 0;
	spi_message_add_tail( &(transfer[0]), &msg );

#if 0
	transfer[1].tx_buf = NULL;
	transfer[1].rx_buf = (u8 *) uMRxBufAddr;
	transfer[1].len = (unsigned) (size);
	transfer[1].bits_per_word = 8;
	transfer[1].delay_usecs = 0;
	spi_message_add_tail( &(transfer[1]), &msg );
#endif
	/* Setting dma address for dma transfer */
	//msg.is_dma_mapped=1;

	//transfer[0].tx_dma = dma_map_single(&(spi_dmb->dev),tx_tmp,2,DMA_TO_DEVICE);  //mkh
	//transfer[1].rx_dma = dma_map_single(&(spi_dmb->dev),uMRxBufAddr,size,DMA_FROM_DEVICE); //mkh
	status = spi_sync(spi_dmb, &msg);

	//dma_unmap_single(&(spi_dmb->dev),(dma_addr_t)tx_tmp,2,DMA_TO_DEVICE); //mkh
	//dma_unmap_single(&(spi_dmb->dev),(dma_addr_t)uMRxBufAddr,size,DMA_FROM_DEVICE); //mkh

	for(i=0; i<size/4 + 4; i++)
	{
		read_buf_ptr[i*4] = uMRxBufAddr[i*4 + 2];
		read_buf_ptr[i*4 + 1] = uMRxBufAddr[i*4+1 + 2];
		read_buf_ptr[i*4 + 2] = uMRxBufAddr[i*4+2 + 2];
		read_buf_ptr[i*4 + 3] = uMRxBufAddr[i*4+3 + 2];
	}

	kfree(tx_tmp);
	kfree(uMRxBufAddr);

	return G_SUCCESS;
}
#else
G_Int8 gdm_hal_burst_read_reg(G_Uint8 addr, G_Uint8* read_buf_ptr, G_Uint32 size)
{
	int i = 0;
	G_Uint32 ghost_addr, tx_data;

	struct spi_message              msg;
	struct spi_transfer             transfer[2];
	unsigned char                   status;
	//unsigned char tx_tmp[2];
	unsigned char *tx_tmp = kmalloc(2,GFP_KERNEL );//{0,}; //mkh

	unsigned char tmp;

	ghost_addr = (((G_Uint32)(addr&0x3F))<<GDM_SPI_ADDR_OFFSET);
	tx_data = GDM_SPI_BURST_READ|ghost_addr;

	tx_tmp[0] = ((tx_data) >> 24) & 0xff;
	tx_tmp[1] = ((tx_data) >> 16) & 0xff;

	// printk("tx_tmp[] = [%x][%x]\r\n", tx_tmp[0], tx_tmp[1]);   

	memset( &msg, 0, sizeof( msg ) );
	memset( transfer, 0, sizeof( transfer ) );
	spi_message_init( &msg );

	msg.spi=spi_dmb;

	transfer[0].tx_buf = (u8 *) tx_tmp;
	transfer[0].rx_buf = (u8 *) read_buf_ptr;
	transfer[0].len = 2 + size;
	transfer[0].bits_per_word = 8;
	transfer[0].delay_usecs = 0;
	spi_message_add_tail( &(transfer[0]), &msg );

	status = spi_sync(spi_dmb, &msg);

	kfree(tx_tmp);

	return G_SUCCESS;
}

#endif
/* --------------------------------------------------------------------------------------------

   FUNCTION    
   GDM_HAL_WRITE_REG



   DESCRIPTION

   This function writes 16bit data to GDM700X GHOST register through host interface
   such as 6bit address - 16bit data parallel bus, I2C or SPI. This function should
   be made appropriately to the target system. Keep it in mind that all host API
   functions are based on this function.



   PARAMETER

   [in] addr - the 6bit address of GDM700X GHOST register to be written
   [in] data - the 16bit data to be written



   RETURN VALUE

   G_SUCCESS - Operation succeeded.
   G_ERROR - Operation failed.

   -------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_write_reg(G_Uint8 addr, G_Uint16 data)
{
	G_Uint32 ghost_addr, tx_data ;

	struct spi_message              msg;
	struct spi_transfer             transfer;
	unsigned char                   status;	
	//unsigned char *tx_tmp = kmalloc(4, GFP_KERNEL | GFP_DMA);
	unsigned char *tx_tmp = kmalloc(4, GFP_KERNEL );

	ghost_addr = (((G_Uint32)(addr&0x3F))<<GDM_SPI_ADDR_OFFSET);
	tx_data = GDM_SPI_SINGLE_WRITE|ghost_addr|data;
	//	printk("tx_data = [%x]\r\n", tx_data);		

	tx_tmp[0] = ((tx_data) >> 24) & 0xff;
	tx_tmp[1] = ((tx_data) >> 16) & 0xff;
	tx_tmp[2] = ((tx_data) >> 8) & 0xff;
	tx_tmp[3] = ((tx_data) >> 0) & 0xff;

	// printk("tx_tmp[] = [%x][%x][%x][%x]\r\n", tx_tmp[0], tx_tmp[1], tx_tmp[2], tx_tmp[3]);		

	memset( &msg, 0, sizeof( msg ) );
	memset( &transfer, 0, sizeof( transfer ) );
	spi_message_init( &msg );

	msg.spi=spi_dmb;

	transfer.tx_buf = (u8 *) tx_tmp;
	transfer.rx_buf = NULL;
	transfer.len = 4;
	transfer.bits_per_word = 8;
	transfer.delay_usecs = 0;
	spi_message_add_tail( &(transfer), &msg );

	/* Setting dma address for dma transfer */
	//msg.is_dma_mapped=1;
	//transfer.tx_dma = dma_map_single(&(spi_dmb->dev),tx_tmp,4,DMA_TO_DEVICE);  //mkh

	status = spi_sync(spi_dmb, &msg);

	//dma_unmap_single(&(spi_dmb->dev),(dma_addr_t)tx_tmp,4,DMA_TO_DEVICE); //mkh

	// printk("status = [%x]\r\n", status);			

	return G_SUCCESS;
} /* gdm_hal_write_reg() */


/* --------------------------------------------------------------------------------------------

   FUNCTION    
   GDM_HAL_CR_READ



   DESCRIPTION

   This function reads the register addressed by GHOST_CR_ADDR register.



   PARAMETER

   [in] addr - the address of register to be read.
   [out] read_buf_ptr - the buffer for saving the 16bit data of GDM700X CR



   RETURN VALUE

   G_SUCCESS - Operation succeeded.
   G_ERROR - Operation failed.

   -------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_cr_read(G_Uint16 addr, G_Uint16* read_buf_ptr)
{
	G_Uint16 command = 0;

	G_Uint8 ret;

	if(read_buf_ptr == NULL)
	{
		return G_ERROR;
	}

	//printk("%s 1st",__FUNCTION__);
	/* check GHOST_CR_CMD[0] - 0:ready, 1:not ready */
	command = 0xFFFF;
	while(command & 0x0001)
	{
		if(gdm_hal_read_reg(GHOST_CR_CMD, &command) != G_SUCCESS)
		{
			return G_ERROR;
		}
	}

	//printk("%s 2nd",__FUNCTION__);
	/* write CR address */
	if(gdm_hal_write_reg(GHOST_CR_ADDR, addr) != G_SUCCESS)
	{
		return G_ERROR;
	}

	//printk("%s 3rd",__FUNCTION__);
	/* write command */
	if(gdm_hal_get_endian() == HAL_BIG_ENDIAN)
	{
		command = (GHOST_CR_CMD_ACTIVATE|GHOST_CR_CMD_READ|GHOST_CR_CMD_BIG_ENDIAN);
	}
	else if(gdm_hal_get_endian() == HAL_LITTLE_ENDIAN)
	{
		command = (GHOST_CR_CMD_ACTIVATE|GHOST_CR_CMD_READ|GHOST_CR_CMD_LITTLE_ENDIAN);
	}
	else
	{
		return G_ERROR;
	}

	//printk("%s 4th",__FUNCTION__);
	if(gdm_hal_write_reg(GHOST_CR_CMD, command) != G_SUCCESS)
	{
		return G_ERROR;
	}

	//printk("%s 5th",__FUNCTION__);
	/* check GHOST_CR_CMD[0] - 0:ready, 1:not ready */
	command = 0xFFFF;
	while(command & 0x0001)
	{
		if(gdm_hal_read_reg(GHOST_CR_CMD, &command) != G_SUCCESS)
		{
			return G_ERROR;
		}
	}

	//printk("%s 6th",__FUNCTION__);
	/* read GHOST_CR_DATA */
	ret = gdm_hal_read_reg(GHOST_CR_DATA, read_buf_ptr);  
	//printk(">>>>GDM<<<< %s, addr:%x , data:%x \n",__FUNCTION__, addr, *read_buf_ptr);
	return ret;
} /* gdm_hal_cr_read() */

/* --------------------------------------------------------------------------------------------

   FUNCTION    
   GDM_HAL_CR_WRITE



   DESCRIPTION

   This function writes the 16bit data to register addressed by GHOST_CR_ADDR register.



   PARAMETER

   [in] addr - the address of register to be written.
   [in] data - the data to be written.




   RETURN VALUE

   G_SUCCESS - Operation succeeded.
   G_ERROR - Operation failed.

   -------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_cr_write
(
 G_Uint16 addr,
 G_Uint16 data
 )
{
	G_Uint16 command = 0;

	//printk(">>>>GDM<<<< %s, addr:%x , data:%x \n",__FUNCTION__, addr, data);

	/* check GHOST_CR_CMD[0] - 0:ready, 1:not ready */
	command = 0xFFFF;
	while(command & 0x0001)
	{
		if(gdm_hal_read_reg(GHOST_CR_CMD, &command) != G_SUCCESS)
		{
			return G_ERROR;
		}
	}

	/* write CR address */
	if(gdm_hal_write_reg(GHOST_CR_ADDR, addr) != G_SUCCESS)
	{
		return G_ERROR;
	}

	/* write CR data */
	if(gdm_hal_write_reg(GHOST_CR_DATA, data) != G_SUCCESS)
	{
		return G_ERROR;
	}

	/* write command */
	if(gdm_hal_get_endian() == HAL_BIG_ENDIAN)
	{
		command = (GHOST_CR_CMD_ACTIVATE|GHOST_CR_CMD_WRITE|GHOST_CR_CMD_BIG_ENDIAN);
	}
	else if(gdm_hal_get_endian() == HAL_LITTLE_ENDIAN)
	{
		command = (GHOST_CR_CMD_ACTIVATE|GHOST_CR_CMD_WRITE|GHOST_CR_CMD_LITTLE_ENDIAN);
	}
	else
	{
		return G_ERROR;
	}

	return gdm_hal_write_reg(GHOST_CR_CMD, command);
} /* gdm_hal_cr_write() */

/* --------------------------------------------------------------------------------------------

   FUNCTION    
   GDM_HAL_MM_READ



   DESCRIPTION

   This function reads the internal memory addressed by GHOST_MM_ADDR_H 
   and GHOST_MM_ADDR_L register.



   PARAMETER

   [in] addr - the address of internal memory to be read.
   [out] read_buf_ptr - the buffer for saving the 32bit data of GDM700X internal memory



   RETURN VALUE

   G_SUCCESS - Operation succeeded.
   G_ERROR - Operation failed.

   -------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_mm_read
(
 G_Uint32 addr,
 G_Uint32* read_buf_ptr
 )
{
	G_Uint16 command = 0;
	G_Uint16 addr_h = (G_Int16)((addr & 0xFFFF0000) >> 16);
	G_Uint16 addr_l = (G_Int16)(addr & 0x0000FFFF);
	G_Uint16 data_h = 0;
	G_Uint16 data_l = 0;

	if(read_buf_ptr == NULL)
	{
		return G_ERROR;
	}

	/* check GHOST_MM_CMD[0] - 0:ready, 1:not ready */
	command = 0xFFFF;
	while(command & 0x0001)
	{
		if(gdm_hal_read_reg(GHOST_MM_CMD, &command) != G_SUCCESS)
		{
			return G_ERROR;
		}
	}

	/* write MM address */
	if(gdm_hal_write_reg(GHOST_MM_ADDR_H, addr_h) != G_SUCCESS)
	{
		return G_ERROR;
	}

	if(gdm_hal_write_reg(GHOST_MM_ADDR_L, addr_l) != G_SUCCESS)
	{
		return G_ERROR;
	}

	/* write command */
	if(gdm_hal_get_endian() == HAL_BIG_ENDIAN)
	{
		command = (GHOST_MM_CMD_ACTIVATE|GHOST_MM_CMD_READ|GHOST_MM_CMD_WORD|GHOST_MM_CMD_BIG_ENDIAN);
	}
	else if(gdm_hal_get_endian() == HAL_LITTLE_ENDIAN)
	{
		command = (GHOST_MM_CMD_ACTIVATE|GHOST_MM_CMD_READ|GHOST_MM_CMD_WORD|GHOST_MM_CMD_LITTLE_ENDIAN);
	}
	else
	{
		return G_ERROR;
	}

	if(gdm_hal_write_reg(GHOST_MM_CMD, command) != G_SUCCESS)
	{
		return G_ERROR;
	}

	/* check GHOST_MM_CMD[0] - 0:ready, 1:not ready */
	command = 0xFFFF;
	while(command & 0x0001)
	{
		if(gdm_hal_read_reg(GHOST_MM_CMD, &command) != G_SUCCESS)
		{
			return G_ERROR;
		}
	}

	/* read GHOST_MM_DATA */
	if(gdm_hal_read_reg(GHOST_MM_DATA_H, &data_h) != G_SUCCESS)
	{
		return G_ERROR;
	}

	if(gdm_hal_read_reg(GHOST_MM_DATA_L, &data_l) != G_SUCCESS)
	{
		return G_ERROR;
	}

	*read_buf_ptr = (G_Int32)(data_h << 16)|(G_Int32)data_l;

	return G_SUCCESS;
} /* gdm_hal_mm_read() */

/* --------------------------------------------------------------------------------------------

   FUNCTION    
   GDM_HAL_MM_WRITE



   DESCRIPTION

   This function writes the 32bit data to internal memory addressed by GHOST_MM_ADDR_H 
   and GHOST_MM_ADDR_L register.



   PARAMETER

   [in] addr - the address of internal memory to be written.
   [in] data - the 32bit data to be written.




   RETURN VALUE

   G_SUCCESS - Operation succeeded.
   G_ERROR - Operation failed.

   -------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_mm_write
(
 G_Uint32 addr,
 G_Uint32 data
 )
{
	G_Uint16 command = 0;
	G_Uint16 addr_h = (G_Uint16)((addr & 0xFFFF0000) >> 16);
	G_Uint16 addr_l = (G_Uint16)(addr & 0x0000FFFF);
	G_Uint16 data_h = (G_Uint16)((data & 0xFFFF0000) >> 16);
	G_Uint16 data_l = (G_Uint16)(data & 0x0000FFFF);

	/* check GHOST_MM_CMD[0] - 0:ready, 1:not ready */
	command = 0xFFFF;
	while(command & 0x0001)
	{
		if(gdm_hal_read_reg(GHOST_MM_CMD, &command) != G_SUCCESS)
		{
			return G_ERROR;
		}
	}

	/* write MM address */
	if(gdm_hal_write_reg(GHOST_MM_ADDR_H, addr_h) != G_SUCCESS)
	{
		return G_ERROR;
	}

	if(gdm_hal_write_reg(GHOST_MM_ADDR_L, addr_l) != G_SUCCESS)
	{
		return G_ERROR;
	}

	/* write MM data */
	if(gdm_hal_write_reg(GHOST_MM_DATA_H, data_h) != G_SUCCESS)
	{
		return G_ERROR;
	}

	if(gdm_hal_write_reg(GHOST_MM_DATA_L, data_l) != G_SUCCESS)
	{
		return G_ERROR;
	}

	/* write command */
	if(gdm_hal_get_endian() == HAL_BIG_ENDIAN)
	{
		command = (GHOST_MM_CMD_ACTIVATE|GHOST_MM_CMD_WRITE|GHOST_MM_CMD_WORD|GHOST_MM_CMD_BIG_ENDIAN);
	}
	else if(gdm_hal_get_endian() == HAL_LITTLE_ENDIAN)
	{
		command = (GHOST_MM_CMD_ACTIVATE|GHOST_MM_CMD_WRITE|GHOST_MM_CMD_WORD|GHOST_MM_CMD_LITTLE_ENDIAN);
	}
	else
	{
		return G_ERROR;
	}

	return gdm_hal_write_reg(GHOST_MM_CMD, command);
} /* gdm_hal_mm_write() */

/* --------------------------------------------------------------------------------------------

   FUNCTION    
   GDM_HAL_MM_BURST_READ



   DESCRIPTION

   This function executes the internal memory burst access read scheme.



   PARAMETER

   [in] addr - the address of internal memory to be read.
   [out] read_buf_ptr - the buffer for saving the 8bit data
   [in] size - the size of data to be read in unit of 8bit byte



   RETURN VALUE

   G_SUCCESS - Operation succeeded.
   G_ERROR - Operation failed.

   -------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_mm_burst_read
(
 G_Uint32 addr,
 G_Uint8* read_buf_ptr,
 G_Uint32 size
 )
{
	G_Uint16 command = 0;
	G_Uint16 addr_h = (G_Int16)((addr & 0xFFFF0000) >> 16);
	G_Uint16 addr_l = (G_Int16)(addr & 0x0000FFFF);
	G_Uint16 data;

	if(read_buf_ptr == NULL)
	{
		return G_ERROR;
	}

	if(size == 0)
	{
		return G_SUCCESS;
	}

	/* check GHOST_MM_CMD[0] - 0:ready, 1:not ready */
	command = 0xFFFF;
	while(command & 0x0001)
	{
		if(gdm_hal_read_reg(GHOST_MM_CMD, &command) != G_SUCCESS)
		{
			return G_ERROR;
		}
	}

	/* write MM address */
	if(gdm_hal_write_reg(GHOST_MM_ADDR_H, addr_h) != G_SUCCESS)
	{
		return G_ERROR;
	}

	if(gdm_hal_write_reg(GHOST_MM_ADDR_L, addr_l) != G_SUCCESS)
	{
		return G_ERROR;
	}

	/* write command : GHOST_MM_CMD_ACTIVATE shouldn't be set when burst mode used */
	if(gdm_hal_get_endian() == HAL_BIG_ENDIAN)
	{
		command = (GHOST_MM_CMD_READ|GHOST_MM_CMD_HALFWORD|GHOST_MM_CMD_BIG_ENDIAN|GHOST_MM_CMD_BURST);
	}
	else if(gdm_hal_get_endian() == HAL_LITTLE_ENDIAN)
	{
		command = (GHOST_MM_CMD_READ|GHOST_MM_CMD_HALFWORD|GHOST_MM_CMD_LITTLE_ENDIAN|GHOST_MM_CMD_BURST);
	}
	else
	{
		return G_ERROR;
	}

	if(gdm_hal_write_reg(GHOST_MM_CMD, command) != G_SUCCESS)
	{
		return G_ERROR;
	}

	/* check GHOST_MM_CMD[0] - 0:ready, 1:not ready */
	command = 0xFFFF;
	while(command & 0x0001)
	{
		if(gdm_hal_read_reg(GHOST_MM_CMD, &command) != G_SUCCESS)
		{
			return G_ERROR;
		}
	}

	while(size > 0)
	{
		/* read GHOST_MM_DATA */
		if(gdm_hal_read_reg(GHOST_MM_DATA_L, &data) != G_SUCCESS)			
		{
			return G_ERROR;
		}
		*read_buf_ptr	= (data &0xff00) >> 8, size--;
		if(size <= 0) break;

		*(read_buf_ptr+1)	= (data & 0x00ff), size--;
		read_buf_ptr	+= 2;
	}

	return G_SUCCESS;
} /* gdm_hal_mm_burst_read() */

/* --------------------------------------------------------------------------------------------

   FUNCTION    
   GDM_HAL_MM_BURST_WRITE



   DESCRIPTION

   This function executes the internal memory burst access write scheme.



   PARAMETER

   [in] addr - the address of internal memory to be read.
   [in] write_buf_ptr - the buffer of data to write.
   [in] size - the size of data to write in unit of 8bit byte.



   RETURN VALUE

   G_SUCCESS - Operation succeeded.
   G_ERROR - Operation failed.

   -------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_mm_burst_write
(
 G_Uint32 addr,
 G_Uint8* write_buf_ptr,
 G_Uint32 size
 )
{
	G_Uint16 command = 0;
	G_Uint16 addr_h = (G_Uint16)((addr & 0xFFFF0000) >> 16);
	G_Uint16 addr_l = (G_Uint16)(addr & 0x0000FFFF);
	G_Uint16 data;

	if(write_buf_ptr == NULL)
	{
		return G_ERROR;
	}

	if(size == 0)
	{
		return G_SUCCESS;
	}

	/* check GHOST_MM_CMD[0] - 0:ready, 1:not ready */
	command = 0xFFFF;
	while(command & 0x0001)
	{
		if(gdm_hal_read_reg(GHOST_MM_CMD, &command) != G_SUCCESS)
		{
			return G_ERROR;
		}
	}

	/* write MM address */
	if(gdm_hal_write_reg(GHOST_MM_ADDR_H, addr_h) != G_SUCCESS)
	{
		return G_ERROR;
	}

	if(gdm_hal_write_reg(GHOST_MM_ADDR_L, addr_l) != G_SUCCESS)
	{
		return G_ERROR;
	}

	/* write command : GHOST_MM_CMD_ACTIVATE shouldn't be set when burst mode used*/
	if(gdm_hal_get_endian() == HAL_BIG_ENDIAN)
	{
		command = (GHOST_MM_CMD_WRITE|GHOST_MM_CMD_HALFWORD|GHOST_MM_CMD_BIG_ENDIAN|GHOST_MM_CMD_BURST);
	}
	else if(gdm_hal_get_endian() == HAL_LITTLE_ENDIAN)
	{
		command = (GHOST_MM_CMD_WRITE|GHOST_MM_CMD_HALFWORD|GHOST_MM_CMD_LITTLE_ENDIAN|GHOST_MM_CMD_BURST);
	}
	else
	{
		return G_ERROR;
	}

	if(gdm_hal_write_reg(GHOST_MM_CMD, command) != G_SUCCESS)
	{
		return G_ERROR;
	}

	/* check GHOST_MM_CMD[0] - 0:ready, 1:not ready */
	command = 0xFFFF;
	while(command & 0x0001)
	{
		if(gdm_hal_read_reg(GHOST_MM_CMD, &command) != G_SUCCESS)
		{
			return G_ERROR;
		}
	}

	while(size > 0)
	{
		data = (*write_buf_ptr << 8);
		size--;

		if(size > 0)
		{
			data |= *(write_buf_ptr+1);
			size--;
		}

		/* write data */
		if(gdm_hal_write_reg(GHOST_MM_DATA_L, data) != G_SUCCESS)
		{
			return G_ERROR;
		}

		write_buf_ptr	+= 2;
	}

	return G_SUCCESS;
} /* gdm_hal_mm_burst_write() */


static int tdmbspi_probe(struct spi_device *spi)
{
	//	struct tdmb_spi *test;

	spi_dmb = spi;

	printk("tdmbspi_probe() \n");

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi_setup(spi);

	return 0;
}



static int __devexit tdmbspi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver tdmbspi_driver = {
	.driver = {
		.name	= "tdmbspi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= tdmbspi_probe,
	.remove		= __devexit_p(tdmbspi_remove),
};

int tdmbspi_init(void)
{
	printk("This is test program for S3C64XX's SPI Driver\n");

	return spi_register_driver(&tdmbspi_driver);
}

void tdmbspi_exit(void)
{
	spi_unregister_driver(&tdmbspi_driver);
}

void spi_open()
{
	printk("spi_open()!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
}

void spi_close()
{
	printk("spi_close()!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
}
