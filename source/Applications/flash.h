/*
F427的FLASH是2M的，总共有23个扇区，这里为了方便直接使用了AQ的代码，没有把11之后的扇区写出来
*/

#ifndef _flash_h
#define _flash_h

#include "aq.h"

#define FLASH_START_ADDR   ((uint32_t)0x080E0000)//存储config参数的起始地址，通常编绎完的程序是大小是到不了这个地址的，所以从这个地址起可以一直存储配置参数
#define FLASH_END_ADDR     ((uint32_t)0x080FFFFF)
#define FLASH_RETRIES      3

// Base address of the Flash sectors
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

extern int flashAddress(uint32_t startAddr, uint32_t *data, uint32_t len);
extern int flashErase(uint32_t startAddr, uint32_t len);
extern uint32_t flashStartAddr(void);
extern uint32_t flashSerno(uint8_t n);

#endif
