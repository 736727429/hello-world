#ifndef UPDATAFLAG_H
#define UPDATAFLAG_H
#include "gd32e230.h"
#include "systick.h"
#include <stdio.h>
#include <stdbool.h>
#include "gd32e230c_eval.h"


#define FLASH_APP_ADDR 0x0800F000
#define FLASH_SECTOR_SIZE 0x400   //1k

void Flash_Erase_page(uint32_t page_Address);
void update_realize_test();
uint8_t read_updateflag();
void Flash_ProgramBytes(uint32_t Address,uint8_t *Buffer,uint16_t Bytecount);
void write_updateflag();
void Flash_Erase_page_test(uint32_t page_Address);

#endif