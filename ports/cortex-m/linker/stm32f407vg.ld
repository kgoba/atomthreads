/* Memory regions for STM32F407VG, 1024K flash, 128K RAM. */

MEMORY
{
    rom (rx)        : ORIGIN = 0x8000000, LENGTH = 1024K
    ram (xrw)       : ORIGIN = 0x20000000, LENGTH = 128K
    ccmram (rw)     : ORIGIN = 0x10000000, LENGTH = 64K
/*
    rom (rx) : ORIGIN = 0x08000000, LENGTH = 512K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
*/
}

/* Include main opencm3 linker script */
INCLUDE libopencm3_stm32f4.ld
