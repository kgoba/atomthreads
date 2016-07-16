/* Memory regions for STM32F303VC, 256K flash, 40K RAM. */

MEMORY
{
    rom (rx) : ORIGIN = 0x08000000, LENGTH = 256K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 40K
    ccmram (rwx) : ORIGIN = 0x10000000, LENGTH = 8K
}

/* Include main opencm3 linker script */
INCLUDE libopencm3_stm32f3.ld
