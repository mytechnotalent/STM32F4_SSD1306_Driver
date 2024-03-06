/**
 * FILE: stm32f401ccux.ld
 *
 * DESCRIPTION:
 * This file contains the linker script utilizing the STM32F401CC6 microcontroller.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: March 5, 2024
 * UPDATE Date: March 6, 2024
 */


/* The entry point. */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack. */
_estack = ORIGIN(RAM) + LENGTH(RAM);                       /* end of "RAM" RAM type memory */

/* The memory definitions. */
MEMORY
{
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 64K
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 256K
}

/* The sections. */
SECTIONS
{
  /* The startup code into "FLASH" ROM type memory. */
  .isr_vector :
  {
    *(.isr_vector)                                         /* startup code */
  } >FLASH

  /* The program code and other data into "FLASH" ROM type memory. */
  .text :
  {
    *(.text)                                               /* .text sections (code) */
    _etext = .;                                            /* define a global symbols at end of code */
  } >FLASH

  /* Constant data into "FLASH" Rom type memory. */
  .rodata :
  {
    *(.rodata)                                             /* .rodata sections (constants, strings, etc.) */
  } >FLASH

  /* Used by the startup to initialize data. */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM" RAM type memory. */
  .data :
  {
    _sdata = .;                                            /* create a global symbol at data start */
    *(.data)                                               /* .data sections */
    _edata = .;                                            /* define a global symbol at data end */
  } >RAM AT> FLASH

  /* Uninitialized data section into "RAM" RAM type memory. */
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section. */
    _sbss = .;                                             /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    _ebss = .;                                             /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

}
