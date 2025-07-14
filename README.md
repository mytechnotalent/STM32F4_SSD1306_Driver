<img src="https://github.com/mytechnotalent/STM32F4_SSD1306_Driver/blob/main/STM32F4_SSD1306_Driver.png?raw=true">

## FREE Reverse Engineering Self-Study Course [HERE](https://github.com/mytechnotalent/Reverse-Engineering-Tutorial)

<br>

# STM32F4 SSD1306 Driver
An STM32F4, SSD1306 display driver written entirely in Assembler.

<br>

# Code
```assembler
/**
 * FILE: main.s
 *
 * DESCRIPTION:
 * This file contains the assembly code for a STM32F401 SSD1306 driver utilizing the
 * STM32F401CC6 microcontroller.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: March 3, 2024
 * UPDATE DATE: March 31, 2024
 */

.syntax unified                                       // use unified assembly syntax
.cpu cortex-m4                                        // target Cortex-M4 core
.fpu softvfp                                          // use software floating point
.thumb                                                // use Thumb instruction set

/**
 * The start address for the .data section defined in linker script.
 */
.word _sdata                                          // start of .data

/**
 * The end address for the .data section defined in linker script.
 */
.word _edata                                          // end of .data

/**
 * The start address for the initialization values of the .data section defined in
 * linker script.
 */
.word _sidata                                         // start of .data init values

/**
 * The start address for the .bss section defined in linker script.
 */
.word _sbss                                           // start of .bss

/**
 * The end address for the .bss section defined in linker script.
 */
.word _ebss                                           // end of .bss

/**
 * Provide weak aliases for each Exception handler to the Default_Handler. As they
 * are weak aliases, any function with the same name will override this definition.
 */
.macro weak name
  .global \name                                       // make symbol global
  .weak \name                                         // mark as weak
  .thumb_set \name, Default_Handler                   // set to Default_Handler
  .word \name                                         // vector entry
.endm

/**
 * Initialize the .isr_vector section. The .isr_vector section contains vector 
 * table.
 */
.section .isr_vector, "a"                             // vector table section

/**
 * The STM32F401RE vector table. Note that the proper constructs must be placed 
 * on this to ensure that it ends up at physical address 0x00000000.
 */
.global isr_vector                                    // export vector table
.type isr_vector, %object                             // object type
isr_vector:
  .word _estack                                       // Initial Stack Pointer
  .word Reset_Handler                                 // Reset Handler
   weak NMI_Handler                                   // NMI Handler
   weak HardFault_Handler                             // HardFault Handler
   weak MemManage_Handler                             // MemManage Handler
   weak BusFault_Handler                              // BusFault Handler
   weak UsageFault_Handler                            // UsageFault Handler
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak SVC_Handler                                   // SVC Handler
   weak DebugMon_Handler                              // Debug Monitor Handler
  .word 0                                             // Reserved
   weak PendSV_Handler                                // PendSV Handler
   weak SysTick_Handler                               // SysTick Handler
  .word 0                                             // Reserved
   weak EXTI16_PVD_IRQHandler                         // EXTI Line 16 Interrupt PVD
   weak TAMP_STAMP_IRQHandler                         // Tamper/TimeStamp Interrupt
   weak EXTI22_RTC_WKUP_IRQHandler                    // RTC Wakeup Interrupt
   weak FLASH_IRQHandler                              // FLASH Global Interrupt
   weak RCC_IRQHandler                                // RCC Global Interrupt
   weak EXTI0_IRQHandler                              // EXTI Line0 Interrupt
   weak EXTI1_IRQHandler                              // EXTI Line1 Interrupt
   weak EXTI2_IRQHandler                              // EXTI Line2 Interrupt
   weak EXTI3_IRQHandler                              // EXTI Line3 Interrupt
   weak EXTI4_IRQHandler                              // EXTI Line4 Interrupt
   weak DMA1_Stream0_IRQHandler                       // DMA1 Stream0 Global Interrupt
   weak DMA1_Stream1_IRQHandler                       // DMA1 Stream1 Global Interrupt
   weak DMA1_Stream2_IRQHandler                       // DMA1 Stream2 Global Interrupt
   weak DMA1_Stream3_IRQHandler                       // DMA1 Stream3 Global Interrupt
   weak DMA1_Stream4_IRQHandler                       // DMA1 Stream4 Global Interrupt
   weak DMA1_Stream5_IRQHandler                       // DMA1 Stream5 Global Interrupt
   weak DMA1_Stream6_IRQHandler                       // DMA1 Stream6 Global Interrupt
   weak ADC_IRQHandler                                // ADC1 Global Interrupt
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak EXTI9_5_IRQHandler                            // EXTI Line[9:5] Interrupts
   weak TIM1_BRK_TIM9_IRQHandle                       // TIM1 Break/TIM9 Global Interrupt
   weak TIM1_UP_TIM10_IRQHandler                      // TIM1 Update/TIM10 Global Interrupt
   weak TIM1_TRG_COM_TIM11_IRQHandler                 // TIM1 T/C/TIM11 Global Interrupt
   weak TIM1_CC_IRQHandler                            // TIM1 Capture Compare Interrupt
   weak TIM2_IRQHandler                               // TIM2 Global Interrupt
   weak TIM3_IRQHandler                               // TIM3 Global Interrupt
   weak TIM4_IRQHandler                               // TIM4 Global Interrupt
   weak I2C1_EV_IRQHandler                            // I2C1 Event Interrupt
   weak I2C1_ER_IRQHandler                            // I2C1 Error Interrupt
   weak I2C2_EV_IRQHandler                            // I2C2 Event Interrupt
   weak I2C2_ER_IRQHandler                            // I2C2 Error Interrupt
   weak SPI1_IRQHandler                               // SPI1 Global Interrupt
   weak SPI2_IRQHandler                               // SPI2 Global Interrupt
   weak USART1_IRQHandler                             // USART1 Global Interrupt
   weak USART2_IRQHandler                             // USART2 Global Interrupt
  .word 0                                             // Reserved
   weak EXTI15_10_IRQHandler                          // EXTI Line[15:10] Interrupts
   weak EXTI17_RTC_Alarm_IRQHandler                   // RTC Alarms EXTI
   weak EXTI18_OTG_FS_WKUP_IRQHandler                 // USB OTG FS Wakeup EXTI
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak DMA1_Stream7_IRQHandler                       // DMA1 Stream7 Global Interrupt
  .word 0                                             // Reserved
   weak SDIO_IRQHandler                               // SDIO Global Interrupt
   weak TIM5_IRQHandler                               // TIM5 Global Interrupt
   weak SPI3_IRQHandler                               // SPI3 Global Interrupt
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak DMA2_Stream0_IRQHandler                       // DMA2 Stream0 Global Interrupt
   weak DMA2_Stream1_IRQHandler                       // DMA2 Stream1 Global Interrupt
   weak DMA2_Stream2_IRQHandler                       // DMA2 Stream2 Global Interrupt
   weak DMA2_Stream3_IRQHandler                       // DMA2 Stream3 Global Interrupt
   weak DMA2_Stream4_IRQHandler                       // DMA2 Stream4 Global Interrupt
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak OTG_FS_IRQHandler                             // USB OTG FS Global Interrupt
   weak DMA2_Stream5_IRQHandler                       // DMA2 Stream5 Global Interrupt
   weak DMA2_Stream6_IRQHandler                       // DMA2 Stream6 Global Interrupt
   weak DMA2_Stream7_IRQHandler                       // DMA2 Stream7 Global Interrupt
   weak USART6_IRQHandler                             // USART6 Global Interrupt
   weak I2C3_EV_IRQHandler                            // I2C3 Event Interrupt
   weak I2C3_ER_IRQHandler                            // I2C3 Error Interrupt
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak SPI4_IRQHandler                               // SPI4 Global Interrupt

/**
 * @brief   This code is called when processor starts execution.
 *
 * @details This is the code that gets called when the processor first
 *          starts execution following a reset event. We first define and init 
 *          the bss section and then define and init the data section, after which
 *          the application supplied main routine is called.
 *
 * @param   None
 * @retval  None
 */
.type Reset_Handler, %function                        // function type
.global Reset_Handler                                 // export symbol
Reset_Handler:
.Reset_Handler_Setup:
  LDR   R4, =_estack                                  // load addr at end of stack R4
  MOV   SP, R4                                        // move addr at end of stack SP
  LDR   R4, =_sdata                                   // copy data seg init flash to SRAM
  LDR   R5, =_edata                                   // copy data seg init flash to SRAM
  LDR   R6, =_sidata                                  // copy data seg init flash to SRAM
  MOVS  R7, #0                                        // zero offset
  B     .Reset_Handler_Loop_Copy_Data_Init            // branch
.Reset_Handler_Copy_Data_Init:
  LDR   R8, [R6, R7]                                  // copy data seg init to regs
  STR   R8, [R4, R7]                                  // copy data seg init tp regs
  ADDS  R7, R7, #4                                    // increment offset
.Reset_Handler_Loop_Copy_Data_Init:
  ADDS  R8, R4, R7                                    // initialize the data segment
  CMP   R8, R5                                        // compare
  BCC   .Reset_Handler_Copy_Data_Init                 // branch if carry is clear
  LDR   R6, =_sbss                                    // copy bss seg init flash to SRAM
  LDR   R8, =_ebss                                    // copy bss seg init flash to SRAM
  MOVS  R7, #0                                        // zero offset
  B     .Reset_Handler_Loop_Fill_Zero_BSS             // branch
.Reset_Handler_Fill_Zero_BSS:
  STR   R7, [R6]                                      // zero fill the bss segment
  ADDS  R6, R6, #4                                    // increment pointer
.Reset_Handler_Loop_Fill_Zero_BSS:
  CMP   R6, R8                                        // compare
  BCC   .Reset_Handler_Fill_Zero_BSS                  // branch if carry is clear
.Reset_Handler_Call_Main:
  BL    main                                          // call main

/**
 * @brief   This code is called when the processor receives an unexpected interrupt.
 *
 * @details This simply enters an infinite loop, preserving the system state for  
 *          examination by a debugger.
 *
 * @param   None
 * @retval  None
 */
.type Default_Handler, %function                      // function type
.global Default_Handler                               // export symbol
Default_Handler:
  BKPT                                                // set processor into debug state
  B.N   Default_Handler                               // infinite loop

/**
 * Initialize the .text section. 
 * The .text section contains executable code.
 */
.section .text                                        // code section

/**
 * Initialize the .text section.
 * The .text section contains executable code.
 */
.section .text

/**
 * @brief  Entry point for initialization and setup of specific functions.
 *
 *         This function is the entry point for initializing and setting up specific functions.
 *         It calls other functions to enable certain features and then enters a loop for further
 *         execution.
 *
 * @param  None
 * @retval None
 */
.type main, %function
.global main
main:
.Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.GPIOC_Enable:
  BL    GPIOB_Enable                                  // call function
.GPIOB_PB8_Alt_Function_Mode_Enable:
  BL    GPIOB_PB8_Alt_Function_Mode_Enable            // call function
.GPIOB_PB8_Open_Drain_Enable:
  BL    GPIOB_PB8_Open_Drain_Enable                   // call function
.GPIOB_PB9_Alt_Function_Mode_Enable:
  BL    GPIOB_PB9_Alt_Function_Mode_Enable            // call function
.GPIOB_PB9_Open_Drain_Enable:
  BL    GPIOB_PB9_Open_Drain_Enable                   // call function
.I2C1_Enable:
  BL    I2C1_Enable                                   // call function
.I2C1_Init:
  BL    I2C1_Init                                     // call function
.SSD1306_Init:
  BL    SSD1306_Init                                  // call function
.SSD1306_Display_Letter_H_1:
  MOV   R0, #0x00                                     // lower col start addr
  MOV   R1, #0x11                                     // higher col start addr
  MOV   R2, #0xB0                                     // page start addr
  LDR   R3, =LETTER_H                                 // load the address of array H
  BL    SSD1306_Display_Letter                        // call function
.SSD1306_Display_Letter_E_1:
  MOV   R0, #0x00                                     // lower col start addr
  MOV   R1, #0x12                                     // higher col start addr
  MOV   R2, #0xB0                                     // page start addr
  LDR   R3, =LETTER_E                                 // load the address of array E
  BL    SSD1306_Display_Letter                        // call function
.SSD1306_Display_Letter_L_1:
  MOV   R0, #0x00                                     // lower col start addr
  MOV   R1, #0x13                                     // higher col start addr
  MOV   R2, #0xB0                                     // page start addr
  LDR   R3, =LETTER_L                                 // load the address of array L
  BL    SSD1306_Display_Letter                        // call function
.SSD1306_Display_Letter_L_2:
  MOV   R0, #0x00                                     // lower col start addr
  MOV   R1, #0x14                                     // higher col start addr
  MOV   R2, #0xB0                                     // page start addr
  LDR   R3, =LETTER_L                                 // load the address of array L
  BL    SSD1306_Display_Letter                        // call function
.SSD1306_Display_Letter_O_1:
  MOV   R0, #0x00                                     // lower col start addr
  MOV   R1, #0x15                                     // higher col start addr
  MOV   R2, #0xB0                                     // page start addr
  LDR   R3, =LETTER_O                                 // load the address of array O
  BL    SSD1306_Display_Letter                        // call function
.SSD1306_Display_Letter_W_1:
  MOV   R0, #0x00                                     // lower col start addr
  MOV   R1, #0x11                                     // higher col start addr
  MOV   R2, #0xB2                                     // page start addr
  LDR   R3, =LETTER_W                                 // load the address of array O
  BL    SSD1306_Display_Letter                        // call function
.SSD1306_Display_Letter_O_2:
  MOV   R0, #0x00                                     // lower col start addr
  MOV   R1, #0x12                                     // higher col start addr
  MOV   R2, #0xB2                                     // page start addr
  LDR   R3, =LETTER_O                                 // load the address of array O
  BL    SSD1306_Display_Letter                        // call function
.SSD1306_Display_Letter_R_1:
  MOV   R0, #0x00                                     // lower col start addr
  MOV   R1, #0x13                                     // higher col start addr
  MOV   R2, #0xB2                                     // page start addr
  LDR   R3, =LETTER_R                                 // load the address of array O
  BL    SSD1306_Display_Letter                        // call function
.SSD1306_Display_Letter_L_3:
  MOV   R0, #0x00                                     // lower col start addr
  MOV   R1, #0x14                                     // higher col start addr
  MOV   R2, #0xB2                                     // page start addr
  LDR   R3, =LETTER_L                                 // load the address of array O
  BL    SSD1306_Display_Letter                        // call function
.SSD1306_Display_Letter_D_1:
  MOV   R0, #0x00                                     // lower col start addr
  MOV   R1, #0x15                                     // higher col start addr
  MOV   R2, #0xB2                                     // page start addr
  LDR   R3, =LETTER_D                                 // load the address of array D
  BL    SSD1306_Display_Letter                        // call function
.SSD1306_Turn_On_Display:
  BL    SSD1306_Turn_On_Display                       // call function
.Loop:
  BL    Loop                                          // call function
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Enables the GPIOB peripheral by setting the corresponding RCC_AHB1ENR bit.
 *
 * @details This function enables the GPIOB peripheral by setting the corresponding RCC_AHB1ENR
 *          bit. It loads the address of the RCC_AHB1ENR register, retrieves the current value
 *          of the register, sets the GPIOBEN bit, and stores the updated value back into the
 *          register.
 *
 * @param   None
 * @retval  None
 */
GPIOB_Enable:
.GPIOB_Enable_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.GPIOB_Enable_Set_RCC_AHB1ENR:
  LDR   R4, =0x40023830                               // load address of RCC_AHB1ENR register
  LDR   R5, [R4]                                      // load value inside RCC_AHB1ENR register
  ORR   R5, #(1<<1)                                   // set the GPIOBEN bit
  STR   R5, [R4]                                      // store value into RCC_AHB1ENR register
.GPIOB_Enable_Pop_Registers
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Enables Alternative Function Mode on GPIOB Pin 8.
 *
 * @details This assembly function enables the Alternative Function Mode on GPIOB Pin 8
 *          by configuring the corresponding bits in the GPIOB_MODER and GPIOB_AFRH registers.
 *          It sets the pin to Alternative Function Mode and configures the specific alternative
 *          function for Pin 8.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB8_Alt_Function_Mode_Enable:
.GPIOB_PB8_Alt_Function_Mode_Enable_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.GPIOB_PB8_Alt_Function_Mode_Enable_Set_GPIOB_MODER:
  LDR   R4, =0x40020400                               // load address of GPIOB_MODER register
  LDR   R5, [R4]                                      // load value inside GPIOB_MODER register
  ORR   R5, #(1<<17)                                  // set the MODER8 bit
  BIC   R5, #(1<<16)                                  // clear the MODER8 bit
  STR   R5, [R4]                                      // store value into GPIOB_MODER register
.GPIOB_PB8_Alt_Function_Mode_Enable_Set_GPIOB_AFRH:
  LDR   R4, =0x40020424                               // load address of GPIOB_AFRH register
  LDR   R5, [R4]                                      // load value inside GPIOB_AFRH register
  BIC   R5, #(1<<3)                                   // clear the AFRH8 bit
  ORR   R5, #(1<<2)                                   // set the AFRH8 bit
  BIC   R5, #(1<<1)                                   // clear the AFRH8 bit
  BIC   R5, #(1<<0)                                   // clear the AFRH8 bit
  STR   R5, [R4]                                      // store value into GPIOB_AFRH register
.GPIOB_PB8_Alt_Function_Mode_Enable_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Enables Open Drain Mode on GPIOB Pin 8.
 *
 * @details This assembly function enables Open Drain Mode on GPIOB Pin 8
 *          by setting the corresponding bit in the GPIOB_OTYPER register.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB8_Open_Drain_Enable:
.GPIOB_PB8_Open_Drain_Enable_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.GPIOB_PB8_Open_Drain_Enable_GPIOB_OTYPER:
  LDR   R4, =0x40020404                               // load address of GPIOB_OTYPER register
  LDR   R5, [R4]                                      // load value inside GPIOB_OTYPER register
  ORR   R5, #(1<<8)                                   // set the OT8 bit
  STR   R5, [R4]                                      // store value into GPIOB_OTYPER register
.GPIOB_PB8_Open_Drain_Enable_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Enables Alternative Function Mode on GPIOB Pin 9.
 *
 * @details This assembly function enables the Alternative Function Mode on GPIOB Pin 9
 *          by configuring the corresponding bits in the GPIOB_MODER and GPIOB_AFRH registers.
 *          It sets the pin to Alternative Function Mode and configures the specific alternative
 *          function for Pin 9.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB9_Alt_Function_Mode_Enable:
.GPIOB_PB9_Alt_Function_Mode_Enable_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.GPIOB_PB9_Alt_Function_Mode_Enable_Set_MODER9:
  LDR   R4, =0x40020400                               // load address of GPIOB_MODER register
  LDR   R5, [R4]                                      // load value inside GPIOB_MODER register
  ORR   R5, #(1<<19)                                  // set the MODER9 bit
  BIC   R5, #(1<<18)                                  // clear the MODER9 bit
  STR   R5, [R4]                                      // store value into GPIOB_MODER register
.GPIOB_PB9_Alt_Function_Mode_Enable_Set_AFRH:
  LDR   R4, =0x40020424                               // load address of GPIOB_AFRH register
  LDR   R5, [R4]                                      // load value inside GPIOB_AFRH register
  BIC   R5, #(1<<7)                                   // clear the AFRH9 bit
  ORR   R5, #(1<<6)                                   // set the AFRH9 bit
  BIC   R5, #(1<<5)                                   // clear the AFRH9 bit
  BIC   R5, #(1<<4)                                   // clear the AFRH9 bit
  STR   R5, [R4]                                      // store value into GPIOB_AFRH register
.GPIOB_PB9_Alt_Function_Mode_Enable_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Enables Open Drain Mode on GPIOB Pin 9.
 *
 * @details This assembly function enables Open Drain Mode on GPIOB Pin 9
 *          by setting the corresponding bit in the GPIOB_OTYPER register.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB9_Open_Drain_Enable:
.GPIOB_PB9_Open_Drain_Enable_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.GPIOB_PB9_Open_Drain_Enable_Set_GPIOB_OTYPER:
  LDR   R4, =0x40020404                               // load address of GPIOB_OTYPER register
  LDR   R5, [R4]                                      // load value inside GPIOB_OTYPER register
  ORR   R5, #(1<<9)                                   // set the OT9 bit
  STR   R5, [R4]                                      // store value into GPIOB_OTYPER register
.GPIOB_PB9_Open_Drain_Enable_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Enables I2C1 Peripheral.
 *
 * @details This assembly function enables the I2C1 peripheral by setting the corresponding
 *          bit in the RCC_APB1ENR register. It loads the address of the RCC_APB1ENR register,
 *          retrieves the current value of the register, sets the I2C1EN bit, and stores the
 *          updated value back into the register.
 *
 * @param   None
 * @retval  None
 */
I2C1_Enable:
.I2C1_Enable_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.I2C1_Enable_Set_RCC_APB1ENR:
  LDR   R4, =0x40023840                               // load address of RCC_APB1ENR register
  LDR   R5, [R4]                                      // load value inside RCC_APB1ENR register
  ORR   R5, #(1<<21)                                  // set the I2C1EN bit
  STR   R5, [R4]                                      // store value into RCC_APB1ENR register
.I2C1_Enable_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Initializes and enables the I2C1 peripheral.
 *
 * @details This assembly function initializes and enables the I2C1 peripheral by configuring
 *          the relevant bits in the I2C1_CR1, I2C1_CR2, I2C1_CCR, and I2C1_TRISE registers.
 *          It performs a software reset, sets the frequency and duty cycle, configures the clock
 *          control register, sets the rise time, and finally, enables the I2C1 peripheral.
 *
 * @param   None
 * @retval  None
 */
I2C1_Init:
.I2C1_Init_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.I2C1_Init_Set_I2C1_CR1:
  LDR   R4, =0x40005400                               // load address of I2C1_CR1 register
  LDR   R5, [R4]                                      // load value inside I2C1_CR1 register
  ORR   R5, #(1<<15)                                  // set the SWRST bit
  STR   R5, [R4]                                      // store value into I2C1_CR1 register
  BIC   R5, #(1<<15)                                  // clear the SWRST bit
  STR   R5, [R4]                                      // store value into I2C1_CR1 register
.I2C1_Init_Set_I2C1_CR2:
  LDR   R4, =0x40005404                               // load address of I2C1_CR2 register
  LDR   R5, [R4]                                      // load value inside I2C1_CR2 register
  ORR   R5, #(1<<5)                                   // set the FREQ bit
  ORR   R5, #(1<<4)                                   // set the FREQ bit
  BIC   R5, #(1<<3)                                   // clear the FREQ bit
  BIC   R5, #(1<<2)                                   // clear the FREQ bit
  ORR   R5, #(1<<1)                                   // set the FREQ bit
  BIC   R5, #(1<<0)                                   // clear the FREQ bit
  STR   R5, [R4]                                      // store value into I2C1_CR2 register
.I2C1_Init_Set_I2C1_CCR:
  LDR   R4, =0x4000541C                               // load address of I2C1_CCR register
  LDR   R5, [R4]                                      // load value inside I2C1_CCR register
  ORR   R5, #(1<<15)                                  // set the F/S bit
  ORR   R5, #(1<<14)                                  // set the DUTY bit
  ORR   R5, #(1<<1)                                   // set the CCR bit
  STR   R5, [R4]                                      // store value into I2C1_CCR register
.I2C1_Init_Set_I2C1_TRISE:
  LDR   R4, =0x40005420                               // load address of I2C1_TRISE register
  LDR   R5, [R4]                                      // load value inside I2C1_TRISE register
  BIC   R5, #(1<<5)                                   // clear the TRISE bit
  ORR   R5, #(1<<4)                                   // set the TRISE bit
  BIC   R5, #(1<<3)                                   // clear the TRISE bit
  ORR   R5, #(1<<2)                                   // set the TRISE bit
  BIC   R5, #(1<<1)                                   // clear the TRISE bit
  BIC   R5, #(1<<0)                                   // clear the TRISE bit
  STR   R5, [R4]                                      // store value into I2C1_TRISE register
.I2C1_Init_Set_I2C1_CR1:
  LDR   R4, =0x40005400                               // load address of I2C1_CR1 register
  LDR   R5, [R4]                                      // load value inside I2C1_CR1 register
  ORR   R5, #(1<<0)                                   // set the PE bit
  STR   R5, [R4]                                      // store value into I2C1_CR1 register
.I2C1_Init_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Initializes the SSD1306 OLED Display.
 *
 * @details This assembly function initializes the SSD1306 OLED display by configuring
 *          the I2C communication parameters and sending the necessary commands to set
 *          up the display parameters.
 *
 * @param   None
 * @retval  None
 */
SSD1306_Init:
.SSD1306_Init_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.SSD1306_Init_Thirty_Microsecond_Delay_1:
  BL    Thirty_Microsecond_Delay                      // call function
.SSD1306_Init_Thirty_Microsecond_Delay_2:
  BL    Thirty_Microsecond_Delay                      // call function
.SSD1306_Init_I2C_Write_Byte_1:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, #0x20                                     // set memory addressing mode, page addressing mode
  BL    I2C_Write_Byte                                // call function
.SSD1306_Init_I2C_Write_Byte_2:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, #0xB0                                     // set page start address for page addressing mode (0-7 pages)
  BL    I2C_Write_Byte                                // call function
.SSD1306_Init_I2C_Write_Byte_3:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, #0xA1                                     // set segment re-map, col addr 127 mapped to SEG0
  BL    I2C_Write_Byte                                // call function
.SSD1306_Init_I2C_Write_Byte_4:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, #0xC8                                     // set COM output scan direction, remapped
  BL    I2C_Write_Byte                                // call function
.SSD1306_Init_I2C_Write_Byte_5:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, #0x00                                     // set lower col start addr for page addr mode
  BL    I2C_Write_Byte                                // call function
.SSD1306_Init_I2C_Write_Byte_6:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, #0x10                                     // set higher col start addr for page addr mode
  BL    I2C_Write_Byte                                // call function
.SSD1306_Init_I2C_Write_Byte_7:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, #0xD5                                     // set display clock
  BL    I2C_Write_Byte                                // call function
.SSD1306_Init_I2C_Write_Byte_8:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, #0xF0                                     // divide ratio/oscillator freq
  BL    I2C_Write_Byte                                // call function
.SSD1306_Init_I2C_Write_Byte_9:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, #0x12                                     // set COM pins hardware config
  BL    I2C_Write_Byte                                // call function
.SSD1306_Init_I2C_Write_Byte_a:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode 
  MOV   R2, #0xDB                                     // set VCOMH deselect level
.SSD1306_Init_I2C_Write_Byte_b:
  BL    I2C_Write_Byte                                // call function
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, #0x20                                     // 0.77 VCC
  BL    I2C_Write_Byte                                // call function
.SSD1306_Init_I2C_Write_Byte_c:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, #0x8D                                     // charge pump setting
  BL    I2C_Write_Byte                                // call function
.SSD1306_Init_I2C_Write_Byte_d:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, #0x14                                     // enable charge pump
  BL    I2C_Write_Byte                                // call function
  BL    SSD1306_Clear_Screen                          // call function
.SSD1306_Init_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Turns on the SSD1306 OLED Display.
 *
 * @details This assembly function turns on the SSD1306 OLED display by sending the necessary
 *          command through I2C communication to set the display panel to an active state.
 *
 * @param   None
 * @retval  None
 */
SSD1306_Turn_On_Display:
.SSD1306_Turn_On_Display_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.SSD1306_Turn_On_Display_I2C_Write_Byte_1:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, #0xAF                                     // set display on
  BL    I2C_Write_Byte                                // call function
.SSD1306_Turn_On_Display_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Turns off the SSD1306 OLED Display.
 *
 * @details This assembly function turns off the SSD1306 OLED display by sending the necessary
 *          command through I2C communication to set the display panel to an inactive state.
 *
 * @param   None
 * @retval  None
 */
SSD1306_Turn_Off_Display:
.SSD1306_Turn_Off_Display_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.SSD1306_Turn_Off_Display_I2C_Write_Byte_1:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, #0xAE                                     // set display off
  BL    I2C_Write_Byte                                // call function
.SSD1306_Turn_Off_Display_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Sets the cursor position on the SSD1306 OLED Display.
 *
 * @details This assembly function sets the cursor position on the SSD1306 OLED display by sending
 *          the necessary commands through I2C communication. It specifies the lower and higher
 *          column start addresses along with the page start address to define the cursor position.
 *
 * @param   R0: Lower column start address.
 * @param   R1: Higher column start address.
 * @param   R2: Page start address.
 *
 * @retval  None
 */
SSD1306_Set_Cursor:
.SSD1306_Set_Cursor_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.SSD1306_Set_Cursor_I2C_Write_Byte_1:
  MOV   R4, R0                                        // copy first arg into R4
  MOV   R5, R1                                        // copy second arg into R5
  MOV   R6, R2                                        // copy third arg into R6 
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, R4                                        // lower col start addr
  BL    I2C_Write_Byte                                // call function
.SSD1306_Set_Cursor_I2C_Write_Byte_2:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // command mode
  MOV   R2, R5                                        // higher col start addr
  BL    I2C_Write_Byte                                // call function
.SSD1306_Set_Cursor_I2C_Write_Byte_3:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x00                                     // memory addr
  MOV   R2, R6                                        // page start addr
  BL    I2C_Write_Byte                                // call function
.SSD1306_Set_Cursor_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Clears the screen of the SSD1306 OLED Display.
 *
 * @details This assembly function clears the entire screen of the SSD1306 OLED display by sending
 *          the necessary commands and data through I2C communication. It utilizes the
 *          SSD1306_Set_Cursor function to position the cursor at the beginning of the display
 *          and then writes data to fill the screen with zeros.
 *
 * @param   None
 * @retval  None
 */
SSD1306_Clear_Screen:
.SSD1306_Clear_Screen_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.SSD1306_Clear_Screen_SSD1306_Set_Cursor:
  MOV   R0, #0x00                                     // lower col start addr
  MOV   R1, #0x10                                     // higher col start addr
  MOV   R2, #0xB0                                     // page start addr
  BL    SSD1306_Set_Cursor                            // call function
.SSD1306_Clear_Screen_I2C_Write_Byte_1:
  MOV   R4, #0x00                                     // set counter
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x40                                     // data mode
  MOV   R2, #0                                        // data
  BL    I2C_Write_Byte                                // call function
.SSD1306_Clear_Screen_SSD1306_Clear_Screen_Loop:
  ADD   R4, #0x1                                      // increment counter
  CMP   R4, #0x480                                    // cmp if 0x480
  BNE   .SSD1306_Clear_Screen_Loop                    // branch not equal
.SSD1306_Clear_Screen_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Displays a letter (character array) on the SSD1306 OLED Display.
 *
 * @details This assembly function displays a letter (character array) on the SSD1306 OLED display
 *          by utilizing the SSD1306_Set_Cursor function to position the cursor and then writing the letter
 *          data to the display memory through I2C communication. The function also calls the
 *          SSD1306_Turn_On_Display function to ensure the display is active.
 *
 * @param   R0: Lower column start address.
 * @param   R1: Higher column start address.
 * @param   R2: Page start address.
 * @param   R3: Address of the character array to be displayed.
 *
 * @retval  None
 */
SSD1306_Display_Letter:
.SSD1306_Display_Letter_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.SSD1306_Display_Letter_SSD1306_Set_Cursor:
  MOV   R4, R0                                        // copy first arg into R4
  MOV   R5, R1                                        // copy second arg into R5
  MOV   R6, R2                                        // copy third arg into R6
  MOV   R7, R3                                        // copy fourth arg into R7
  BL    SSD1306_Set_Cursor                            // call function
.SSD1306_Display_Letter_Set_Counter:
  MOV   R8, #0                                        // set counter
.SSD1306_Display_Letter_Loop:
.SSD1306_Display_Letter_I2C_Write_Byte_1:
  MOV   R0, #0x3C                                     // SSD1306 I2C addr
  MOV   R1, #0x40                                     // data mode
  LDRB  R2, [R7, R8]                                  // load  byte at addr in R8 and inc by counter
  BL    I2C_Write_Byte                                // call function
.SSD1306_Display_Letter_Inc_Counter:
  ADDS  R8, #1                                        // inc counter
.SSD1306_Display_Letter_CMP_End_Of_Array:
  CMP   R8, #6                                        // compare if end of array
  BNE   .SSD1306_Display_Letter_Loop                  // branch if not equal
.SSD1306_Display_Letter_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief   Writes a byte to the I2C device.
 *
 * @details This assembly function writes a byte to the I2C device. It waits for the device
 *          to be ready and sends the data byte using I2C communication.
 *
 * @param   R0: I2C device address.
 * @param   R1: I2C data mode (0x00 for command, 0x40 for data).
 * @param   R2: Byte of data to be sent.
 *
 * @retval  None
 */
I2C_Write_Byte:
.I2C_Write_Byte_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
.I2C_Write_Byte_Copy_Params:
  MOV   R4, R0                                        // copy first arg into R4
  MOV   R5, R1                                        // copy second arg into R5
  MOV   R6, R2                                        // copy third arg into R6
.I2C_Wait_Not_Busy:
  LDR   R7, =0x40005418                               // load address of I2C1_SR2 register
  LDR   R8, [R7]                                      // load value inside I2C1_SR2 register
  TST   R8, #(1<<1)                                   // read the BUSY bit, if 0, then BNE
  BNE   .I2C_Wait_Not_Busy                            // branch if not equal
.I2C_Write_Byte_Set_I2C1_CR1:
  LDR   R7, =0x40005400                               // load address of I2C1_CR1 register
  LDR   R8, [R7]                                      // load value inside I2C1_CR1 register
  ORR   R8, #(1<<8)                                   // set the START bit
  ORR   R8, #(1<<0)                                   // set the PE bit
  STR   R8, [R7]                                      // store value into I2C1_CR1 register
.I2C_Wait_Start:
  LDR   R7, =0x40005414                               // load address of I2C1_SR1 register
  LDR   R8, [R7]                                      // load value inside I2C1_SR1 register
  TST   R8, #(1<<0)                                   // read the SB bit, if 1, then BEQ
  BEQ   .I2C_Wait_Start                               // branch if equal
  LDR   R7, =0x40005410                               // load address of I2C1_DR register
  LSL   R4, #1                                        // left shift to make room for the rw bit
  STR   R4, [R7]                                      // store value into I2C1_DR register
.I2C_Wait_Addr_Flag:
  LDR   R7, =0x40005414                               // load address of I2C1_SR1 register
  LDR   R8, [R7]                                      // load value inside I2C1_SR1 register
  LDR   R7, =0x40005414                               // load address of I2C1_SR1 register
  LDR   R8, [R7]                                      // load value inside I2C1_SR1 register
  TST   R8, #(1<<1)                                   // read the ADDR bit, if 1, then BEQ
  BEQ   .I2C_Wait_Addr_Flag                           // branch if equal
  LDR   R7, =0x40005418                               // load address of I2C1_SR2 register
  LDR   R8, [R7]                                      // load value inside I2C1_SR2 register
  STR   R8, [R7]                                      // store value into I2C1_SR2 register
.I2C_Wait_Data_Empty_Send_Mem_Addr:
  LDR   R7, =0x40005414                               // load address of I2C1_SR1 register
  LDR   R8, [R7]                                      // load value inside I2C1_SR1 register
  TST   R8, #(1<<7)                                   // read the TxE bit, if 0, then BEQ
  BEQ   .I2C_Wait_Data_Empty_Send_Mem_Addr            // branch if equal
  LDR   R7, =0x40005410                               // load address of I2C1_DR register
  STR   R5, [R7]                                      // store value into I2C1_DR register
.I2C_Wait_Data_Empty_Send_Data:
  LDR   R7, =0x40005414                               // load address of I2C1_SR1 register
  LDR   R8, [R7]                                      // load value inside I2C1_SR1 register
  TST   R8, #(1<<7)                                   // read the TxE bit, if 1, then BNE
  BEQ   .I2C_Wait_Data_Empty_Send_Data                // branch if equal
.I2C_Send_Data:
  LDR   R7, =0x40005410                               // load address of I2C1_DR register
  STR   R6, [R7]                                      // store value into I2C1_DR register
.I2C_Wait_Data_Transfer_Finished:
  LDR   R7, =0x40005414                               // load address of I2C1_SR1 register
  LDR   R8, [R7]                                      // load value inside I2C1_SR1 register
  TST   R8, #(1<<2)                                   // read the BTF bit, if 0, then BEQ
  BEQ   .I2C_Wait_Data_Transfer_Finished              // branch if equal
  LDR   R7, =0x40005400                               // load address of I2C1_CR1 register
  LDR   R8, [R7]                                      // load value inside I2C1_CR1 register
  ORR   R8, #(1<<9)                                   // set the STOP bit
  ORR   R8, #(1<<0)                                   // set the PE bit
  STR   R8, [R7]                                      // store value into I2C1_CR1 register
.I2C_Write_Byte_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief  Delay for approximately 30 microseconds.
 *
 *         This function creates a delay of approximately 30 microseconds.
 *
 * @param  None
 * @retval None
 */
Thirty_Microsecond_Delay:
.Thirty_Microsecond_Delay_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push registers R4-R12, LR to the stack
Thirty_Microsecond_Delay_Number_Of_Loops:
  MOV   R4, #7                                        // number of loops
.Thirty_Microsecond_Delay_Outer_Loop:
  MOV   R5, #0xFFFF                                   // set initial delay count
.Thirty_Microsecond_Delay_Inner_Loop:
  SUB   R5, #1                                        // decrement delay count
  CMP   R5, #0                                        // check if delay count reached zero
  BNE   .Thirty_Microsecond_Delay_Inner_Loop          // continue loop if delay count not reached zero
  SUB   R4, #1                                        // decrement loop counter
  CMP   R4, #0                                        // check if delay count reached zero
  BNE   .Thirty_Microsecond_Delay_Outer_Loop          // continue outer loop if more loops to go
  POP   {R4-R12, LR}                                  // pop registers R4-R12, LR from the stack
  BX    LR                                            // return to caller

/**
 * @brief  Infinite loop function.
 *
 *         This function implements an infinite loop using an unconditional branch (B) statement.
 *         It is designed to keep the program running indefinitely by branching back to itself.
 *
 * @param  None
 * @retval None
 */
Loop:
  B     .                                             // branch infinite loop


/**
 * Initialize the .rodata section.
 * The .rodata section is used for constants and static strings.
 */
.section .rodata

LETTER_A:
  .byte 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C, 0x41
LETTER_B:
  .byte 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x42
LETTER_C:
  .byte 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22, 0x43
LETTER_D:
  .byte 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x44
LETTER_E:
  .byte 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x45
LETTER_F:
  .byte 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01, 0x46
LETTER_G:
  .byte 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A, 0x47
LETTER_H:
  .byte 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x48
LETTER_I:
  .byte 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00, 0x49
LETTER_J:
  .byte 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01, 0x4A
LETTER_K:
  .byte 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x4B
LETTER_L:
  .byte 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40, 0x4C
LETTER_M:
  .byte 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x4D
LETTER_N:
  .byte 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x4E
LETTER_O:
  .byte 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x4F
LETTER_P:
  .byte 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06, 0x50
LETTER_Q:
  .byte 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x51
LETTER_R:
  .byte 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46, 0x52
LETTER_S:
  .byte 0x00, 0x46, 0x49, 0x49, 0x49, 0x31, 0x53
LETTER_T:
  .byte 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x54
LETTER_U:
  .byte 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x55
LETTER_V:
  .byte 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x56
LETTER_W:
  .byte 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x57
LETTER_X:
  .byte 0x00, 0x63, 0x14, 0x08, 0x14, 0x63, 0x58
LETTER_Y:
  .byte 0x00, 0x07, 0x08, 0x70, 0x08, 0x07, 0x59
LETTER_Z:
  .byte 0x00, 0x61, 0x51, 0x49, 0x45, 0x43, 0x5A
LETTER_NULL:
  .byte 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00


/**
 * Initialize the .data section.
 * The .data section is used for initialized global or static variables.
 */
.section .data


/**
 * Initialize the .bss section.
 * The .bss section is typically used for uninitialized global or static variables.
 */
.section .bss
```

<br>

## License
[MIT](https://raw.githubusercontent.com/mytechnotalent/STM32F4_SSD1306_Driver/main/LICENSE)
