/**
 * FILE: main.s
 *
 * DESCRIPTION:
 * This file contains the assembly code for a STM32F401 SSD1306 driver utilizing the STM32F401CC6 microcontroller.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: March 3, 2024
 * UPDATE DATE: March 8, 2024
 *
 * ASSEMBLE AND LINK w/ SYMBOLS:
 * 1. arm-none-eabi-as -g main.s -o main.o
 * 2. arm-none-eabi-ld main.o -o main.elf -T STM32F401CCUX_FLASH.ld
 * 3. openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program main.elf verify reset exit"
 * ASSEMBLE AND LINK w/o SYMBOLS:
 * 1. arm-none-eabi-as -g main.s -o main.o
 * 2. arm-none-eabi-ld main.o -o main.elf -T STM32F401CCUX_FLASH.ld
 * 3. arm-none-eabi-objcopy -O binary --strip-all main.elf main.bin
 * 3. openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program main.bin 0x08000000 verify reset exit"
 * DEBUG w/ SYMBOLS:
 * 1. openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg
 * 2. arm-none-eabi-gdb main.elf
 * 3. target remote :3333
 * 4. monitor reset halt
 * 5. l
 * DEBUG w/o SYMBOLS:
 * 1. openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg
 * 2. arm-none-eabi-gdb main.bin
 * 3. target remote :3333
 * 4. monitor reset halt
 * 5. x/8i $pc
 */


.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb


/**
 * The start address for the .data section defined in linker script.
 */
.word _sdata

/**
 * The end address for the .data section defined in linker script.
 */
.word _edata

/**
 * The start address for the initialization values of the .data section
 * defined in linker script.
 */
.word _sidata

/**
 * The start address for the .bss section defined in linker script.
 */
.word _sbss

/**
 * The end address for the .bss section defined in linker script.
 */
.word _ebss


/**
 * Provide weak aliases for each Exception handler to the Default_Handler.
 * As they are weak aliases, any function with the same name will override
 * this definition.
 */
.macro weak name
  .global \name
  .weak \name
  .thumb_set \name, Default_Handler
  .word \name
.endm


/**
 * Initialize the .isr_vector section.
 * The .isr_vector section contains vector table.
 */
.section .isr_vector, "a"

/**
 * The STM32F401CCUx vector table.  Note that the proper constructs
 * must be placed on this to ensure that it ends up at physical address
 * 0x0000.0000.
 */
.global isr_vector
.type isr_vector, %object
isr_vector:
  .word _estack
  .word Reset_Handler
   weak NMI_Handler
   weak HardFault_Handler
   weak MemManage_Handler
   weak BusFault_Handler
   weak UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
   weak SVC_Handler
   weak DebugMon_Handler
  .word 0
   weak PendSV_Handler
   weak SysTick_Handler
  .word 0
   weak EXTI16_PVD_IRQHandler                              // EXTI Line 16 interrupt /PVD through EXTI line detection interrupt
   weak TAMP_STAMP_IRQHandler                              // Tamper and TimeStamp interrupts through the EXTI line
   weak EXTI22_RTC_WKUP_IRQHandler                         // EXTI Line 22 interrupt /RTC Wakeup interrupt through the EXTI line
   weak FLASH_IRQHandler                                   // FLASH global interrupt
   weak RCC_IRQHandler                                     // RCC global interrupt
   weak EXTI0_IRQHandler                                   // EXTI Line0 interrupt
   weak EXTI1_IRQHandler                                   // EXTI Line1 interrupt
   weak EXTI2_IRQHandler                                   // EXTI Line2 interrupt
   weak EXTI3_IRQHandler                                   // EXTI Line3 interrupt
   weak EXTI4_IRQHandler                                   // EXTI Line4 interrupt
   weak DMA1_Stream0_IRQHandler                            // DMA1 Stream0 global interrupt
   weak DMA1_Stream1_IRQHandler                            // DMA1 Stream1 global interrupt
   weak DMA1_Stream2_IRQHandler                            // DMA1 Stream2 global interrupt
   weak DMA1_Stream3_IRQHandler                            // DMA1 Stream3 global interrupt
   weak DMA1_Stream4_IRQHandler                            // DMA1 Stream4 global interrupt
   weak DMA1_Stream5_IRQHandler                            // DMA1 Stream5 global interrupt
   weak DMA1_Stream6_IRQHandler                            // DMA1 Stream6 global interrupt
   weak ADC_IRQHandler                                     // ADC1 global interrupt
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
   weak EXTI9_5_IRQHandler                                 // EXTI Line[9:5] interrupts
   weak TIM1_BRK_TIM9_IRQHandle                            // TIM1 Break interrupt and TIM9 global interrupt
   weak TIM1_UP_TIM10_IRQHandler                           // TIM1 Update interrupt and TIM10 global interrupt
   weak TIM1_TRG_COM_TIM11_IRQHandler                      // TIM1 Trigger and Commutation interrupts and TIM11 global interrupt
   weak TIM1_CC_IRQHandler                                 // TIM1 Capture Compare interrupt
   weak TIM2_IRQHandler                                    // TIM2 global interrupt
   weak TIM3_IRQHandler                                    // TIM3 global interrupt
   weak TIM4_IRQHandler                                    // TIM4 global interrupt
   weak I2C1_EV_IRQHandler                                 // I2C1 event interrupt
   weak I2C1_ER_IRQHandler                                 // I2C1 error interrupt
   weak I2C2_EV_IRQHandler                                 // I2C2 event interrupt
   weak I2C2_ER_IRQHandler                                 // I2C2 error interrupt
   weak SPI1_IRQHandler                                    // SPI1 global interrupt
   weak SPI2_IRQHandler                                    // SPI2 global interrupt
   weak USART1_IRQHandler                                  // USART1 global interrupt
   weak USART2_IRQHandler                                  // USART2 global interrupt
  .word 0                                                  // reserved
   weak EXTI15_10_IRQHandler                               // EXTI Line[15:10] interrupts
   weak EXTI17_RTC_Alarm_IRQHandler                        // EXTI Line 17 interrupt / RTC Alarms (A and B) through EXTI line interrupt
   weak EXTI18_OTG_FS_WKUP_IRQHandler                      // EXTI Line 18 interrupt / USBUSB OTG FS Wakeup through EXTI line interrupt
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
   weak DMA1_Stream7_IRQHandler                            // DMA1 Stream7 global interrupt
  .word 0                                                  // reserved
   weak SDIO_IRQHandler                                    // SDIO global interrupt
   weak TIM5_IRQHandler                                    // TIM5 global interrupt
   weak SPI3_IRQHandler                                    // SPI3 global interrupt
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
   weak DMA2_Stream0_IRQHandler                            // DMA2 Stream0 global interrupt
   weak DMA2_Stream1_IRQHandler                            // DMA2 Stream1 global interrupt
   weak DMA2_Stream2_IRQHandler                            // DMA2 Stream2 global interrupt
   weak DMA2_Stream3_IRQHandler                            // DMA2 Stream3 global interrupt
   weak DMA2_Stream4_IRQHandler                            // DMA2 Stream4 global interrupt
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
   weak OTG_FS_IRQHandler                                  // USB On The Go FS global interrupt
   weak DMA2_Stream5_IRQHandler                            // DMA2 Stream5 global interrupt
   weak DMA2_Stream6_IRQHandler                            // DMA2 Stream6 global interrupt
   weak DMA2_Stream7_IRQHandler                            // DMA2 Stream7 global interrupt
   weak USART6_IRQHandler                                  // USART6 global interrupt
   weak I2C3_EV_IRQHandler                                 // I2C3 event interrupt
   weak I2C3_ER_IRQHandler                                 // I2C3 error interrupt
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
   weak SPI4_IRQHandler                                    // SPI4 global interrupt

/**
 * @brief  This code is called when processor starts execution.
 *
 *         This is the code that gets called when the processor first
 *         starts execution following a reset event. We first define and init 
 *         the bss section and then define and init the data section, after which
 *         the application supplied main routine is called.
 *
 * @param  None
 * @retval None
 */
.type Reset_Handler, %function
.global Reset_Handler
Reset_Handler:
  LDR   R0, =_estack                                       // load address at end of the stack into R0
  MOV   SP, R0                                             // move address at end of stack into SP
  LDR   R0, =_sdata                                        // copy the data segment initializers from flash to SRAM
  LDR   R1, =_edata                                        // copy the data segment initializers from flash to SRAM
  LDR   R2, =_sidata                                       // copy the data segment initializers from flash to SRAM
  MOVS  R3, #0                                             // copy the data segment initializers from flash to SRAM
  B     .Reset_Handler_Loop_Copy_Data_Init                 // branch
.Reset_Handler_Copy_Data_Init:
  LDR   R4, [R2, R3]                                       // copy the data segment initializers into registers
  STR   R4, [R0, R3]                                       // copy the data segment initializers into registers
  ADDS  R3, R3, #4                                         // copy the data segment initializers into registers
.Reset_Handler_Loop_Copy_Data_Init:
  ADDS  R4, R0, R3                                         // initialize the data segment
  CMP   R4, R1                                             // initialize the data segment
  BCC   .Reset_Handler_Copy_Data_Init                      // branch if carry is clear
  LDR   R2, =_sbss                                         // copy the bss segment initializers from flash to SRAM
  LDR   R4, =_ebss                                         // copy the bss segment initializers from flash to SRAM
  MOVS  R3, #0                                             // copy the bss segment initializers from flash to SRAM
  B     .Reset_Handler_Loop_Fill_Zero_BSS                  // branch
.Reset_Handler_Fill_Zero_BSS:
  STR   R3, [R2]                                           // zero fill the bss segment
  ADDS  R2, R2, #4                                         // zero fill the bss segment
.Reset_Handler_Loop_Fill_Zero_BSS:
  CMP   R2, R4                                             // zero fill the bss segment
  BCC   .Reset_Handler_Fill_Zero_BSS                       // branch if carry is clear
  BL    main                                               // call function

/**
 * @brief  This code is called when the processor receives and unexpected interrupt.
 *
 *         This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval None
 */
.type Default_Handler, %function
.global Default_Handler
Default_Handler:
  BKPT                                                     // set processor into debug state
  B.N   Default_Handler                                    // call function, force thumb state


/**
 * Initialize the .text section.
 * The .text section contains executable code.
 */
.section .text

/**
 * @brief  Entry point for initialization and setup of specific functions.
 *
 *         This function is the entry point for initializing and setting up specific functions.
 *         It calls other functions to enable certain features and then enters a loop for further execution.
 *
 * @param  None
 * @retval None
 */
.type main, %function
.global main
main:
  PUSH  {R4-R11, LR}                                       // push registers R4-R11, LR to the stack
  BL    GPIOB_Enable                                       // call function
  BL    GPIOB_PB8_Alt_Function_Mode_Enable                 // call function
  BL    GPIOB_PB8_Open_Drain_Enable                        // call function
  BL    GPIOB_PB9_Alt_Function_Mode_Enable                 // call function
  BL    GPIOB_PB9_Open_Drain_Enable                        // call function
  BL    I2C1_Enable                                        // call function
  BL    I2C1_Init                                          // call function
  BL    SSD1306_Init                                       // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x11                                          // higher col start addr
  MOV   R2, #0xB0                                          // page start addr
  LDR   R3, =LETTER_H                                      // load the address of array H
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x12                                          // higher col start addr
  MOV   R2, #0xB0                                          // page start addr
  LDR   R3, =LETTER_E                                      // load the address of array E
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x13                                          // higher col start addr
  MOV   R2, #0xB0                                          // page start addr
  LDR   R3, =LETTER_L                                      // load the address of array L
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x14                                          // higher col start addr
  MOV   R2, #0xB0                                          // page start addr
  LDR   R3, =LETTER_L                                      // load the address of array L
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x15                                          // higher col start addr
  MOV   R2, #0xB0                                          // page start addr
  LDR   R3, =LETTER_O                                      // load the address of array O
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x11                                          // higher col start addr
  MOV   R2, #0xB2                                          // page start addr
  LDR   R3, =LETTER_W                                      // load the address of array O
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x12                                          // higher col start addr
  MOV   R2, #0xB2                                          // page start addr
  LDR   R3, =LETTER_O                                      // load the address of array O
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x13                                          // higher col start addr
  MOV   R2, #0xB2                                          // page start addr
  LDR   R3, =LETTER_R                                      // load the address of array O
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x14                                          // higher col start addr
  MOV   R2, #0xB2                                          // page start addr
  LDR   R3, =LETTER_L                                      // load the address of array O
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x15                                          // higher col start addr
  MOV   R2, #0xB2                                          // page start addr
  LDR   R3, =LETTER_D                                      // load the address of array D
  BL    SSD1306_Display_Letter                             // call function
  BL    SSD1306_Turn_On_Display                            // call function
  BL    Loop                                               // call function
  POP   {R4-R11, LR}                                       // pop registers R4-R11, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Enables the GPIOB peripheral by setting the corresponding RCC_AHB1ENR bit.
 *
 * @details This function enables the GPIOB peripheral by setting the corresponding
 *          RCC_AHB1ENR bit.  It loads the address of the RCC_AHB1ENR register, retrieves
 *          the current value of the register, sets the GPIOBEN bit, and stores the
 *          updated value back into the register.
 *
 * @param   None
 * @retval  None
 */
GPIOB_Enable:
  PUSH  {R4-R11, LR}                                       // push registers R4-R11, LR to the stack
  LDR   R4, =0x40023830                                    // load address of RCC_AHB1ENR register
  LDR   R5, [R4]                                           // load value inside RCC_AHB1ENR register
  ORR   R5, #(1<<1)                                        // set the GPIOBEN bit
  STR   R5, [R4]                                           // store value into RCC_AHB1ENR register
  POP   {R4-R11, LR}                                       // pop registers R4-R11, LR from the stack
  BX    LR                                                 // return to caller

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
  PUSH  {R4-R11, LR}                                       // push registers R4-R11, LR to the stack
  LDR   R4, =0x40020400                                    // load address of GPIOB_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOB_MODER register
  ORR   R5, #(1<<17)                                       // set the MODER8 bit
  BIC   R5, #(1<<16)                                       // clear the MODER8 bit
  STR   R5, [R4]                                           // store value into GPIOB_MODER register
  LDR   R4, =0x40020424                                    // load address of GPIOB_AFRH register
  LDR   R5, [R4]                                           // load value inside GPIOB_AFRH register
  BIC   R5, #(1<<3)                                        // clear the AFRH8 bit
  ORR   R5, #(1<<2)                                        // set the AFRH8 bit
  BIC   R5, #(1<<1)                                        // clear the AFRH8 bit
  BIC   R5, #(1<<0)                                        // clear the AFRH8 bit
  STR   R5, [R4]                                           // store value into GPIOB_AFRH register
  POP   {R4-R11, LR}                                       // pop registers R4-R11, LR from the stack
  BX    LR                                                 // return to caller

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
  PUSH  {R4-R11, LR}                                       // push registers R4-R11, LR to the stack
  LDR   R4, =0x40020404                                    // load address of GPIOB_OTYPER register
  LDR   R5, [R4]                                           // load value inside GPIOB_OTYPER register
  ORR   R5, #(1<<8)                                        // set the OT8 bit
  STR   R5, [R4]                                           // store value into GPIOB_OTYPER register
  POP   {R4-R11, LR}                                       // pop registers R4-R11, LR from the stack
  BX    LR                                                 // return to caller

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
  PUSH  {R4-R11, LR}                                       // push registers R4-R11, LR to the stack
  LDR   R4, =0x40020400                                    // load address of GPIOB_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOB_MODER register
  ORR   R5, #(1<<19)                                       // set the MODER9 bit
  BIC   R5, #(1<<18)                                       // clear the MODER9 bit
  STR   R5, [R4]                                           // store value into GPIOB_MODER register
  LDR   R4, =0x40020424                                    // load address of GPIOB_AFRH register
  LDR   R5, [R4]                                           // load value inside GPIOB_AFRH register
  BIC   R5, #(1<<7)                                        // clear the AFRH9 bit
  ORR   R5, #(1<<6)                                        // set the AFRH9 bit
  BIC   R5, #(1<<5)                                        // clear the AFRH9 bit
  BIC   R5, #(1<<4)                                        // clear the AFRH9 bit
  STR   R5, [R4]                                           // store value into GPIOB_AFRH register
  POP   {R4-R11, LR}                                       // pop registers R4-R11, LR from the stack
  BX    LR                                                 // return to caller

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
  PUSH  {R4-R11, LR}                                       // push registers R4-R11, LR to the stack
  LDR   R4, =0x40020404                                    // load address of GPIOB_OTYPER register
  LDR   R5, [R4]                                           // load value inside GPIOB_OTYPER register
  ORR   R5, #(1<<9)                                        // set the OT9 bit
  STR   R5, [R4]                                           // store value into GPIOB_OTYPER register
  POP   {R4-R11, LR}                                       // pop registers R4-R11, LR from the stack
  BX    LR                                                 // return to caller

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
  PUSH  {R4-R11, LR}                                       // push registers R4-R11, LR to the stack
  LDR   R4, =0x40023840                                    // load address of RCC_APB1ENR register
  LDR   R5, [R4]                                           // load value inside RCC_APB1ENR register
  ORR   R5, #(1<<21)                                       // set the I2C1EN bit
  STR   R5, [R4]                                           // store value into RCC_APB1ENR register
  POP   {R4-R11, LR}                                       // pop registers R4-R11, LR from the stack
  BX    LR                                                 // return to caller

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
  PUSH  {R4-R11, LR}                                       // push registers R4-R11, LR to the stack
  LDR   R4, =0x40005400                                    // load address of I2C1_CR1 register
  LDR   R5, [R4]                                           // load value inside I2C1_CR1 register
  ORR   R5, #(1<<15)                                       // set the SWRST bit
  STR   R5, [R4]                                           // store value into I2C1_CR1 register
  BIC   R5, #(1<<15)                                       // clear the SWRST bit
  STR   R5, [R4]                                           // store value into I2C1_CR1 register
  LDR   R4, =0x40005404                                    // load address of I2C1_CR2 register
  LDR   R5, [R4]                                           // load value inside I2C1_CR2 register
  ORR   R5, #(1<<5)                                        // set the FREQ bit
  ORR   R5, #(1<<4)                                        // set the FREQ bit
  BIC   R5, #(1<<3)                                        // clear the FREQ bit
  BIC   R5, #(1<<2)                                        // clear the FREQ bit
  ORR   R5, #(1<<1)                                        // set the FREQ bit
  BIC   R5, #(1<<0)                                        // clear the FREQ bit
  STR   R5, [R4]                                           // store value into I2C1_CR2 register
  LDR   R4, =0x4000541C                                    // load address of I2C1_CCR register
  LDR   R5, [R4]                                           // load value inside I2C1_CCR register
  ORR   R5, #(1<<15)                                       // set the F/S bit
  ORR   R5, #(1<<14)                                       // set the DUTY bit
  ORR   R5, #(1<<1)                                        // set the CCR bit
  STR   R5, [R4]                                           // store value into I2C1_CCR register
  LDR   R4, =0x40005420                                    // load address of I2C1_TRISE register
  LDR   R5, [R4]                                           // load value inside I2C1_TRISE register
  BIC   R5, #(1<<5)                                        // clear the TRISE bit
  ORR   R5, #(1<<4)                                        // set the TRISE bit
  BIC   R5, #(1<<3)                                        // clear the TRISE bit
  ORR   R5, #(1<<2)                                        // set the TRISE bit
  BIC   R5, #(1<<1)                                        // clear the TRISE bit
  BIC   R5, #(1<<0)                                        // clear the TRISE bit
  STR   R5, [R4]                                           // store value into I2C1_TRISE register
  LDR   R4, =0x40005400                                    // load address of I2C1_CR1 register
  LDR   R5, [R4]                                           // load value inside I2C1_CR1 register
  ORR   R5, #(1<<0)                                        // set the PE bit
  STR   R5, [R4]                                           // store value into I2C1_CR1 register
  POP   {R4-R11, LR}                                       // pop registers R4-R11, LR from the stack
  BX    LR                                                 // return to caller

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
  PUSH  {R4-R11, LR}                                       // push registers R4-R11, LR to the stack
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0x20                                          // set memory addressing mode, page addressing mode
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0xB0                                          // set page start address for page addressing mode (0-7 pages)
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0xA1                                          // set segment re-map, col addr 127 mapped to SEG0
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0xC8                                          // set COM output scan direction, remapped
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0x00                                          // set lower col start addr for page addr mode
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0x10                                          // set higher col start addr for page addr mode
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0xD5                                          // set display clock
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0xF0                                          // divide ratio/oscillator freq
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0x12                                          // set COM pins hardware config
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode 
  MOV   R2, #0xDB                                          // set VCOMH deselect level
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0x20                                          // 0.77 VCC
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0x8D                                          // charge pump setting
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0x14                                          // enable charge pump
  BL    I2C_Write_Byte                                     // call function
  BL    SSD1306_Clear_Screen                               // call function
  POP   {R4-R11, LR}                                       // pop registers R4-R11, LR from the stack
  BX    LR                                                 // return to caller

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
  PUSH  {R4-R11, LR}                                       // push registers R4-R11, LR to the stack
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0xAF                                          // set display on
  BL    I2C_Write_Byte                                     // call function
  POP   {R4-R11, LR}                                       // pop registers R4-R11, LR from the stack
  BX    LR                                                 // return to caller

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
  PUSH  {R4-R11, LR}                                       // push registers R4-R11, LR to the stack
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0xAE                                          // set display off
  BL    I2C_Write_Byte                                     // call function
  POP   {R4-R11, LR}                                       // pop registers R4-R11, LR from the stack
  BX    LR                                                 // return to caller

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
  PUSH  {R4-R11, LR}                                       // push registers R4-R11, LR to the stack
  MOV   R4, R0                                             // copy first arg into R4
  MOV   R5, R1                                             // copy second arg into R5
  MOV   R6, R2                                             // copy third arg into R6 
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, R4                                             // lower col start addr
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, R5                                             // higher col start addr
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // memory addr
  MOV   R2, R6                                             // page start addr
  BL    I2C_Write_Byte                                     // call function
  POP   {R4-R11, LR}                                       // pop registers R4-R11, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Clears the screen of the SSD1306 OLED Display.
 *
 * @details This assembly function clears the entire screen of the SSD1306 OLED display by sending
 *          the necessary commands and data through I2C communication. It utilizes the SSD1306_Set_Cursor
 *          function to position the cursor at the beginning of the display and then writes data to
 *          fill the screen with zeros.
 *
 * @param   None
 * @retval  None
 */
SSD1306_Clear_Screen:
  PUSH  {R4-R11, LR}                                       // push registers R4-R11, LR to the stack
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x10                                          // higher col start addr
  MOV   R2, #0xB0                                          // page start addr
  BL    SSD1306_Set_Cursor                                 // call function
  MOV   R11, #0x00                                         // set counter
.SSD1306_Clear_Screen_Loop:
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x40                                          // data mode
  MOV   R2, #0                                             // data
  BL    I2C_Write_Byte                                     // call function
  ADD   R11, #0x1                                          // increment counter
  CMP   R11, #0x480                                        // cmp if 0x480
  BNE   .SSD1306_Clear_Screen_Loop                         // branch not equal
  POP   {R4-R11, LR}                                       // pop registers R4-R11, LR from the stack
  BX    LR                                                 // return to caller

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
  PUSH  {R4-R11, LR}                                       // push registers R4-R11, LR to the stack
  MOV   R4, R0                                             // copy first arg into R4
  MOV   R5, R1                                             // copy second arg into R5
  MOV   R6, R2                                             // copy third arg into R6
  MOV   R7, R3                                             // copy fourth arg into R7
  BL    SSD1306_Set_Cursor                                 // call function
  MOV   R12, #0                                            // counter
.SSD1306_Display_Letter_Loop:
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x40                                          // data mode
  LDRB  R2, [R7, R12]                                      // load  byte at addr in R8 and inc by counter
  BL    I2C_Write_Byte                                     // call function
  ADDS  R12, #1                                            // inc counter
  CMP   R12, #12                                           // compare if end of array
  BNE   .SSD1306_Display_Letter_Loop                       // branch if not equal
  POP   {R4-R11, LR}                                       // pop registers R4-R11, LR from the stack
  BX    LR                                                 // return to caller

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
  PUSH  {R4-R11, LR}                                       // push registers R4-R11, LR to the stack
  MOV   R4, R0                                             // copy first arg into R4
  MOV   R5, R1                                             // copy second arg into R5
  MOV   R6, R2                                             // copy third arg into R6
.I2C_Wait_Not_Busy:
  LDR   R7, =0x40005418                                    // load address of I2C1_SR2 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR2 register
  TST   R8, #(1<<1)                                        // read the BUSY bit, if 0, then BNE
  BNE   .I2C_Wait_Not_Busy                                 // branch if not equal
  LDR   R7, =0x40005400                                    // load address of I2C1_CR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_CR1 register
  ORR   R8, #(1<<8)                                        // set the START bit
  ORR   R8, #(1<<0)                                        // set the PE bit
  STR   R8, [R7]                                           // store value into I2C1_CR1 register
.I2C_Wait_Start:
  LDR   R7, =0x40005414                                    // load address of I2C1_SR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR1 register
  TST   R8, #(1<<0)                                        // read the SB bit, if 1, then BEQ
  BEQ   .I2C_Wait_Start                                    // branch if equal
  LDR   R7, =0x40005410                                    // load address of I2C1_DR register
  LSL   R4, #1                                             // left shift to make room for the rw bit
  STR   R4, [R7]                                           // store value into I2C1_DR register
.I2C_Wait_Addr_Flag:
  LDR   R7, =0x40005414                                    // load address of I2C1_SR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR1 register
  LDR   R7, =0x40005414                                    // load address of I2C1_SR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR1 register
  TST   R8, #(1<<1)                                        // read the ADDR bit, if 1, then BEQ
  BEQ   .I2C_Wait_Addr_Flag                                // branch if equal
  LDR   R7, =0x40005418                                    // load address of I2C1_SR2 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR2 register
  STR   R8, [R7]                                           // store value into I2C1_SR2 register
.I2C_Wait_Data_Empty_Send_Mem_Addr:
  LDR   R7, =0x40005414                                    // load address of I2C1_SR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR1 register
  TST   R8, #(1<<7)                                        // read the TxE bit, if 0, then BEQ
  BEQ   .I2C_Wait_Data_Empty_Send_Mem_Addr                 // branch if equal
  LDR   R7, =0x40005410                                    // load address of I2C1_DR register
  STR   R5, [R7]                                           // store value into I2C1_DR register
.I2C_Wait_Data_Empty_Send_Data:
  LDR   R7, =0x40005414                                    // load address of I2C1_SR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR1 register
  TST   R8, #(1<<7)                                        // read the TxE bit, if 1, then BNE
  BEQ   .I2C_Wait_Data_Empty_Send_Data                     // branch if equal
.I2C_Send_Data:
  LDR   R7, =0x40005410                                    // load address of I2C1_DR register
  STR   R6, [R7]                                           // store value into I2C1_DR register
.I2C_Wait_Data_Transfer_Finished:
  LDR   R7, =0x40005414                                    // load address of I2C1_SR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR1 register
  TST   R8, #(1<<2)                                        // read the BTF bit, if 0, then BEQ
  BEQ   .I2C_Wait_Data_Transfer_Finished                   // branch if equal
  LDR   R7, =0x40005400                                    // load address of I2C1_CR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_CR1 register
  ORR   R8, #(1<<9)                                        // set the STOP bit
  ORR   R8, #(1<<0)                                        // set the PE bit
  STR   R8, [R7]                                           // store value into I2C1_CR1 register
  POP   {R4-R11, LR}                                       // pop registers R4-R11, LR from the stack
  BX    LR                                                 // return to caller

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
  B     .                                                  // branch infinite loop

/**
 * Initialize the .rodata section.
 * The .rodata section is used for constants and static strings.
 */
.section .rodata
LETTER_A:
  .byte 0xC0, 0x00, 0x38, 0x00, 0x26, 0x00, 0x38, 0x00, 0xC0, 0x00, 0x00, 0x00
LETTER_B:
  .byte 0x00, 0x00, 0xFE, 0x00, 0x92, 0x00, 0x92, 0x00, 0x92, 0x00, 0x6C, 0x00
LETTER_C:
  .byte 0x7C, 0x00, 0x82, 0x00, 0x82, 0x00, 0x82, 0x00, 0x64, 0x00, 0x00, 0x00
LETTER_D:
  .byte 0x00, 0x00, 0xFE, 0x00, 0x82, 0x00, 0x82, 0x00, 0x82, 0x00, 0x7C, 0x00
LETTER_E:
  .byte 0x00, 0x00, 0xFE, 0x00, 0x92, 0x00, 0x92, 0x00, 0x92, 0x00, 0x00, 0x00
LETTER_F:
  .byte 0x00, 0x00, 0xFE, 0x00, 0x12, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x00
LETTER_G:
  .byte 0x7C, 0x00, 0x82, 0x00, 0x82, 0x00, 0x92, 0x00, 0x74, 0x00, 0x00, 0x00
LETTER_H:
  .byte 0x00, 0x00, 0xFE, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0xFE, 0x00
LETTER_I:
  .byte 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
LETTER_J:
  .byte 0x40, 0x00, 0x80, 0x00, 0x80, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00
LETTER_K:
  .byte 0x00, 0x00, 0xFE, 0x00, 0x10, 0x00, 0x18, 0x00, 0x64, 0x00, 0x82, 0x00
LETTER_L:
  .byte 0x00, 0x00, 0xFE, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00, 0x00
LETTER_M:
  .byte 0x00, 0x00, 0xFE, 0x00, 0x18, 0x00, 0xF0, 0x00, 0x18, 0x00, 0xFE, 0x00
LETTER_N:
  .byte 0x00, 0x00, 0xFE, 0x00, 0x0C, 0x00, 0x38, 0x00, 0x60, 0x00, 0xFE, 0x00
LETTER_O:
  .byte 0x7C, 0x00, 0xC6, 0x00, 0x82, 0x00, 0x82, 0x00, 0xC2, 0x00, 0x7C, 0x00
LETTER_P:
  .byte 0x00, 0x00, 0xFE, 0x00, 0x12, 0x00, 0x12, 0x00, 0x12, 0x00, 0x0C, 0x00
LETTER_Q:
  .byte 0x7C, 0x00, 0xC6, 0x00, 0x82, 0x00, 0x82, 0x00, 0xC6, 0x00, 0xFC, 0x01
LETTER_R:
  .byte 0x00, 0x00, 0xFE, 0x00, 0x12, 0x00, 0x12, 0x00, 0x72, 0x00, 0x8E, 0x00
LETTER_S:
  .byte 0x00, 0x00, 0x6C, 0x00, 0xCA, 0x00, 0x92, 0x00, 0x92, 0x00, 0x74, 0x00
LETTER_T:
  .byte 0x02, 0x00, 0x02, 0x00, 0xFE, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00
LETTER_U:
  .byte 0x00, 0x00, 0x7E, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x7E, 0x00
LETTER_V:
  .byte 0x06, 0x00, 0x38, 0x00, 0xC0, 0x00, 0x38, 0x00, 0x06, 0x00, 0x00, 0x00
LETTER_W:
  .byte 0x7E, 0x00, 0xE0, 0x00, 0x30, 0x00, 0x3E, 0x00, 0x60, 0x00, 0xFE, 0x00
LETTER_X:
  .byte 0x82, 0x00, 0x6C, 0x00, 0x10, 0x00, 0x6C, 0x00, 0x82, 0x00, 0x00, 0x00
LETTER_Y:
  .byte 0x02, 0x00, 0x0C, 0x00, 0xF0, 0x00, 0x0C, 0x00, 0x02, 0x00, 0x00, 0x00
LETTER_Z:
  .byte 0xC2, 0x00, 0xA2, 0x00, 0x92, 0x00, 0x8A, 0x00, 0x86, 0x00, 0x00, 0x00
LETTER_SPACE:
  .byte 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

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
