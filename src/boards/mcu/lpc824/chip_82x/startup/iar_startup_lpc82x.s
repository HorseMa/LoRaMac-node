;/*****************************************************************************
; * @file:    startup_LPC8xx.s
; * @purpose: CMSIS Cortex-M0+ Core Device Startup File 
; *           for the NXP LPC8xx Device Series 
; * @version: V1.0
; * @date:    16. Aug. 2012
; *----------------------------------------------------------------------------
; *
; * Copyright (C) 2012 ARM Limited. All rights reserved.
; *
; * ARM Limited (ARM) is supplying this software for use with Cortex-M0+ 
; * processor based microcontrollers.  This file can be freely distributed 
; * within development tools that are supporting such ARM based processors. 
; *
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; *****************************************************************************/


;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)
	
        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA

__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler

        DCD     NMI_Handler
        DCD     HardFault_Handler
__vector_table_0x1c
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     SVC_Handler                 ; SVCall Handler
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     PendSV_Handler              ; PendSV Handler
        DCD     SysTick_Handler             ; SysTick Handler

        ; External Interrupts
        DCD     SPI0_IRQHandler             ; SPI0 controller
        DCD     SPI1_IRQHandler             ; SPI1 controller
        DCD     0                           ; Reserved               
        DCD     UART0_IRQHandler            ; UART0             
        DCD     UART1_IRQHandler            ; UART1            
        DCD     UART2_IRQHandler            ; UART2
        DCD     0                           ; Reserved
        DCD     I2C1_IRQHandler             ; I2C1 controller                         
        DCD     I2C0_IRQHandler             ; I2C0 controller                         
        DCD     SCT_IRQHandler              ; Smart Counter Timer                        
        DCD     MRT_IRQHandler              ; Multi-Rate Timer
        DCD     CMP_IRQHandler              ; Comparator
        DCD     WDT_IRQHandler              ; PIO1 (0:11)
        DCD     BOD_IRQHandler              ; Brown Out Detect                
        DCD     FLASH_IRQHandler            ; Non-Volatile Memory Controller               
        DCD     WKT_IRQHandler              ; Wakeup timer
        DCD     ADC_SEQA_IRQHandler         ; ADC Sequence A Completion [Only on LPC82X]
        DCD     ADC_SEQB_IRQHanlder         ; ADC Sequence B Completion [Only on LPC82X]
        DCD     ADC_THCMP_IRQHandler        ; ADC Threshold compare [Only on LPC82X]
        DCD     ADC_OVR_IRQHandler          ; ADC Overrun [Only on LPC82X]
        DCD     DMA_IRQHandler              ; DMA Controller [Only on LPC82X]
        DCD     I2C2_IRQHandler             ; I2C2 Controller [Only on LPC82X]
        DCD     I2C3_IRQHandler             ; I2C3 Controller [Only on LPC82X]
        DCD     0                           ; Reserved 
        DCD     PININT0_IRQHandler          ; PIO INT0
        DCD     PININT1_IRQHandler          ; PIO INT1
        DCD     PININT2_IRQHandler          ; PIO INT2
        DCD     PININT3_IRQHandler          ; PIO INT3
        DCD     PININT4_IRQHandler          ; PIO INT4
        DCD     PININT5_IRQHandler          ; PIO INT5
        DCD     PININT6_IRQHandler          ; PIO INT6
        DCD     PININT7_IRQHandler          ; PIO INT7
__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size 	EQU 	__Vectors_End - __Vectors


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B .

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
        B .

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B .

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B .

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B .

	PUBWEAK Reserved_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
Reserved_IRQHandler
        B .

        PUBWEAK	SPI0_IRQHandler
        PUBWEAK SPI1_IRQHandler
        PUBWEAK	UART0_IRQHandler
        PUBWEAK UART1_IRQHandler
        PUBWEAK	UART2_IRQHandler
        PUBWEAK	I2C0_IRQHandler
        PUBWEAK	I2C1_IRQHandler
        PUBWEAK SCT_IRQHandler      
        PUBWEAK MRT_IRQHandler      
        PUBWEAK CMP_IRQHandler      
        PUBWEAK WDT_IRQHandler      
        PUBWEAK BOD_IRQHandler      
        PUBWEAK FLASH_IRQHandler    
        PUBWEAK WKT_IRQHandler      
        PUBWEAK ADC_SEQA_IRQHandler 
        PUBWEAK ADC_SEQB_IRQHanlder 
        PUBWEAK ADC_THCMP_IRQHandler
        PUBWEAK ADC_OVR_IRQHandler  
        PUBWEAK DMA_IRQHandler      
        PUBWEAK	I2C2_IRQHandler
        PUBWEAK	I2C3_IRQHandler
        PUBWEAK	PININT0_IRQHandler
        PUBWEAK	PININT1_IRQHandler
        PUBWEAK	PININT2_IRQHandler
        PUBWEAK	PININT3_IRQHandler
        PUBWEAK	PININT4_IRQHandler
        PUBWEAK	PININT5_IRQHandler
        PUBWEAK	PININT6_IRQHandler
        PUBWEAK	PININT7_IRQHandler
 
SPI0_IRQHandler:
SPI1_IRQHandler:
UART0_IRQHandler:
UART1_IRQHandler:
UART2_IRQHandler:
I2C0_IRQHandler:
I2C1_IRQHandler:
SCT_IRQHandler:      
MRT_IRQHandler:      
CMP_IRQHandler:      
WDT_IRQHandler:      
BOD_IRQHandler:      
FLASH_IRQHandler:    
WKT_IRQHandler:      
ADC_SEQA_IRQHandler: 
ADC_SEQB_IRQHanlder: 
ADC_THCMP_IRQHandler:
ADC_OVR_IRQHandler:  
DMA_IRQHandler:      
I2C2_IRQHandler:
I2C3_IRQHandler:
SCT_IRQHandler:
MRT_IRQHandler:
CMP_IRQHandler:
WDT_IRQHandler:
BOD_IRQHandler:
WKT_IRQHandler:
PININT0_IRQHandler:
PININT1_IRQHandler:
PININT2_IRQHandler:
PININT3_IRQHandler:
PININT4_IRQHandler:
PININT5_IRQHandler:
PININT6_IRQHandler:
PININT7_IRQHandler:
Default_Handler:
        B .

;/************************************************************** 
;Code Read Protection feature
;------------------------------------------------------------
;CRP level     value        Comment
;------------------------------------------------------------
;CRP Disabled 0xFFFFFFFF (By default)
;NO ISP mode  0x4E697370
;   CRP1      0x12345678 
;   CRP2      0x87654321 
;   CRP3      0x43218765
;Caution: If CRP level is selected, JTAG access is disabled!
;The IAR linker script for the device will place the crp section
;at the appropriate offset
;***************************************************************/
        SECTION .crp:CODE:ROOT(2)

        DATA

        DCD	0xFFFFFFFF

        END
