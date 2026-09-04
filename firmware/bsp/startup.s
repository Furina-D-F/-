.syntax unified
.cpu cortex-m4
.thumb

.section .isr_vector, "a", %progbits
.global __isr_vector
.type __isr_vector, %object
__isr_vector:
    .word __StackTop
    .word Reset_Handler + 1
    .rept 9
    .word Default_Handler
    .endr
    .word vPortSVCHandler
    .word Default_Handler
    .word Default_Handler
    .word xPortPendSVHandler
    .word xPortSysTickHandler
.size __isr_vector, . - __isr_vector

.section .text.Reset_Handler, "ax", %progbits
.global Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
    ldr r0, =main
    bx r0
.size Reset_Handler, . - Reset_Handler

.section .text.Default_Handler, "ax", %progbits
.global Default_Handler
.type Default_Handler, %function
Default_Handler:
    b .
.size Default_Handler, . - Default_Handler