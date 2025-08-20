/*
 * assembly.s
 *
 */
 
 @ DO NOT EDIT
	.syntax unified
    .text
    .global ASM_Main
    .thumb_func

@ DO NOT EDIT
vectors:
	.word 0x20002000
	.word ASM_Main + 1

@ DO NOT EDIT label ASM_Main
ASM_Main:

	@ Some code is given below for you to start with
	LDR R0, RCC_BASE  		@ Enable clock for GPIOA and B by setting bit 17 and 18 in RCC_AHBENR
	LDR R1, [R0, #0x14]
	LDR R2, AHBENR_GPIOAB	@ AHBENR_GPIOAB is defined under LITERALS at the end of the code
	ORRS R1, R1, R2
	STR R1, [R0, #0x14]

	LDR R0, GPIOA_BASE		@ Enable pull-up resistors for pushbuttons
	MOVS R1, #0b01010101
	STR R1, [R0, #0x0C]
	LDR R1, GPIOB_BASE  	@ Set pins connected to LEDs to outputs
	LDR R2, MODER_OUTPUT
	STR R2, [R1, #0]
	MOVS R2, #0         	@ NOTE: R2 will be dedicated to holding the value on the LEDs

@ TODO: Add code, labels and logic for button checks and LED patterns


main_loop:

    @ First read the pushbutton inputs from GPIOA (IDR register at offset 0x10)
    LDR  R0, GPIOA_BASE
    LDR  R6, [R0, #0x10]       @ Save the current button states into R6

    @ SW2 (PA2): when pressed, force the LED pattern to 0xAA
    MOVS R7, #4                @ Create a mask to isolate bit 2 (PA2)
    TST  R6, R7                @ Test if SW2 is pressed (active low)
    BNE  check_sw3             @ If not pressed, skip to SW3 check
    MOVS R2, #0xAA             @ If pressed, load 0xAA into the LED register
    STR  R2, [R1, #0x14]       @ Update the LED output to show 0xAA
wait_sw2_release:
    LDR  R6, [R0, #0x10]       @ Keep checking the input state
    MOVS R7, #4                @ Mask again for SW2
    TST  R6, R7                @ Check if SW2 is still being pressed
    BEQ  wait_sw2_release      @ Stay in this loop until button is released
    B    main_loop             @ Once released, return to the start of main loop

    @ SW3 (PA3): when pressed, freeze the LEDs at their current state
check_sw3:
    MOVS R7, #8                @ Mask for bit 3 (PA3)
    TST  R6, R7                @ Check if SW3 is pressed
    BNE  check_sw0_sw1         @ If not pressed, continue to SW0/SW1 checks
freeze_loop:
    LDR  R6, [R0, #0x10]       @ Keep reading button inputs
    MOVS R7, #8                @ Mask for SW3
    TST  R6, R7                @ Test again
    BEQ  freeze_loop           @ Stay here until the button is released
    B    main_loop             @ When released, go back to main loop

    @ SW0 (PA0): change step size of increment
check_sw0_sw1:
    MOVS R7, #1                @ Mask for bit 0 (PA0)
    TST  R6, R7                @ Test if SW0 is pressed
    BEQ  sw0_pressed           @ If pressed, branch to handler
    MOVS R4, #1                @ If not pressed, step = 1
    B    after_sw0
sw0_pressed:
    MOVS R4, #2                @ If pressed, step = 2
after_sw0:

    @ SW1 (PA1): change timing delay
    MOVS R7, #2                @ Mask for bit 1 (PA1)
    TST  R6, R7                @ Test if SW1 is pressed
    BEQ  sw1_pressed           @ If pressed, use short delay
    LDR  R5, LONG_DELAY_CNT    @ If not pressed, use long delay (~0.7s)
    B    after_sw1
sw1_pressed:
    LDR  R5, SHORT_DELAY_CNT   @ If pressed, use short delay (~0.3s)
after_sw1:

    @ Delay loop: waste cycles to create time delay
    MOV   R0, R5               @ Copy delay count into R0
delay_loop:
    SUBS  R0, R0, #1           @ Subtract 1 each cycle
    BNE   delay_loop           @ Loop until R0 reaches zero

    @ Normal LED incrementing behaviour
    ADDS  R2, R2, R4           @ Increment LED value by step (R4 = 1 or 2)
    MOVS  R7, #0xFF            @ Prepare mask to keep only lower 8 bits
    ANDS  R2, R2, R7           @ Mask so LEDs wrap around after 255
    STR   R2, [R1, #0x14]      @ Write new value to LEDs
    B     main_loop            @ Repeat forever


write_leds:

	STR R2, [R1, #0x14]
	B main_loop

@ --------------------------
@ Subroutine: check_buttons
@ --------------------------

check_buttons:
	PUSH {R0-R7, LR}

	@ Reading the inputs
	LDR R0, GPIOA_BASE
	LDR R6, [R0, #0x10]    @ IDR register

	@ SW0: increment by 2
	MOVS R7, #1            @ mask = 0b0001
	ANDS R7, R6, R7        @ test SW0
	CMP R7, #0
	BEQ sw0_pressed
	MOVS R4, #1
	B check_sw1

check_sw1:
	@ SW1: short delay
	MOVS R7, #2            @ mask = 0b0010
	ANDS R7, R6, R7
	CMP R7, #0
	BEQ sw1_pressed
	LDR R5, LONG_DELAY_CNT
	B check_sw2

check_sw2:
	@ SW2: force 0xAA
	MOVS R7, #4            @ mask = 0b0100
	ANDS R7, R6, R7
	CMP R7, #0
	BEQ sw2_pressed
	B check_sw3
sw2_pressed:
	MOVS R2, #0xAA
	POP {R0-R7, PC}        @ Return early


sw3_pressed:
	B check_sw3            @ Stay here until released

done_buttons:
	POP {R0-R7, PC}



@ --------------------------
@ Subroutine: delay_loop
@ --------------------------

delay_loop_inner:
	SUBS R0, R0, #1
	BNE delay_loop_inner
	POP {R0-R2, PC}

@ LITERALS; DO NOT EDIT
	.align
RCC_BASE: 			.word 0x40021000
AHBENR_GPIOAB: 		.word 0b1100000000000000000
GPIOA_BASE:  		.word 0x48000000
GPIOB_BASE:  		.word 0x48000400
MODER_OUTPUT: 		.word 0x5555

@ TODO: Add your own values for these delays
@ LONG_DELAY_CNT: 	.word 0
@ SHORT_DELAY_CNT: 	.word 0

@ Counts tuned experimentally
LONG_DELAY_CNT: 	.word 1400000     @ Approximately ≈0.7s
SHORT_DELAY_CNT: 	.word 600000     @ Approximately ≈0.3s
