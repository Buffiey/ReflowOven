; 76E003 ADC test program: Reads channel 7 on P1.1, pin 14
; This version uses an LED as voltage reference connected to pin 6 (P1.7/AIN0)

$NOLIST
$MODN76E003
$LIST

;  N76E003 pinout:
;                               -------
;       PWM2/IC6/T0/AIN4/P0.5 -|1    20|- P0.4/AIN5/STADC/PWM3/IC3
;               TXD/AIN3/P0.6 -|2    19|- P0.3/PWM5/IC5/AIN6
;               RXD/AIN2/P0.7 -|3    18|- P0.2/ICPCK/OCDCK/RXD_1/[SCL]
;                    RST/P2.0 -|4    17|- P0.1/PWM4/IC4/MISO
;        INT0/OSCIN/AIN1/P3.0 -|5    16|- P0.0/PWM3/IC3/MOSI/T1
;              INT1/AIN0/P1.7 -|6    15|- P1.0/PWM2/IC2/SPCLK
;                         GND -|7    14|- P1.1/PWM1/IC1/AIN7/CLO
;[SDA]/TXD_1/ICPDA/OCDDA/P1.6 -|8    13|- P1.2/PWM0/IC0
;                         VDD -|9    12|- P1.3/SCL/[STADC]
;            PWM5/IC7/SS/P1.5 -|10   11|- P1.4/SDA/FB/PWM1
;                               -------
;


BAUD              EQU 115200 ; Baud rate of UART in bps
TIMER1_RELOAD     EQU (0x100-(CLK/(16*BAUD)))
TIMER0_RELOAD_1MS EQU (0x10000-(CLK/1000))
CLK           EQU 16600000 ; Microcontroller system frequency in Hz
TIMER0_RATE   EQU 4096     ; 2048Hz squarewave (peak amplitude of CEM-1203 speaker)
TIMER0_RELOAD EQU ((65536-(CLK/TIMER0_RATE)))
TIMER2_RATE   EQU 1000     ; 1000Hz, for a timer tick of 1ms
TIMER2_RELOAD EQU ((65536-(CLK/TIMER2_RATE)))


BsTime equ P1.3
BsTemp equ P0.0
BrTime equ P0.1
BRTemp equ P0.2
B01 equ P0.3
; Reset vector
org 0x0000
    ljmp main

org 0x0003
	reti

; Timer/Counter 0 overflow interrupt vector
org 0x000B
	ljmp Timer0_ISR

; External interrupt 1 vector (not used in this code)
org 0x0013
	reti

; Timer/Counter 1 overflow interrupt vector (not used in this code)
org 0x001B
	reti


; Serial port receive/transmit interrupt vector (not used in this code)
org 0x0023 
	reti
org 0x002B
	ljmp Timer2_ISR

title:     db 'RC:     RT:', 0
title2:    db 'SC:     ST:', 0

cseg
; These 'equ' must match the hardware wiring
LCD_RS equ P1.3
LCD_E  equ P1.4
LCD_D4 equ P0.0
LCD_D5 equ P0.1
LCD_D6 equ P0.2
LCD_D7 equ P0.3
SOUND_OUT equ P1.2
PWM_OUT equ P1.0
$NOLIST
$include(LCD_4bit.inc) ; A library of LCD related functions and utility macros
$LIST

; These register definitions needed by 'math32.inc'
DSEG at 30H
x:   ds 4
y:   ds 4
bcd: ds 5
VLED_ADC: ds 2
sec: ds 3
Count1ms:     ds 2 ; Used to determine when half second has passed
FSM1_state: ds 1
temp_soak: ds 2
time_soak: ds 2
temp_refl: ds 2
time_refl: ds 2
refltime: ds 2
refltemp: ds 2
soaktime: ds 2
soaktemp: ds 2
shutoff: ds 2

seconds: 	  ds 1
pwm: ds 3
pwm_counter:  ds 1


temp: ds 4
VAL_LM4040: ds 2
VAL_LM335: ds 2

BSEG
beep_enable: dbit 1
mf: dbit 1
PB0: dbit 1
PB1: dbit 1
PB2: dbit 1
PB3: dbit 1
PB4: dbit 1
PB5: dbit 1
PB6: dbit 1
PB7: dbit 1

s_flag: 		   dbit 1
half_seconds_flag: dbit 1
flag1: 			   dbit 1
flag2:			   dbit 1
flag0:			   dbit 1


$NOLIST
$include(math32.inc)
$LIST





;                        1234567890123456    
rt:    db ' RT ',0  ;Reflow time,Reflow Temp, Soak Time
rc:    db 'RC ',0
st:    db ' ST ',0
sc:    db 'SC ',0
;ti:    db 'Ti',0
To:    db 'T0',0
state1messagetop:    db 's1     RAMP2SOAK', 0
state1messagebottom: db 'tt   ts    t    ', 0
state2messagetop:    db 's2          SOAK', 0
state3messagetop:    db 's3     RAMP2PEAK', 0
state4messagetop:    db 's4        REFLOW', 0
state5messagetop:    db 's5     cooldown ', 0
errorshutoffmessage: db '****SHUT OFF****', 0
CSEG
;---------------------------------;
; Routine to initialize the ISR   ;
; for timer 0                     ;
;---------------------------------;
Timer0_Init:
	orl CKCON, #0b00001000 ; Input for timer 0 is sysclk/1
	mov a, TMOD
	anl a, #0xf0 ; 11110000 Clear the bits for timer 0
	orl a, #0x01 ; 00000001 Configure timer 0 as 16-timer
	mov TMOD, a
	mov TL0, #High(TIMER0_RELOAD)
	mov TH0, #low(TIMER0_RELOAD)
	; Enable the timer and interrupts
    setb ET0  ; Enable timer 0 interrupt
    setb TR0  ; Start timer 0
	ret


; ==============================
; Timer0 ISR for Buzzer Control
; ==============================
Timer0_ISR:
    ;clr TR0                 ; Stop the timer to prevent timing issues
   ; mov TH0, #high(TIMER0_RELOAD)
   ; mov TL0, #low(TIMER0_RELOAD)
   ; setb TR0                ; Restart the timer
beep_check:
    jb beep_enable, toggle_sound  ; If beep_enable is set, toggle sound
    ;clr SOUND_OUT           ; Otherwise, ensure it's off
    reti

toggle_sound:
	clr TR0                 ; Stop the timer to prevent timing issues
    mov TH0, #high(TIMER0_RELOAD)
    mov TL0, #low(TIMER0_RELOAD)
    setb TR0  
    cpl SOUND_OUT           ; Toggle buzzer state
    reti

; ==============================
; Start Beep Function
; ==============================
Start_Beep:
 setb beep_enable        ; Enable beep flag              ; Start Timer 0 if not already running
 ljmp beep_check

; ==============================
; Stop Beep Function
; ==============================
Stop_Beep:
    clr beep_enable         ; Disable beep flag
    clr SOUND_OUT           ; Ensure buzzer is off
    clr TR0                 ; Stop Timer 0 to save power
  reti
  
;---------------------------------;
; Routine to initialize the ISR   ;
; for timer 2                     ;
;---------------------------------;
Timer2_Init:
	mov T2CON, #0 ; Stop timer/counter.  Autoreload mode.
	mov TH2, #high(TIMER2_RELOAD)
	mov TL2, #low(TIMER2_RELOAD)
	; Set the reload value
	orl T2MOD, #0x80 ; Enable timer 2 autoreload
	mov RCMP2H, #high(TIMER2_RELOAD)
	mov RCMP2L, #low(TIMER2_RELOAD)
	; Init One millisecond interrupt counter.  It is a 16-bit variable made with two 8-bit parts
	clr a
	mov Count1ms+0, a
	mov Count1ms+1, a
	; Enable the timer and interrupts
	mov pwm_counter, #0

	orl EIE, #0x80 ; Enable timer 2 interrupt ET2=1
    setb TR2  ; Enable timer 2
	ret

;---------------------------------;
; ISR for timer 2                 ;
;---------------------------------;
Timer2_ISR:
	clr TF2  ; Timer 2 doesn't clear TF2 automatically. Do it in the ISR.  It is bit addressable.
	cpl P0.4 ; To check the interrupt rate with oscilloscope. It must be precisely a 1 ms pulse.
	
	; The two registers used in the ISR must be saved in the stack
	push acc
	push psw

Skip_Count:
	inc pwm_counter
	clr c 
	mov a, pwm
	subb a, pwm_counter
	cpl c 
	mov PWM_OUT, c  

	mov a, pwm_counter
	cjne a, #100, Timer2_ISR_continue
	mov pwm_counter, #0
	inc seconds
	setb s_flag

Timer2_ISR_continue:
	inc Count1ms+0    ; Increment the low 8-bits first
	mov a, Count1ms+0 ; If the low 8-bits overflow, then increment high 8-bits
	jnz Inc_Done
	inc Count1ms+1
	ljmp Inc_done
	
go_to_Timer2_ISR_done:
	ljmp Timer2_ISR_done
Inc_Done:
	; Check if half second has passed
	mov a, Count1ms+0
	cjne a, #low(1000), go_to_Timer2_ISR_done ; Warning: this instruction changes the carry flag!
	mov a, Count1ms+1
	cjne a, #high(1000), go_to_Timer2_ISR_done
	
	; 500 milliseconds have passed.  Set a flag so the main program knows
	setb half_seconds_flag ; Let the main program know half second had passed
	cpl TR0 ; Enable/disable timer/counter 0. This line creates a beep-silence-beep-silence sound.
	; Reset to zero the milli-seconds counter, it is a 16-bit variable
	clr a
	mov Count1ms+0, a
	mov Count1ms+1, a ;this block of code is for getting it to stop at 0000
    mov a, shutoff+1 ;loads most significant digit into a
    cjne a, #0x00, test1 ;checks if a which has most significant digit is 0, if not 0 regular action, if 0 then check least significant digit
	clr a ;clears a
	mov a, shutoff+0 ;loads least sigficant digit into a
	cjne a, #0x00, test0 ;check if least significant digit is 0, if not 0 regular action, if 0 then should should bcd counter should be 0
	mov shutoff+0, a ;load the 0 into a.
	ljmp test0
	
check0: ;this block of code is for getting it to stop at 0000
    mov a, shutoff+1 ;loads most significant digit into a
    cjne a, #0x00, test0 ;checks if a which has most significant digit is 0, if not 0 regular action, if 0 then check least significant digit
	clr a ;clears a
	mov a, shutoff+0 ;loads least sigficant digit into a
	cjne a, #0x00, test0 ;check if least significant digit is 0, if not 0 regular action, if 0 then should should bcd counter should be 0
	mov shutoff+0, a ;load the 0 into a.
	ljmp test1
test0:
	jnb flag0, test1
	clr a
	mov a, shutoff+0
	add a, #0x99 ;subtracts 
	cjne a, #0x99, Timer2_ISR_da0
	add a, #0x01 ; if i dont add this it starts at 98, instead of 99
	mov shutoff+0, a
	clr a
	mov a, shutoff+1
	add a, #0x99 ;subtracts from the most significant digit
	da a
	mov shutoff+1, a
	;clr a 
	ljmp check0

Timer2_ISR_da0:
	da a ; Decimal adjust instruction.  Check datasheet for more details!
	mov shutoff+0, a
	; Increment the BCD counter


check1: ;this block of code is for getting it to stop at 0000
    mov a, time_soak+1 ;loads most significant digit into a
    cjne a, #0x00, test1 ;checks if a which has most significant digit is 0, if not 0 regular action, if 0 then check least significant digit
	clr a ;clears a
	mov a, time_soak+0 ;loads least sigficant digit into a
	cjne a, #0x00, test1 ;check if least significant digit is 0, if not 0 regular action, if 0 then should should bcd counter should be 0
	mov time_soak+0, a ;load the 0 into a.
	ljmp test2

test1:
	jnb flag1, test2
	clr a
	mov a, time_soak+0
	add a, #0x99 ;subtracts 
	cjne a, #0x99, Timer2_ISR_da
	add a, #0x01; if i dont add this it starts at 98, instead of 99
	mov time_soak+0, a
	clr a
	mov a, time_soak+1
	add a, #0x99 ;subtracts from the most significant digit
	da a
	mov time_soak+1, a
	;clr a 
	ljmp check1

Timer2_ISR_da:
	da a ; Decimal adjust instruction.  Check datasheet for more details!
	mov time_soak+0, a

check2: ;this block of code is for getting it to stop at 0000
    mov a, time_refl+1 ;loads most significant digit into a
    cjne a, #0x00, test2 ;checks if a which has most significant digit is 0, if not 0 regular action, if 0 then check least significant digit
	clr a ;clears a
	mov a, time_refl+0 ;loads least sigficant digit into a
	cjne a, #0x00, test2 ;check if least significant digit is 0, if not 0 regular action, if 0 then should should bcd counter should be 0
	mov time_refl+0, a ;load the 0 into a.
	ljmp check2

test2:
	jnb flag2, Timer2_ISR_done
	clr a
	mov a, time_refl+0
	add a, #0x99 ;subtracts 
	cjne a, #0x99, Timer2_ISR_da2
	add a, #0x01; if i dont add this it starts at 98, instead of 99
	mov time_refl+0, a
	clr a
	mov a, time_refl+1
	add a, #0x99 ;subtracts from the most significant digit
	da a
	mov time_refl+1, a
	;clr a 
	ljmp check2

Timer2_ISR_da2:
	da a ; Decimal adjust instruction.  Check datasheet for more details!
	mov time_refl+0, a
	
Timer2_ISR_done:
	pop psw
	pop acc
	reti

Init_All:
	; Configure all the pins for biderectional I/O
	mov	P3M1, #0x00
	mov	P3M2, #0x00
	mov	P1M1, #0x00
	mov	P1M2, #0x00
	mov	P0M1, #0x00
	mov	P0M2, #0x00
	
	orl	CKCON, #0x10 ; CLK is the input for timer 1
	orl	PCON, #0x80 ; Bit SMOD=1, double baud rate
	mov	SCON, #0x52
	anl	T3CON, #0b11011111
	anl	TMOD, #0x0F ; Clear the configuration bits for timer 1
	orl	TMOD, #0x20 ; Timer 1 Mode 2
	mov	TH1, #TIMER1_RELOAD ; TH1=TIMER1_RELOAD;
	setb TR1
	
	; Using timer 0 for delay functions.  Initialize here:
	clr	TR0 ; Stop timer 0
	orl	CKCON,#0x08 ; CLK is the input for timer 0
	anl	TMOD,#0xF0 ; Clear the configuration bits for timer 0
	orl	TMOD,#0x01 ; Timer 0 in Mode 1: 16-bit timer
	
	; Initialize and start the ADC:
	
	; Initialize (P0.4, P0.5) as input.
	orl P0M1, #0b00110000
    anl P0M2, #0b11001111
    ; Initialize the pins used by the ADC (P1.1, P1.7) as input.
	orl	P1M1, #0b10000010
	anl	P1M2, #0b01111101
	  

	
	; AINDIDS select if some pins are analog inputs or digital I/O:
	mov AINDIDS, #0x00 ; Disable all analog inputs
	orl AINDIDS, #0b10110001 ; Using AIN0, AIN4, AIN5, AIN7
	orl ADCCON1, #0x01 ; Enable ADC
	setb EA   ; Enable global interrupts
	setb ET0  ; Enable Timer 0 interrupt
	setb TR0  ; Ensure Timer 0 is running
	
	

	ret
load_temp:
	jnb TI, load_temp ; wait for SBUF to be ready
	clr TI			  ; clr TI so we know SBUF is getting our new value
	mov SBUF, a		  ; move new value into SBUF
	ret

send_temp:
	mov a, bcd+3
	swap a			; upper digit of bcd+3
	anl a, #0x0F    ; Mask the lower nibble (idk what this means)
	add a, #48      ; convert to ASCI
	lcall load_temp ; load the value
    mov SBUF, a     ; send value
    
    mov a, bcd+3	; lower digit of bcd+3
    anl a, #0x0F
    add a, #48
    lcall load_temp
    mov SBUF, a
    
	mov a, bcd+2
	swap a			; upper digit of bcd+2
	anl a, #0x0F    
	add a, #48      
	lcall load_temp 
    mov SBUF, a     
    
    mov a, bcd+2	; lower digit of bcd+2
    anl a, #0x0F
    add a, #48
    lcall load_temp
    mov SBUF, a
    
    mov a, #'.'		; decimal point
    lcall load_temp
    mov SBUF, a
    
    mov a, bcd+1	; upper digit of bcd+1
	swap a
	anl a, #0x0F 
	add a, #48 
	lcall load_temp
    mov SBUF, a 
    
    mov a, bcd+1	; lower digit of bcd+1
    anl a, #0x0F
    add a, #48
    lcall load_temp
    mov SBUF, a
    
    mov a, bcd+0	; upper digit of bcd+0
	swap a
	anl a, #0x0F 
	add a, #48 
	lcall load_temp
    mov SBUF, a 
    
    mov a, bcd+0	; lower digit of bcd+0
    anl a, #0x0F
    add a, #48
    lcall load_temp
    mov SBUF, a
    
	mov a, #'\r'	; cursor to far left
    lcall load_temp
    mov SBUF, a
    
    mov a, #'\n'	; new line
    lcall load_temp
    mov SBUF, a
    
	ret	
send_temp_done:
	ret
wait_1ms:
	clr	TR0 ; Stop timer 0
	clr	TF0 ; Clear overflow flag
	mov	TH0, #high(TIMER0_RELOAD_1MS)
	mov	TL0,#low(TIMER0_RELOAD_1MS)
	setb TR0
	jnb	TF0, $ ; Wait for overflow
	ret

; Wait the number of miliseconds in R2
waitms:
	lcall wait_1ms
	djnz R2, waitms
	ret


ADC_to_PB:
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x05 ; Select AIN5
	
	clr ADCF
	setb ADCS   ; ADC start trigger signal
    jnb ADCF, $ ; Wait for conversion complete

	setb PB7
	setb PB6
	setb PB5
	setb PB4
	setb PB3
	setb PB2
	setb PB1
	setb PB0
	
	; Check PB7
ADC_to_PB_L7:
	clr c
	mov a, ADCRH
	subb a, #0xf0
	jc ADC_to_PB_L6
	clr PB7
	ret

	; Check PB6
ADC_to_PB_L6:
	clr c
	mov a, ADCRH
	subb a, #0xd0
	jc ADC_to_PB_L5
	clr PB6
	ret

	; Check PB5
ADC_to_PB_L5:
	clr c
	mov a, ADCRH
	subb a, #0xb0
	jc ADC_to_PB_L4
	clr PB5
	ret

	; Check PB4
ADC_to_PB_L4:
	clr c
	mov a, ADCRH
	subb a, #0x90
	jc ADC_to_PB_L3
	clr PB4
	ret

	; Check PB3
ADC_to_PB_L3:
	clr c
	mov a, ADCRH
	subb a, #0x70
	jc ADC_to_PB_L2
	clr PB3
	ret

	; Check PB2
ADC_to_PB_L2:
	clr c
	mov a, ADCRH
	subb a, #0x50
	jc ADC_to_PB_L1
	clr PB2
	ret

	; Check PB1
ADC_to_PB_L1:
	clr c
	mov a, ADCRH
	subb a, #0x30
	jc ADC_to_PB_L0
	clr PB1
	ret

	; Check PB0
ADC_to_PB_L0:
	clr c
	mov a, ADCRH
	subb a, #0x10
	jc ADC_to_PB_Done
	clr PB0
	ret
	
ADC_to_PB_Done:
	; No pusbutton pressed	
	ret

	
jump_to_loop:
	ret
	
increment_value_rc:
	mov a, temp_refl+0
	add a, #1
	da a 
	mov temp_refl+0, a
    
    cjne a, #0x99, jump_to_loop
    mov a, temp_refl+1
    add a, #1
    da a
    mov temp_refl+1, a
    mov temp_refl+0, #0x00
	ret

increment_value_rt:
	mov a, time_refl+0
	add a, #1
	da a 
	mov time_refl+0, a
    
    cjne a, #0x99, jump_to_loop
    mov a, time_refl+1
    add a, #1
    da a
    mov time_refl+1, a
    mov time_refl+0, #0x00
	ret

increment_value_sc:
	mov a, temp_soak+0
	add a, #1
	da a 
	mov temp_soak+0, a
    
    cjne a, #0x99, jump_to_loop
    mov a, temp_soak+1
    add a, #1
    da a
    mov temp_soak+1, a
    mov temp_soak+0, #0x00
	ret

increment_value_st:
	mov a, time_soak+0
	add a, #1
	da a 
	mov time_soak+0, a
    
    cjne a, #0x99, jump_to_loop
    mov a, time_soak+1
    add a, #1
    da a
    mov time_soak+1, a
    mov time_soak+0, #0x00
	ret
loop:

	lcall ADC_to_PB
    lcall Display_Value
    Set_Cursor(1, 1)
    Send_Constant_String(#rc)
    Set_Cursor(1, 9)
    Send_Constant_String(#rt)
	Set_Cursor(2, 1)
    Send_Constant_String(#sc)
    Set_Cursor(2, 9)
    Send_Constant_String(#st)
    jb PB7, loop_rt
    Wait_Milli_Seconds(#100)
    jb PB7, loop_rt
    mov R2, #100
    lcall waitms
	
	lcall increment_value_rc

loop_rt:
	jb PB6, loop_sc
	Wait_Milli_Seconds(#100)
    jb PB6, loop_sc
    mov R2, #100
    lcall waitms

	lcall increment_value_rt

loop_sc:
	jb PB5, loop_st
	Wait_Milli_Seconds(#100)
    jb PB5, loop_st
    mov R2, #100
    lcall waitms

	lcall increment_value_sc

loop_st:
	jb PB4, loop_out
	Wait_Milli_Seconds(#100)
    jb PB4, loop_out
    mov R2, #100
    lcall waitms

	lcall increment_value_st
loop_out:
    ret

Display_Value:
    Set_Cursor(1, 4)
    Display_BCD(temp_refl+1)
    Display_BCD(temp_refl+0)

	Set_Cursor(1, 13)
	Display_BCD(time_refl+1)
	Display_BCD(time_refl+0)

	Set_Cursor(2, 4)
	Display_BCD(temp_soak+1)
	Display_BCD(temp_soak+0)

	Set_Cursor(2, 13)
	Display_BCD(time_soak+1)
	Display_BCD(time_soak+0)
    ret
   
Read_ADC:
	clr ADCF
	setb ADCS ;  ADC start trigger signal
    jnb ADCF, $ ; Wait for conversion complete
    
    ; Read the ADC result and store in [R1, R0]
    mov a, ADCRL
    anl a, #0x0f
    mov R0, a
    mov a, ADCRH   
    swap a
    push acc
    anl a, #0x0f
    mov R1, a
    pop acc
    anl a, #0xf0
    orl a, R0
    mov R0, A
	ret

    
update_temp:
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x00 ; Select channel 0

	lcall Read_ADC
	; reads signal from pin 6
	mov VAL_LM4040+0, R0
	mov VAL_LM4040+1, R1
	
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x04 ; Select channel 6
	lcall Read_ADC
	; reads signal from pin 1
	mov VAL_LM335+0, R0
    mov VAL_LM335+1, R1 

	; Read the signal from pin 14
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x07 ; Select channel 7 
	lcall Read_ADC
      
    ; moves the voltage at AIN7 into x
	mov x+0, R0
	mov x+1, R1
	mov x+2, #0
	mov x+3, #0
	
	Load_y(00040959) ; The MEASURED voltage reference: 4.0959V, with 4 decimal places 
	lcall mul32 ; 4.096 * ADCop07
	; moves ADC ref into y
	mov y+0, VAL_LM4040+0
	mov y+1, VAL_LM4040+1
	mov y+2, #0
	mov y+3, #0
	lcall div32 ; (4.096 * ADCop07) / ADCref
	;vout is in x
op_v_to_htemp:

	Load_y(1000)
	lcall mul32
	; the r1/r2
	Load_y(314) 
	lcall div32
	
	Load_y(1000)
	lcall mul32 
	Load_y(41) 
	lcall div32 
	
	
	
	mov temp+0, x+0
	mov temp+1, x+1
	mov temp+2, x+2
	mov temp+3, x+3
    ; temp = hot_temp
	
lm335_v_to_ctemp:

	mov x+0, VAL_LM335+0
	mov x+1, VAL_LM335+1
	mov x+2, #0
	mov x+3, #0
	
	Load_y(00040959)
	lcall mul32
	mov y+0, VAL_LM4040+0
	mov y+1, VAL_LM4040+1
	mov y+2, #0
	mov y+3, #0
	lcall div32 
	Load_y(100)
	lcall mul32
	Load_y(02730000)
	lcall sub32
	Load_y(00040000)
	lcall sub32
	
actual_temp:

	mov y+0, x+0
	mov y+1, x+1
	mov y+2, x+2
	mov y+3, x+3
	
	mov x+0, temp+0
	mov x+1, temp+1
	mov x+2, temp+2
	mov x+3, temp+3
	
	lcall add32
	
	mov temp+0, x+0
	mov temp+1, x+1
	mov temp+2, x+2
	mov temp+3, x+3
	
	ret

Display_formated_BCD:
	Set_Cursor(2, 10)
	Display_BCD(bcd+3)
	Display_BCD(bcd+2)
	Display_char(#'.')
	Display_BCD(bcd+1)
	ret
	
LCD_Display:
	lcall hex2bcd
	lcall Display_formated_BCD
	ret
go_to_continue_FSM1State1:
	ljmp continue_FSM1State1
check_if_shutoff:
	load_y(00500000)
	mov x+0, temp+0
	mov x+1, temp+1
	mov x+2, temp+2
	mov x+3, temp+3
	
	lcall x_lt_y
	
	jnb mf, go_to_continue_FSM1State1
	ljmp shutoff_instr
	
shutoff_instr:
	lcall re_initialize
	ljmp forever
	
re_initialize:
	Load_x(0225) 
    lcall hex2bcd
    mov temp_refl+0, bcd+0
    mov temp_refl+1, bcd+1
	
	Load_x(0045)
    lcall hex2bcd
	mov time_refl+0, bcd+0
	mov time_refl+1, bcd+1
	
	Load_x(0170)
	lcall hex2bcd
	mov temp_soak+0, bcd+0
	mov temp_soak+1, bcd+1
	
	Load_x(0090)
    lcall hex2bcd
	mov time_soak+0, bcd+0
	mov time_soak+1, bcd+1
	
	Load_x(0060)
    lcall hex2bcd
	mov shutoff+0, bcd+0
	mov shutoff+1, bcd+1
    
    mov FSM1_state, #0
    
    clr flag1
    clr flag2
    clr flag0
    
    ljmp forever


main:
 	mov sp, #0x7f
	lcall Init_All
	lcall Timer2_Init
    
    setb EA   ; Enable Global interrupts
    lcall LCD_4BIT

    Load_x(0200) 
    lcall hex2bcd
    mov temp_refl+0, bcd+0
    mov temp_refl+1, bcd+1
	
	Load_x(0030)
    lcall hex2bcd
	mov time_refl+0, bcd+0
	mov time_refl+1, bcd+1
	
	Load_x(0150)
	lcall hex2bcd
	mov temp_soak+0, bcd+0
	mov temp_soak+1, bcd+1
	
	Load_x(0070)
    lcall hex2bcd
	mov time_soak+0, bcd+0
	mov time_soak+1, bcd+1
	
	Load_x(0060)
    lcall hex2bcd
	mov shutoff+0, bcd+0
	mov shutoff+1, bcd+1
    
    mov FSM1_state, #0
    
    clr flag1
    clr flag2
    clr flag0
	ljmp forever
gotoShutoff_instr:

ljmp shutoff_instr	
;--------------start
forever:
lcall ADC_to_PB
lcall update_temp
lcall send_temp
Wait_Milli_Seconds(#50)
jnb PB1, gotoShutoff_instr
Wait_Milli_Seconds(#50)
jnb PB1, gotoShutoff_instr


; Finite State machine
FSM1:    
	mov a, FSM1_state	
FSM1_state0:

	cjne a, #0, FSM1_state1
	lcall loop
	mov pwm, #100

	
check_start_button:
 	jb PB0, check_restart_button
	Wait_Milli_Seconds(#50)
	jb PB0, check_restart_button
	mov FSM1_state, #1 
	ljmp FSM1_state0_done

check_restart_button:
	jb PB2, go_to_forever_1
	Wait_Milli_Seconds(#50)
	jb PB2, go_to_forever_1
	ljmp re_initialize
	
 FSM1_state0_done:
    lcall Start_Beep
    mov R2, #200
 	lcall waitms 
    lcall Stop_Beep
          ; Stop beep
          
    ljmp FSM2
go_to_forever_1:
ljmp forever
go_to_FSM1_state2:
ljmp FSM1_state2
 FSM1_state1:
	 cjne a, #1, go_to_FSM1_state2
	 mov pwm, #0
	 lcall LCD_Display
	 setb flag0
	 clr half_seconds_flag
;	Set_Cursor(2, 1)
;    Send_Constant_String(#state1messagebottom)
    Set_Cursor(1, 1)
    Send_Constant_String(#state1messagetop)
    
   ; lcall x_eq_y
	mov x+0, shutoff+0
    mov x+1, shutoff+1
    mov x+2, #0
    mov x+3, #0
    Load_y(0)
    lcall x_eq_y 
	 
	 jnb mf, continue_FSM1State1
	 ljmp check_if_shutoff
	 mov R2, #200
 	lcall waitms 
 	
continue_FSM1State1:
	 mov y+3, temp_soak+1
	 mov y+2, temp_soak+0
	 mov y+1, #0
	 mov y+0, #0
	 Set_Cursor(2, 1)
	 mov x+3, temp+3
	 mov x+2, temp+2
	 mov x+1, temp+1
	 mov x+0, temp+0
	 
	 lcall hex2bcd
	 mov x+3, bcd+3
	 mov x+2, bcd+2
	 mov x+1, bcd+1
	 mov x+0, bcd+0
	 
	 lcall x_gt_y ; checks if the temp > soak temp

	 jnb mf, go_to_forever_2

	 mov FSM1_state, #2
 FSM1_state1_done:
  lcall Start_Beep
    mov R2, #200
 	lcall waitms 
    lcall Stop_Beep
	 ljmp FSM2
go_to_forever_2:
ljmp forever
go_to_FSM1_state3:
ljmp FSM1_state3
 FSM1_state2:	
	 cjne a, #2, go_to_FSM1_state3
	 lcall LCD_Display
	 setb flag1
	 clr half_seconds_flag
	Set_Cursor(1, 1)
    Send_Constant_String(#state2messagetop)
	Set_Cursor(2, 1)
;    Send_Constant_String(#state1messagebottom)
	Display_BCD(time_soak+1)
	Display_BCD(time_soak+0)
	Set_Cursor(2, 6)
	lcall LCD_Display
	 mov pwm, #60
	 
	 mov x+3, temp+3
	 mov x+2, temp+2
	 mov x+1, temp+1
	 mov x+0, temp+0
	 
	 lcall hex2bcd
	 mov x+3, bcd+3
	 mov x+2, bcd+2
	 mov x+1, bcd+1
	 mov x+0, bcd+0
	 
	 mov x+3, #0
	 mov x+2, #0
	 mov x+1, time_soak+1
	 mov x+0, time_soak+0
	 load_y(0)

	 lcall x_eq_y ; check if soak time goes to 0
	 
	 jnb mf, check_again
	 ljmp change_to_State3
check_again:
	load_y(9000)
	lcall x_gt_y
	jnb mf, go_to_forever_3
	 mov R2, #200
 	lcall waitms 
change_to_State3:
 	mov FSM1_state, #3
 ;check the condition if it doesnt increase temp for 60 seconds or more	
 FSM1_state2_done:
 	lcall Start_Beep
    mov R2, #200
 	lcall waitms 
    lcall Stop_Beep
 	ljmp FSM2
go_to_forever_3:
ljmp forever
go_to_FSM1_state4:
ljmp FSM1_state4
 FSM1_state3:
 	
     cjne a, #3, go_to_FSM1_state4 
     lcall LCD_Display
     Set_Cursor(1, 1)
    Send_Constant_String(#state3messagetop)
	Set_Cursor(2, 1)
    Send_Constant_String(#state1messagebottom)
	 mov pwm, #0
	 mov y+3, temp_refl+1
	 mov y+2, temp_refl+0
	 mov y+1, #0
	 mov y+0, #0
	 
	 mov x+3, temp+3
	 mov x+2, temp+2
	 mov x+1, temp+1
	 mov x+0, temp+0
	 
	 lcall hex2bcd
	 mov x+3, bcd+3
	 mov x+2, bcd+2
	 mov x+1, bcd+1
	 mov x+0, bcd+0
	 lcall Display_formated_BCD
	 lcall x_gt_y
	 
	 jnb mf, go_to_forever_3
	 
	 mov R2, #200
 	lcall waitms 
	 mov FSM1_state, #4
 FSM1_state3_done:
 	 lcall Start_Beep
    mov R2, #200
 	lcall waitms 
    lcall Stop_Beep
	 ljmp FSM2
go_to_forever_4:
ljmp forever	 
go_to_FSM1_state5:
ljmp FSM1_state5
 FSM1_state4:
 	
	 cjne a, #4, go_to_FSM1_state5
	 lcall LCD_Display
	 setb flag2
	 clr flag1
	 clr half_seconds_flag
	 Set_Cursor(1, 1)
    Send_Constant_String(#state4messagetop)
	Set_Cursor(2, 1)
;    Send_Constant_String(#state1messagebottom)
	Display_BCD(time_refl+1)
	Display_BCD(time_refl+0)
	 mov pwm, #50
	 
	 mov x+3, temp+3
	 mov x+2, temp+2
	 mov x+1, temp+1
	 mov x+0, temp+0
	 
	 lcall hex2bcd
	 mov x+3, bcd+3
	 mov x+2, bcd+2
	 mov x+1, bcd+1
	 mov x+0, bcd+0
	 
	 mov x+3, #0
	 mov x+2, #0
	 mov x+1, time_refl+1
	 mov x+0, time_refl+0
	 load_y(0)

	 lcall x_eq_y ; check if soak time goes to 0
	 
	 jnb mf, check_again2
	 ljmp change_to_state4
	 
check_again2:
	 load_y(9000)
	 lcall x_gt_y
	 jnb mf, go_to_forever_5
change_to_state4:
	 mov R2, #200
 	lcall waitms 
 	 mov FSM1_state, #5
 FSM1_state4_done:
  lcall Start_Beep
    mov R2, #200
 	lcall waitms 
    lcall Stop_Beep
 	ljmp FSM2
 go_to_forever_5:
ljmp forever	
 FSM1_state5:
 	
	 cjne a, #5, go_to_forever_5
	 lcall LCD_Display
	 clr flag1
	 clr flag2
	 Set_Cursor(1, 1)
    Send_Constant_String(#state5messagetop)
	mov pwm, #100
	load_y(600000)
	 mov x+3, temp+3
	 mov x+2, temp+2
	 mov x+1, temp+1
	 mov x+0, temp+0
	 
	 lcall hex2bcd
	 mov x+3, bcd+3
	 mov x+2, bcd+2
	 mov x+1, bcd+1
	 mov x+0, bcd+0
	 
	 mov x+3, temp+3
	 mov x+2, temp+2
	 mov x+1, temp+1
	 mov x+0, temp+0
	 
	 lcall x_lt_y
	 
	 jnb mf, FSM2
	 
	 mov R2, #200
 	lcall waitms 
	 mov FSM1_state, #0
 FSM1_state5_done:
 	 lcall Start_Beep
    mov R2, #200
 	lcall waitms 
    lcall Stop_Beep
 	ljmp FSM2 ; 
 
 FSM2: 
 ljmp forever

 
 END
