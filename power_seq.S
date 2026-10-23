#define __SFR_OFFSET 0
#include <avr/io.h>

;
;-----------------------------------------------------------
; ADC = VOLTAGE / ADC_REF * 1023
; voltage divider formula:
;       vOut = vIn * [r2 / (r1 + r2)]
;
; Ex. for 5V w/ ADC_REF = 4.82V using a 10k resistor network
;       vOut = 5.01 * (10k / (10k + 10k)) = 2.505v
;       ADC  = 2.501v / 4.82v * 1023 = 530.8 ADC
;       
;       MIN = ADC * .95  = 504 ADC
;       MAX = ADC * 1.05 = 557 ADC
;-----------------------------------------------------------
;

; leds
.equ    FAULT_LED_PIN,   9
.equ    GOOD_LED_PIN,    11


; 5v
.equ    DETECT_5V_PIN,  0       ; Arduino A0 = ADC0 / PC0
.equ    ENABLE_5V_PIN,  2       ; Arduino D2 = PD2

; assumptions that divider is 10k / 10k
; avcc is 4.82v
.equ    MIN_5V_ADC,     504
.equ    MAX_5V_ADC,     557

; 12v
.equ    DETECT_12V_PIN, 1       ; Arduino A1 = ADC1 / PC1
.equ    ENABLE_12V_PIN, 3       ; Arduino D3 = PD3

; assumptions of 20k / 10k divider
; avcc is 4.82v
.equ    MIN_12V_ADC,    807
.equ    MAX_12V_ADC,    891

; -5v
; assumptions that divider network is 47k/68k
; pos v = 5.01
; avcc os 4.82v
.equ    MIN_NEG5V_ADC,   173
.equ    MAX_NEG5V_ADC,   217
.equ    DETECT_NEG5V_ADC,   2   ; Arduino A2 = ADC2 / PC2

.equ    ADC_REF,        4.82

;
; The rails defined below will be used to store the number of bad voltage readings
; 	if ALLOWED_BAD_READS occurs on any one rail, a fault state will be entered
; 	Each rail will be cleared on every good read of that rail. 
;	Note: in the future, more advanced fault detection should be implemented
;       For instance: a GBBGGGBBGGBBBBGGBGB state should most definitely enter a FAULT state
;       Potentially we could track faults per 100 reads? Research what is safest 
;
; if a ADC is read that is either 50% (min_ADC >> 1) or > 200% ((min_ADC >> 1) + ADC) of the min ADC 
;	for that rail, then the fault state will be entered even if that rail has not reached 
;	ALLOWED_BAD_READS
;
;       ;EDIT: this information is not accurate to the current implementation...correct this later
;
;	ex. for the 5v rail the min ADC is at 504. So the ADC that will trigger an immediate
;		fault is found by - > (504 >> 1) + 504 = 252 + 504 = 756 ADC
;		So an ADC < 252 || > 756 shuts down power
;		While a range of [252, 756] will incr that rails bad read count
;
.equ	ALLOWED_BAD_READS,  5

.section .bss

NEG5V_BAD_COUNT:
	.skip 1

_5V_BAD_COUNT:
	.skip 1

_12V_BAD_COUNT:
	.skip 1

;
; High 4 bits = [Error details | Low 4 bits = Affected Rail]
; Can also be used to indicate more specific details later...
;	-5V ERROR = 1 = 0x01
;	 5V ERROR = 2 = 0x02
;       12V ERROR = 3 = 0x03
ERROR_STATE:
	.skip 1

.section .text

.global sequencer_start
.type sequencer_start, @function

.extern serial_power_good
          
sequencer_start:
    rjmp SETUP

; select the multiplexer values in r24 using low 4 bits:
;   r24 -> 0 | 0 | 0 | 0 | MUX3 | MUX2 | MUX1 | MUX0
; returns result in r19:r18
; Clobbers r16, r24, SREG
GET_ADC:
    ; clear high 4 bits of r23
    andi    r24,    0x0F

    ; Select AVCC from the REFS1 and REFS0 selection registers
    ;   REFS1 = 0,  REFS0 = 1
    ;   Low 4 bits of r24 occupy MUX3-MUX0
    ldi     r16,    (1 << REFS0)
    or      r16,    r24
    sts     ADMUX,  r16

    ; set ADEN 1, Prescaler selection to 128 (ADPS2 = ADPS1 = ADPS0 = 1)
    ldi     r16,    (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)
    sts     ADCSRA, r16


    rcall   ADC_CONVERT ; val is discarded
    rcall   ADC_CONVERT ; val is kept

    lds   r18, ADCL
    lds   r19, ADCH

    ret

ADC_CONVERT:
    ; set ADSC to start conversion

    lds     r16,    ADCSRA
    ori     r16,    (1 << ADSC)
    sts     ADCSRA, r16

ADC_WAIT:
    ; poll ADSC

    lds     r16, ADCSRA
    sbrc    r16, ADSC
    rjmp    ADC_WAIT

    ret

INIT_COUNTERS:	
    clr     r16

    sts     NEG5V_BAD_COUNTER,	r16
    sts     _12V_BAD_COUNTER,   r16
    sts	    _5V_BAD_COUNTER,    r16
    ret

INIT_ERROR_STATE:
    cls	    r16

    sts     ERROR_STATE,   r16
    ret`
INIT_ADC_PINS:
    ; set as inputs
    cbi     DDRC,   DETECT_5V_PIN
    cbi     DDRC,   DETECT_12V_PIN
    cbi     DDRC,   DETECT_NEG5V_ADC

    ; no internal pull-up
    cbi     PORTC,   DETECT_5V_PIN
    cbi     PORTC,   DETECT_12V_PIN
    cbi     PORTC,   DETECT_NEG5V_ADC   
    ret

INIT_OUTPUT_PINS:  
    ; set initial state low
    cbi     PORTD, ENABLE_5V_PIN
    cbi     PORTD, ENABLE_12V_PIN

    ; configure as output
    sbi     DDRD,  ENABLE_5V_PIN
    sbi     DDRD,  ENABLE_12V_PIN
    ret

;
; r24 = rdc channel
; r21:r20 = min adc val
; r23:r22 = max adc val
; returns r24:
;   2 = bad read, check if r28 has reached it's bound
;   1 = good
;   0 = Immediate shutdown
;Clobbers:
;   r16, r18-r24
CHECK_VOLTAGE:
    ; r24 contains rdc channel
    rcall   GET_ADC

    ; val < min?
    cp      r18,    r20
    cpc     r19,    r21
    brlo    VOLTAGE_BAD

    ; val > max
    cp      r22,    r18
    cpc     r23,    r19
    brlo    VOLTAGE_BAD

    ldi     r24,    1
    ret

VOLTAGE_BAD:
    ; if voltage is bad, we need to decide whether it is a fault or if we incr the rails bad read counter

    ; min adc / 2 = low bound to auto shutdown
    lsr     r21
    ror	    r20

    ; min adc * 2 = high bound to auto shutdown
    lsl     r23
    rol     r22

    ; val < min / 2?
    cp      r18,	r20
    cp      r19,        r21
    brlo    TRIGGER_SHUTDOWN
    
    ; this max needs more consideration giving it can exceed 1023
    ; val > min * 2?
    cp      r22,	r18
    cp	    r23,	r19
    brlo    TRIGGER_SHUTDOWN
    ldi     r24,	2	
    ret

TRIGGER_SHUTDOWN:
    clr    r24
    ret

; Clobbers the following registers
; r16, r18, r19, r20, r21, r22, r23, r24
CHECK_NEG5V:
    ldi     r20,    lo8(MIN_NEG5V_ADC)
    ldi     r21,    hi8(MIN_NEG5V_ADC) 

    ldi     r22,    lo8(MAX_NEG5V_ADC)
    ldi     r23,    hi8(MAX_NEG5V_ADC)

    ldi     r24,    DETECT_NEG5V_ADC

    rcall   CHECK_VOLTAGE

    cpi     r24,    0
    breq    FAULT

    cpi     r24,    1
    breq    NEG5V_GOOD
NEG5V_BAD:
    lds     r16,    NEG5V_BAD_COUNT
    inc     r16
    sts     NEG5V_BAD_COUNT, r16

    cpi     r16,   ALLOWED_BAD_READS
    brsh    FAULT

    ret
NEG5V_GOOD:
    clr     r16
    sts     NEG5V_BAD_COUNT, r16
    
    ret

; Clobbers the following registers
; r18, r19, r20, r21, r22, r23, r24
CHECK_5V:
    ldi     r20,    lo8(MIN_5V_ADC)
    ldi     r21,    hi8(MIN_5V_ADC) 

    ldi     r22,    lo8(MAX_5V_ADC)
    ldi     r23,    hi8(MAX_5V_ADC)

    ldi     r24,    DETECT_5V_PIN

    rcall   CHECK_VOLTAGE
    
    cpi     r24,    0
    breq    FAULT

    cpi     r24,    1
    breq    _5V_GOOD
_5V_BAD:
    lds     r16,    _5V_BAD_COUNT
    inc     r16
    sts     _5V_BAD_COUNT, r16

    cpi     r16,    ALLOWED_BAD_READS
    brsh    FAULT

    ret
_5V_GOOD:
    clr     r16
    sts     _5V_BAD_COUNT, r16
    
    ret

; Clobbers the following registers
; r18, r19, r20, r21, r22, r23, r24
CHECK_12V:
    ldi     r20,    lo8(MIN_12V_ADC)
    ldi     r21,    hi8(MIN_12V_ADC) 

    ldi     r22,    lo8(MAX_12V_ADC)
    ldi     r23,    hi8(MAX_12V_ADC)

    ldi     r24,    DETECT_12V_PIN

    rcall   CHECK_VOLTAGE

    cpi     r24,    0
    breq    FAULT

    cpi     r24,     1
    breq    _5V_GOOD
_12V_BAD:
    lds    r16,      _5V_BAD_COUNT
    inc    r16
    sts    _5V_BAD_COUNT, r16

    cpi    r16,     ALLOWED_BAD_READS
    brsh   FAULT

    ret
_12V_GOOD:
    clr     r16
    sts     _12_BAD_COUNT, r16

    ret

ENABLE_5V:
    sbi     PORTD, ENABLE_5V_PIN
    rcall   DELAY_20MS
    rcall   CHECK_5V
    ret

ENABLE_12V:
    sbi     PORTD, ENABLE_12V_PIN
    rcall   DELAY_20MS
    rcall   CHECK_12V
    ret

SETUP:
    rcall   INIT_COUNTERS
    rcall   INIT_ERROR_STATE
    rcall   INIT_ADC_PINS
    rcall   INIT_OUTPUT_PINS
    rcall   CHECK_NEG5V
    rcall   ENABLE_5V
    rcall   ENABLE_12V
    rjmp    RUN

RUN:
    rcall   CHECK_ALL_RAILS
    rjmp    RUN

CHECK_ALL_RAILS:
    rcall   CHECK_NEG5V
    rcall   CHECK_5V
    rcall   CHECK_12V

    ret

FAULT:
    ; +12 V OFF first
    cbi   PORTD, 3

    rcall DELAY_20MS

    ; +5 V OFF second
    cbi   PORTD, 2

    rjmp  FAULT_LOOP

FAULT_LOOP:
    rjmp FAULT_LOOP

DELAY_20MS:
    ; Temporary placeholder.
    ; IMPORTANT: must return.
    ret
