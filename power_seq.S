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
.equ    ABS_LOW_5V,     200
.equ    MIN_5V_ADC,     504
.equ    MAX_5V_ADC,     557
.EQU    ABS_HIGH_5V,    800

; 12v
.equ    DETECT_12V_PIN, 1       ; Arduino A1 = ADC1 / PC1
.equ    ENABLE_12V_PIN, 3       ; Arduino D3 = PD3

; assumptions of 20k / 10k divider
; avcc is 4.82v
.equ    ABS_LOW_12V,    500
.equ    MIN_12V_ADC,    807
.equ    MAX_12V_ADC,    891
.equ    ABS_HIGH_12V,   1000

; -5v
; assumptions that divider network is 47k/68k
; pos v = 5.01
; avcc os 4.82v
.equ    ABS_LOW_NEG5V,   100
.equ    MIN_NEG5V_ADC,   173
.equ    MAX_NEG5V_ADC,   217
.equ    ABS_HIGH_NEG5V,  300

.equ    DETECT_NEG5V_ADC,   2   ; Arduino A2 = ADC2 / PC2

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

; select the multiplexer values in r17 using low 4 bits:
;   r17 -> 0 | 0 | 0 | 0 | MUX3 | MUX2 | MUX1 | MUX0
; returns result in r19:r18
; Clobbers r16, r17, SREG
GET_ADC:
    ; clear high 4 bits of r23
    andi    r17,    0x0F

    ; Select AVCC from the REFS1 and REFS0 selection registers
    ;   REFS1 = 0,  REFS0 = 1
    ;   Low 4 bits of r24 occupy MUX3-MUX0
    ldi     r16,    (1 << REFS0)
    or      r16,    r17
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

    sts     NEG5V_BAD_COUNT,	r16
    sts     _12V_BAD_COUNT,   r16
    sts	    _5V_BAD_COUNT,    r16
    ret

INIT_ERROR_STATE:
    clr	    r16

    sts     ERROR_STATE,   r16
    ret

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
    cbi     PORTB, FAULT_LED_PIN
    cbi     PORTB, GOOD_LED_PIN

    ; configure as output
    sbi     DDRD,  ENABLE_5V_PIN
    sbi     DDRD,  ENABLE_12V_PIN
    cbi     DDRB,  FAULT_LED_PIN
    cbi     DDRB,  GOOD_LED_PIN
    ret


FAULT_LED:
    cbi     PORTB,  GOOD_LED_PIN
    sbi     PORTB,  FAULT_LED_PIN
    ret

GOOD_LED:
    cbi     PORTB,  FAULT_LED_PIN
    cbi     PORTB,  GOOD_LED_PIN
    ret

;
; r19:r18 = ADC readings
; r21:r20 = absolute low threshold
; r23:r22 = min acceptable threshold
; r25:r24 = max acceptable threshold 
; r27:r26 = absolute high threshold
; r17 =   = adc channel
; returns r24:
;   2 = bad read, check if r28 has reached it's bound
;   1 = good
;   0 = Immediate shutdown
;Clobbers:
;   r16, r17-r27
CHECK_VOLTAGE:
    rcall   GET_ADC
    
    ; adc < abs_low 
    cp      r18,    r20
    cpc     r19,    r21
    brlo    TRIGGER_SHUTDOWN

    ; adc > abs_max
    cp      r26,    r18
    cpc     r27,    r19
    brlo    TRIGGER_SHUTDOWN

    ; adc < min?
    cp      r18,    r22
    cpc     r19,    r23
    brlo    VOLTAGE_BAD

    ; adc > max
    cp      r24,    r18
    cpc     r25,    r19
    brlo    VOLTAGE_BAD

    ldi     r24,    1
    ret

VOLTAGE_BAD:
    ldi     r24,    2
    ret

TRIGGER_SHUTDOWN:
    clr    r24
    ret

; Clobbers the following registers
; r16, r18, r19, r20, r21, r22, r23, r24
CHECK_NEG5V:
    ldi     r20,    lo8(ABS_LOW_NEG5V)
    ldi     r21,    hi8(ABS_LOW_NEG5V)

    ldi     r22,    lo8(MIN_NEG5V_ADC)
    ldi     r23,    hi8(MIN_NEG5V_ADC) 

    ldi     r24,    lo8(MAX_NEG5V_ADC)
    ldi     r25,    hi8(MAX_NEG5V_ADC)

    ldi     r26,    lo8(ABS_HIGH_NEG5V)
    ldi     r27,    hi8(ABS_HIGH_NEG5V)

    ldi     r17,    DETECT_NEG5V_ADC

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
    ldi     r20,    lo8(ABS_LOW_5V)
    ldi     r21,    hi8(ABS_LOW_5V)

    ldi     r22,    lo8(MIN_5V_ADC)
    ldi     r23,    hi8(MIN_5V_ADC) 

    ldi     r24,    lo8(MAX_5V_ADC)
    ldi     r25,    hi8(MAX_5V_ADC)

    ldi     r26,    lo8(ABS_HIGH_5V)
    ldi     r27,    hi8(ABS_HIGH_5V)

    ldi     r17,    DETECT_5V_PIN

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
    ldi     r20,    lo8(ABS_LOW_12V)
    ldi     r21,    hi8(ABS_LOW_12V)

    ldi     r22,    lo8(MIN_12V_ADC)
    ldi     r23,    hi8(MIN_12V_ADC) 

    ldi     r24,    lo8(MAX_12V_ADC)
    ldi     r25,    hi8(MAX_12V_ADC)

    ldi     r26,    lo8(ABS_HIGH_12V)
    ldi     r27,    hi8(ABS_HIGH_12V)

    ldi     r17,    DETECT_12V_PIN

    rcall   CHECK_VOLTAGE

    cpi     r24,    0
    breq    FAULT

    cpi     r24,     1
    breq    _12V_GOOD
_12V_BAD:
    lds    r16,      _12V_BAD_COUNT
    inc    r16
    sts    _12V_BAD_COUNT, r16

    cpi    r16,     ALLOWED_BAD_READS
    brsh   FAULT

    ret

_12V_GOOD:
    clr     r16
    sts     _12V_BAD_COUNT, r16

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
    rcall   GOOD_LED
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
    
    rcall FAULT_LED
    rjmp  FAULT_LOOP

FAULT_LOOP:
    rjmp FAULT_LOOP

DELAY_20MS:
    ; Temporary placeholder.
    ; IMPORTANT: must return.
    ret
