.include "m328pdef.inc"

.equ F_CPU = 16000000
.equ BAUD = 9600
.equ UBRR_VALUE = F_CPU/16/BAUD-1
.equ PWM_LEFT = PB1
.equ PWM_RIGHT = PB2
.equ DIR_LEFT = PD6
.equ DIR_RIGHT = PD5
.equ LED_PORT = PORTC
.equ LED_DDR = DDRC
.equ SENSOR_COUNT = 8
.equ CENTER_POS = 3500
.equ KP = 25
.equ KI = 0
.equ KD = 92

.def sensor_index = r16
.def adc_val_low = r24
.def adc_val_high = r25

.def pos_low = r10
.def pos_high = r11
.def error = r12
.def last_error = r13
.def derivative = r14
.def output = r15

.dseg
sensor_values: .byte 16

.cseg
.org 0x0000
    rjmp main

init:
    sbi DDRB, PWM_LEFT
    sbi DDRB, PWM_RIGHT
    sbi DDRD, DIR_LEFT
    sbi DDRD, DIR_RIGHT
    ldi r16, 0xFF
    out LED_DDR, r16
    ldi r16, (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1)|(1<<WGM10)
    out TCCR1A, r16
    ldi r16, (1<<CS11)
    out TCCR1B, r16
    ldi r16, (1<<ADEN)|(1<<ADSC)|(1<<ADPS2)|(1<<ADPS1)
    out ADCSRA, r16
    ret

read_adc:
    push r16
    andi r24, 0x07
    mov r16, r24
    ori r16, (1<<REFS0)
    out ADMUX, r16
    sbi ADCSRA, ADSC
wait_adc:
    sbis ADCSRA, ADIF
    rjmp wait_adc
    sbi ADCSRA, ADIF
    in r24, ADCL
    in r25, ADCH
    pop r16
    ret

qtr_read_all:
    ldi sensor_index, 0
next_sensor:
    mov r24, sensor_index
    rcall read_adc
    ldi ZH, high(sensor_values)
    ldi ZL, low(sensor_values)
    add ZL, sensor_index
    adc ZH, _zero_reg_
    st Z+, r24
    st Z, r25
    inc sensor_index
    cpi sensor_index, SENSOR_COUNT
    brlo next_sensor
    ret

fixed_mul:
    mov r30, r18
    mov r31, r19
    mul r30, r31
    mov r20, r1
    clr r21
    ret

compute_pid:
    ldi r16, low(CENTER_POS)
    ldi r17, high(CENTER_POS)
    sub pos_low, r16
    sbc pos_high, r17
    mov error, pos_high
    mov r16, last_error
    sub error, r16
    mov derivative, error
    mov last_error, error
    mov r18, KP
    mov r19, error
    rcall fixed_mul
    mov r22, r20
    mov r18, KD
    mov r19, derivative
    rcall fixed_mul
    add r22, r20
    mov output, r22
    ret

set_motors:
    ldi r16, 128
    sub r16, output
    out OCR1A, r16
    ldi r17, 128
    add r17, output
    out OCR1B, r17
    ret

main:
    rcall init

loop:
    rcall qtr_read_all
    rcall compute_pid
    rcall set_motors
    rjmp loop
