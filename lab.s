;-------------------------------------------------------------------------------
;Encabezado
;-------------------------------------------------------------------------------
    
; Archivo: Prelab_main.s
; Dispositivo: PIC16F887
; Autor: José Fernando de León González
; Compilador:  pic-as (v2.30), MPLABX v5.40
;
; Programa: Contador binario de 4 bits utilizando push-buttons e interrupciones y contador binario de 4 bits utilizando el timer0 e interrupciones
; Hardware: Leds y resistencias en el puerto A y puerto D, pushbuttons en el puerto B.
;
; Creado: 7/02/22
; Última modificación: 7/02/22
    
PROCESSOR 16F887

;-------------------------------------------------------------------------------
;Palabras de configuración 
;-------------------------------------------------------------------------------
    
; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF            ; Brown Out Reset Selection bits (BOR enabled)
  CONFIG  IESO = OFF             ; Internal External Switchover bit (Internal/External Switchover mode is enabled)
  CONFIG  FCMEN = OFF            ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

;-------------------------------------------------------------------------------
;Librerías incluidas
;-------------------------------------------------------------------------------
  
#include <xc.inc>

;-------------------------------------------------------------------------------
;Macros
;-------------------------------------------------------------------------------

  restart_tmr0 macro
    BANKSEL TMR0        ; banco 00
    MOVLW 100            ; cargar valor inicial a W
    MOVWF TMR0          ; cargar el valor inicial al TIMER0
    BCF T0IF            ; limpiar la bandera  de overflow del TIMER0
    endm
;-------------------------------------------------------------------------------
;Variables
;-------------------------------------------------------------------------------

  PSECT udata_shr ; Variables en la memoria RAM compartida entre bancos
    
    W_TEMP: DS 1	; 1 byte reservado (W Temporal)
    STATUS_TEMP: DS 1   ; 1 byte reservado (STATUS Temporal)
    
    value:		DS 1	; (Variable que contiene valor a mostrar en los displays de 7-seg)
    flags:		DS 1	; (Variable que indica que display hay que encender en cada instante)
    nibbles:		DS 2	; (Variable que divide los nibbles alto y bajo de valor)
    display_val:	DS 2	; Representación de cada nibble en el display de 7-seg

;-------------------------------------------------------------------------------
;Vector Reset
;-------------------------------------------------------------------------------

PSECT VectorReset, class = CODE, abs, delta = 2 ; delta = 2: Las instrucciones necesitan 2 localidades para ejecutarse & abs = absolute: indicamos que contamos a partir de 0x0000
ORG 00h  ; la localidad del vector reset es 0x0000
 
VectorReset:
    PAGESEL main
    GOTO main

;-------------------------------------------------------------------------------
;Vector de interrupción
;-------------------------------------------------------------------------------
ORG 04h	    ; posición 0004h para las interrupciones
push:
    MOVWF W_TEMP		;guardamos los valores previos del STATUS y el W en variables temporales
    SWAPF STATUS, W
    MOVWF STATUS_TEMP
isr:
    
    BTFSC   RBIF
    call    int_iocb
    BTFSC   T0IF
    call    int_tmr0
    
pop:				;regresamos los valores de W y STATUS
    SWAPF STATUS_TEMP, W
    MOVWF STATUS
    SWAPF W_TEMP, F
    SWAPF W_TEMP, W
    retfie

;-------------------------------------------------------------------------------
;Subrutinas de interrupción
;-------------------------------------------------------------------------------
    
int_iocb:
    banksel PORTB	    ; Vamos al banco del PORTB
    btfss PORTB, 0	    ; Revisamos si el botón del bit0 de PORTB sigue presionado
    incf PORTA		    ; NO: Incrementamos PORTA
    btfss PORTB, 1	    ; Sí: Revisamos si el botón del bit1 de PORTB sigue presionado
    decf PORTA		    ; NO: Decrementamos PORTA
    BCF RBIF		    ; SÍ: limpiamos la bandera de interrupción
    return
   
int_tmr0:
    
    call display_selection
    restart_tmr0
    return
    
display_selection:

    BCF	    PORTD, 0		; Apagamos display de nibble alto
    BCF	    PORTD, 1		; Apagamos display de nibble bajo
    BTFSC   flags, 0		; Verificamos bandera
    goto    display_0		;  
    goto    display_1
    
    return
    
display_0:			
    MOVF    display_val, W	; Movemos display a W
    MOVWF   PORTC		; Movemos Valor de tabla a PORTC
    BSF	PORTD, 1	; Encendemos display de nibble bajo
    BCF	flags, 0	; Cambiamos bandera para cambiar el otro display en la siguiente interrupción
    
    return

display_1:
    MOVF    display_val+1, W	; Movemos display+1 a W
    MOVWF   PORTC		; Movemos Valor de tabla a PORTC
    BSF	PORTD, 0	; Encendemos display de nibble alto
    BSF	flags, 0	; Cambiamos bandera para cambiar el otro display en la siguiente interrupción
    
    return
;-------------------------------------------------------------------------------
;Tabla para display de siete segmentos
;-------------------------------------------------------------------------------

PSECT table, class = CODE, abs, delta = 2
ORG 100h 

table:
    CLRF PCLATH
    BSF PCLATH, 0           ; PCLATH en 01
    ANDLW 0X0F
    ADDWF PCL               ; PC = PCLATH + PCL | Sumamos W al PCL para seleccionar un dato en la tabla
    retlw 00111111B         ; 0
    retlw 00000110B         ; 1
    retlw 01011011B         ; 2
    retlw 01001111B         ; 3
    retlw 01100110B         ; 4
    retlw 01101101B         ; 5
    retlw 01111101B         ; 6
    retlw 00000111B         ; 7
    retlw 01111111B         ; 8 
    retlw 01101111B         ; 9
    retlw 01110111B         ; A
    retlw 01111100B         ; b
    retlw 00111001B         ; C
    retlw 01011110B         ; D
    retlw 01111001B         ; C
    retlw 01110001B         ; F

;-------------------------------------------------------------------------------
;main (configuración)
;-------------------------------------------------------------------------------
main:			    
    call config_clock	    ; configuramos el reloj 
    call config_tmr0	    ; configuramos el TIMER0
    call config_ports	    ; configuramos puertos
    call config_IOCRB	    ; configuramos las interruptions on change
    call config_int_enable  ; activamos las interrupciones
    
    banksel PORTA
    			    ;---------------------------------------------------
			    ;TEMPORIZACIÓN TIMER0: 100 ms = 4*4us*(256-100)*4
			    ;---------------------------------------------------
    
;-------------------------------------------------------------------------------
;Loop
;-------------------------------------------------------------------------------

loop:
    MOVF   PORTA, W		; Valor del PORTA a W
    MOVWF   value		; Movemos W a variable valor
    call    nibble_save		; Guardamos nibble alto y bajo de valor
    call    display		; Guardamos los valores a enviar en PORTC para mostrar valor en hex
    goto loop
;-------------------------------------------------------------------------------
;subrutinas
;-------------------------------------------------------------------------------
config_IOCRB:
    banksel TRISA
    BSF IOCB, 0
    BSF IOCB, 1	    ; seteamos los bits 0 y 1 del puerto B como interrupt on change
    
    banksel PORTA
    MOVF PORTB, W   ; Al leer termina condición de mismatch
    BCF RBIF
    
    return
config_ports:
    
    banksel ANSEL       ; banco 11
    CLRF ANSEL		; pines digitales
    CLRF ANSELH
    
    banksel TRISA       ; banco 01
    CLRF TRISA		; PORTA como salida
    CLRF TRISC		; PORTC como salida
    CLRF TRISD		; PORTD como salida
    
    BSF  TRISB0
    BSF  TRISB1         ; pines 1 & 2 del puerto B como entradas
    
    BCF OPTION_REG, 7	;Habilitar Pull-ups
    
    banksel PORTA       ; banco 00
    CLRF PORTA		; limpiamos PORTA
    CLRF PORTC		; limpiamos PORTC
    CLRF PORTD		; limpiamos PORTD
    CLRF flags
    return
    
config_int_enable:
    BSF GIE ; INTCON
    BSF RBIE
    BCF RBIF
    
    BSF T0IE
    BCF T0IF
    return
    
config_clock:
    banksel OSCCON      ;banco 01
    BCF IRCF2
    BSF IRCF1
    BCF IRCF0           ; IRCF <2:0> -> 010 250 kHz
    
    BSF SCS             ;reloj interno
    return
    
config_tmr0:
    banksel TRISA  ; banco 01
    BCF T0CS            ; TIMER0 como temporizador
    BCF PSA             ; Prescaler a TIMER0
    BCF PS2
    BCF PS1
    BSF PS0             ; PS<2:0> -> preescaler 001 1:4
    
    restart_tmr0    
    return
    
nibble_save:
    MOVLW   0x0F		    
    ANDWF   value, W		; Se hace un AND de value con el valor en W para guardar solo el nibble bajo
    MOVWF   nibbles		; Guardar el nibble bajo en el primer registro de la variable nibbles	
				 
    MOVLW   0xF0		      
    ANDWF   value, W		; Se hace un AND con el valor en W para guardazr solo el nibble alto	  
    MOVWF   nibbles+1		; Enviar el valor al segundo registro de la variable nibbles	      
    SWAPF   nibbles+1, F	; Utilizar un SWAPF para mover los nibbles de su posición high a la posición low	      
    
    return
    
display:
    MOVF    nibbles, W		; Movemos nibble bajo a W
    call    table		; Buscamos valor a cargar en PORTC
    MOVWF   display_val		; Guardamos en el primer registro de la variable display_val
    
    MOVF    nibbles+1, W	; Movemos nibble alto a W
    call    table		; Buscamos valor a cargar en PORTC
    MOVWF   display_val+1	; Guardamos en en el segundo registro de la variable display_val
    
    return 
END





