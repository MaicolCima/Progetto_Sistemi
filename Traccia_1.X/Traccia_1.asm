; ***********************************************************
; Autori: 
; Cimarelli Maicol & Di Carlo Gabriele
;
;Descrizione software:
; - Firmware che conta quante volte un pulsante viene premuto,
;   ogni 4 secondi scrive su porta seriale (UART) il totale di pressioni fino
;   a quel momento, sotto forma di numero a due cifre decimali.
; - utilizzo del TIMER1 per conteggio dei 4 secondi
; - utilizzo del TIMER0 per controllo debouncing
; - utilizzo della PORTB per pulsante
;
;Descrizione hardware
; - scheda Cedar Pic Board (vedere schematico).
;   - MCU: PIC16F887 (clock interno 4 MHz)
;   - Pulsante: RB0
;
; Uso memoria:
; - ROM: ~ ///// word
; - RAM: /////// byte
; ***********************************************************
;
;
;
; 1) Direttiva Processor
    
    PROCESSOR 16F887		// direttiva che definisce il tipo di processore
    
; 2) Direttiva #include
    
    #include <xc.inc>		// file che contiene le definizioni dei simboli (nomi registri, nomi bit dei registri, ecc).
    
; 3) Direttiva CONFIG
    
        CONFIG "FOSC = INTRC_NOCLKOUT"// Oscillatore interno
	CONFIG "WDTE = OFF"           // Watchdog Timer disabilitato 
	CONFIG "PWRTE = OFF"	      // Power-up Timer disabilitato 
	CONFIG "CP = OFF"             // Program memory code protection disabilitato
	CONFIG "CPD = OFF"            // Data memory code protection disabilitato
	CONFIG "BOREN = OFF"	      // Brown-out Reset disabilitato 
	CONFIG "LVP = OFF"            // Low-Voltage Programming disabilitato
	CONFIG "DEBUG = OFF"          // Background debugger disabilitato
	CONFIG  "BOR4V = BOR21V"       // Brown-out Reset Selection bit (Brown-out Reset set to 2.1V)
	CONFIG  "WRT = OFF"            // Flash Program Memory Self Write Enable bits (Write protection off)
	
;
; 4) Direttiva EQU
; ***Definizione di costanti***
; Costanti con cui settare il contatore del timer per contare un determinato intervallo di tempo.
; Poiché le impostazioni del timer (vedere subroutine INIT_HW) determinano un periodo per un singolo
; incremento di 256 us, un periodo di 10 ms (ad esempio) equivale a 39 incrementi (ticks).
; Il timer setta il flag di oveflow (bit T0IF del registro INTCON) al
; passaggio da 0xFF a 0x00, per cui occorre impostare il contatore TMR0
; al valore 256 - 39.
; (La direttiva EQU assegna una costante ad una label).

tmr_10ms	EQU		(256 - 39)	; valore iniziale del contatore di TMR0 per contare 10 ms
	

; ***Definizione di variabili***

PSECT udata_shr
; variabili utilizzate per salvare lo stato della CPU all'ingresso della routine di interrupt
w_temp:
    DS		1			; riserva un byte di memoria associato alla label w_temp
status_temp:
    DS		1			; riserva un byte di memoria associato alla label status_temp
pclath_temp:
    DS		1			; riserva un byte di memoria associato alla label pclath_temp
; variabile per memorizzare lo stato precedente di PORTB
portBPrev:
    DS		1
; flag che indica quando la CPU può andare in sleep (solo il bit 0 e' utilizzato)
canSleep:
    DS		1
; variabile che indica il contatore
counter:
    DS		1
    

    
GLOBAL resetVec,isr
    ciao
    
    




