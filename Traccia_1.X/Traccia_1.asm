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

        

; *** Vettore di reset ***
    
PSECT resetVec,class=CODE,delta=2
resetVec:
			pagesel	start			; imposta la pagina della memoria di programma in cui si trova l'indirizzo della label start
			goto	start			; salta all'indirizzo indicato dalla label start



;
;
; *** Programma principale ***
;
;
PSECT MainCode,global,class=CODE,delta=2
start:			; N.B: l'assembler non accetta una label sulla stessa riga di una direttiva
			; inizializzazione hardware
			
			
main_loop:
    
waitSleep:
    
goSleep:
    
    
    
    
    
			;*** Subroutine INIT_HW: inizializzazione dell'hardware ***
INIT_HV:		
			; OPTION_REG:
			; - Bit 7: PULL UP enable negato sulla PORTB (0)
			; - Bit 6: Interrupt edge select bit 1 rising / 0 falling (0)
			; - Bit 5: TIMER0 clock source select 1 T0CKI / 0 Internal (Fosc/4) (0)
			; - Bit 4: TIMET0 source edge select 1 falling / 0 rising (0)
			; - Bit 3: Prescaler assignment 1 WDT / 0 TIMER0 (0)
			; - Bit 2-0: Prescaler rate select (111) TIMER0 rate 1:256
			banksel OPTION_REG		; banco di OPTION_REG
			movlw	00000111B		 
			movwf	OPTION_REG		; copia W (11000110b) in OPTION_REG
			
			
			; registro INTCON:
			; - tutti gli interrupt inzialmente disabilitati
			; (verranno abilitati nel programma principale, quando tutte
			;  le periferiche saranno correttamente inizializzate)
			clrf	INTCON
			
			
			; Porte I/O:
			; - porte A,C,B,E settate come input digitali per tutti i pin
			; - porta D: pin 0..3 settati come output (LED), pin 4..7 come input
			banksel	TRISA			; banco di TRISA, stesso banco anche per gli altri registri TRISx
			movlw	0xFF			; carica costante FF in W
			movwf	TRISA			; copia W (FF) in TRISA
			movwf	TRISC			; copia W (FF) in TRISC
			movwf	TRISB			; copia W (FF) in TRISB
			movwf	TRISE			; copia W (FF) in TRISE
			movlw	0xF0			; carica costante F0 in W
			movwf	TRISD			; copia W (F0) in TRISD
    

			; Di default, tutti i pin connessi all'ADC sono settati come input analogici,
			; impedendone l'uso come I/O digitali. L'impostazione seguente rende I/O digitali
			; i pin RB0..RB3
			banksel ANSELH			; banco di ANSELH
			clrf	ANSELH			; AN8..AN13 disattivati

			; PORTB: abilita interrupt on-change sui 4 bit più bassi 
			; NOTA: 
			; In realtà le specifiche ci richiedono di utilizzare solo RB0 quindi dovremmo
			; scrivere 0x01 su IOCB ed abilitare l'IOC solo su RB0.
			; Decidiamo di attivare RB<0-3> per mostrare come scrivere il codice non caso 
			; più frequente di utilizzo di più pulsanti.		
			movlw	0x0F
			banksel	IOCB
			movwf	IOCB

			; Timer1
			; Impostazioni:
			; - usa quarzo esterno (32768 Hz)
			; - modalita' asincrona (funziona con quarzo esterno anche durante sleep)
			; - prescaler = 2
			; - timer off
			; Con la frequenza del quarzo ed il prescaler a 2 si ha:
			; - singolo tick ~= 61.036 us
			; - periodo max = 4 s (contatore a 16 bit)
			
			; TlCON:
			; - Bit 7: TIMER1 gate inverter 1 High / 0 Low (0)
			; - Bit 6: TIMER1 gate enable bit 1 gate controlled / 0 always counting (0)
			; - Bit 5-4: TIMER1 input clock prescale prescale 1:2 (01)
			; - Bit 3: TIMER1 oscillator enable control 1 LP oscillator enable clock / 0 LP oscillator OFF (1)
			; - Bit 2: External clock input syncronization 1 Non sincronizzato / 0 sincronizzato (1)
			; - Bit 1: TMR1CS TIMER clock sourc select 1 external / 0 internal (1)
			; - Bit 0: TMR1 On 1 enables / 0 stops (0)
			banksel	T1CON
			movlw	00011110B                 ;TMR1 OFF
			movwf	T1CON
			
			
			return	; uscita da subroutine e ritorno al punto del codice in cui era stata chiamata


