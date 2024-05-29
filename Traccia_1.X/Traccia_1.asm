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
    #define SHOW_SLEEP
    
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
	
; TIMER1:
; In base alle impostazioni del timer, un periodo di 4 s con prescaler uguale a 2 corrisponde a
; 65536 incrementi. Il contatore di timer1 e' a 16 bit, per cui occorre
; impostare il contatore inizialmente a 65536 - 65536 = 0
	
tmr_4s		EQU		(0)	; valore iniziale del contatore di timer1 per contare 500 ms
		
; NOTA: e' una costante a 16 bit, il codice ne usera' 8 per volta per caricare
;  le due meta' del contatore di timer1

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
; buffer per la conversione del numero in decimale
decimals:
    DS		2
; variabili USART
usart_counter:
    DS		1

PSECT udata
print_buffer:
    DS		20

 
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
			pagesel	INIT_HW			; direttiva che imposta la pagina della memoria di programma in cui risiede la subroutine INIT_HW
			call	INIT_HW			; chiamata alla subroutine indicata dalla label INIT_HW

			; inizializzazione stato LED (tutti LED spenti)
			banksel	PORTD			; selezione banco RAM di PORTD
			clrf	PORTD

			; inizializzazione stato precedente porta B (pulsanti non premuti)
			movlw	0x0F
			movwf	portBPrev

			; Abilita interrupt on-change di PORTB per evento pulsante
			banksel	PORTB
			movf	PORTB, w	; legge PORTB per azzerare condizione di mismatch (vedere datasheet)
			bcf	INTCON, INTCON_RBIF_POSITION; azzera flag di avvenuto interrupt
			bsf	INTCON, INTCON_RBIE_POSITION	; abilita interrupt PORTB on-change

			; carica contatore timer1 con valore iniziale per contare 4 s
			pagesel	reload_timer1
			call	reload_timer1
			
			; abilita interrupt timer1
			banksel	PIE1
			bsf	PIE1,PIE1_TMR1IE_POSITION
			
			; Abilita gli interrupt delle periferiche aggiuntive (tra cui timer1)
			bsf	INTCON,INTCON_PEIE_POSITION

			; Abilita gli interrupt globalmente
			bsf	INTCON,INTCON_GIE_POSITION

			; Inizialmente la CPU puo' andare in sleep
			bsf	canSleep,0
			
			
main_loop:
			; Dato che tutto il lavoro e' svolto dalla routine di interrupt,
			; il programma principale potrebbe mandare il microcontrollore
			; in modalita' sleep per essere risvegliato dal successivo
			; interrupt. Ma non tutte le periferiche funzionano durante lo
			; sleep, come il timer 0 (usato per il debouncing) e il timer2
			; (usato per generare la PWM per il buzzer), che dipendono
			; dal clock della CPU.
			; Utilizziamo percio' un bit che indica quando il programma puo'
			; andare in sleep, che sara' settato dall'interrupt quando opportuno.
    
waitSleep:
			bcf	INTCON, INTCON_GIE_POSITION	; disabilita interrupt globalmente
								; stiamo iniziando una sezione critica, dunque
								; non possiamo interrompere la sequenzialita'
								; delle operazioni. Perche'?
			btfsc	canSleep, 0			; sleep possibile?
			goto	goSleep
			bsf	INTCON, INTCON_GIE_POSITION
			goto	waitSleep
    
goSleep:
			; Per vedere se va in sleep
			#ifdef SHOW_SLEEP
				banksel	PORTD
				bcf PORTD,3		; spegne LED4 prima di sleep
			#endif
			
			sleep					; la CPU si ferma!
			bsf	INTCON, INTCON_GIE_POSITION
			; a questo punto la CPU si e' risvegliata per via di un
			; interrupt, che nel nostro caso puo' essere solo un pulsante
			; (PORTB on-change interrupt) o il timer1.
			; Avendo riabilitato gli interrupt (bit GIE), viene subito
			; eseguita la routine di interrupt, quindi il programma
			; continua.
			; NOTA: abilitando una sorgente di interrupt (come RBIE) senza
			; abilitare GIE, si ottiene il risveglio dallo sleep senza
			; bisogno di avere una routine di interrupt.

			#ifdef SHOW_SLEEP
                    
			banksel	PORTD
			bsf		PORTD,3		; accende LED4 dopo risveglio
			#endif

			goto	main_loop		; ripete il loop principale del programma
			; NOTA: il codice deve sempre essere racchiuso in un loop!

    
			;*** Subroutine INIT_HW: inizializzazione dell'hardware ***
INIT_HW:		
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
			
			; USART
			; BRGH = 1, BRG16 = 0, SYNC = 0
			; BAUD RATE 9600 -> SPBRG = 25 decimale
			;
			banksel TXSTA
			movlw 00100100B
			movwf TXSTA
			banksel RCSTA
			movlw 10010000B
			movwf RCSTA
			banksel BAUDCTL
			movlw 00000000B
			movwf BAUDCTL
			banksel SPBRG
			movlw 12
			movwf SPBRG
			banksel SPBRGH
			movlw 00000000B
			movwf SPBRGH
			
			return	; uscita da subroutine e ritorno al punto del codice in cui era stata chiamata
			
			
INCREMENTO_CONTATORE:   
			incf counter, f		 ; Incrementa il valore nel registro f
			return   
			

reload_timer1:
			; ricarica contatore timer1 per ricominciare conteggio.
			; In modalita' asincrona, occorre arrestare il timer prima
			; di aggiornare i due registri del contatore
			banksel	T1CON
			bcf	T1CON,T1CON_TMR1ON_POSITION	; arresta timer
			; le funzioni "low" e "high" forniscono il byte meno e piu'
			;  significativo di una costante maggiore di 8 bit
			banksel	TMR1L
			movlw	low  tmr_4s
			movwf	TMR1L
			movlw	high tmr_4s
			movwf	TMR1H
			banksel	PIR1
			bcf	PIR1,PIR1_TMR1IF_POSITION		; azzera flag interrupt
			banksel	T1CON
			bsf	T1CON,T1CON_TMR1ON_POSITION		; riattiva timer
			return

formatta_contatore:			

	
print_eusart:
			banksel print_buffer
			movlw   'H'            ; Carattere 'H'
			movwf   print_buffer
			movlw   'e'	          ; Carattere 'e'
			movwf   print_buffer + 1
			movlw   'l'            ; Carattere 'l'
			movwf   print_buffer + 2
			movlw   'l'             ; Carattere 'l'
			movwf   print_buffer + 3
			movlw   'o'             ; Carattere 'o'
			movwf   print_buffer + 4
			movlw   10               ; Fine stringa
			movwf   print_buffer + 5
			
			movlw   6
			movwf usart_counter
			banksel print_buffer
			movlw print_buffer
			movwf FSR
			banksel PIE1
			bsf PIE1, 4
			bcf	canSleep, 0
			return
			

			
PSECT isrVec,class=CODE,delta=2
isr:			; Salvataggio stato registri CPU (context saving).
			; A differenza di quasi tutte le altre architetture, il PIC non salva lo stato
			; della CPU automaticamente all'ingresso di un interrupt (e non lo ripristina
			; all'uscita). Questo perche' non esiste uno stack utilizzabile genericamente,
			; ma solo uno stack limitato al salvataggio ed al ripristino di PC.
			; In genere quindi, per assicurare che il programma principale funzioni sempre
			; correttamente anche in presenza di interrupt, occorre gestire queste due
			; fasi manualmente. I registri da salvare per il PIC16 sono W, STATUS e PCLATH.
			; In alcuni casi cio' puo' non essere necessario:
			; - se l'interrupt esegue solo istruzioni che non alterano tali registri.
			; - se il programma principale non usa mai tali registri (ad esempio, l'esecuzione
			;   di tutti i task e' gestita dalla routine di interrupt, il programma principale
			;   esegue solo un loop vuoto).
			movwf	w_temp			; copia W in w_temp
			swapf	STATUS,w		; inverte i nibble di STATUS salvando il risultato in W.
									; Questo trucco permette di copiare STATUS senza alterarlo
									; (swapf e' una delle poche istruzioni che non alterano i bit di stato).
			movwf	status_temp		; copia W (= STATUS) in status_temp (con i nibble invertiti).
			movf	PCLATH,w		; copia il registro PCLATH in W (registro da salvare perche' contiene i
									; bit piu' significativi del program counter, usati da GOTO e CALL,
									; e settati dalla direttiva pagesel).
			movwf	pclath_temp		; copia W (= PCLATH) in pclath_temp.

			; *** codice interrupt ***
			; poiche' nel PIC16 esiste una solo routine di interrupt per tutte le sorgenti, occorre
			; testare tutti i flag che possono essere stati settati per vedere quale periferica
			; ha generato l'interrupt. Una volta processato l'interrupt, occorre anche assicurarsi
			; di azzerare tale flag, altrimenti la CPU entrerebbe in interrupt continuamente

    
			
test_timer0:
			btfss	INTCON,INTCON_T0IF_POSITION		; se il bit T0IF = 1 (c'e' stato un interrupt del timer), salta istruzione seguente
			goto 	test_button		; salta a test successivo
			btfss	INTCON,INTCON_T0IE_POSITION    ; controlla anche che l'interrupt fosse effettivamente abilitato
			goto	test_button
			; avvenuto interrupt timer0: termine debouncing
			bsf	INTCON,INTCON_RBIE_POSITION	; riabilita interrupt porta B
			bcf	INTCON,INTCON_T0IF_POSITION	; azzera flag interrupt timer
			bcf	INTCON,INTCON_T0IE_POSITION	; disabilita interrupt timer, verrà abililato quando serve in test_button
			; se il debouncing e' seguito al rilascio del pulsante,
			; segnala al programma principale che puo' andare in sleep.
			bsf	canSleep, 0	; altrimenti abilita sleep
			goto	irq_end		; vai a fine routine di interrupt
			
			
			
			
			
test_button:
			; testa evento port-change di PORTB (RBIF + RBIE)
			btfss	INTCON,INTCON_RBIF_POSITION	    ; test flag interrupt PORTB
			goto	test_timer1
			btfss	INTCON,INTCON_RBIE_POSITION	    ; test se l'interrupt PORTB è abilitato
			goto	test_timer1
			
			; avvenuto evento port-change di PORTB
			banksel	PORTB
			movf	PORTB, w	; legge PORTB eliminando condizione di mismatch
			bcf	INTCON,INTCON_RBIF_POSITION	; azzera flag evento interrupt
			; XOR di stato precedente porta (portBPrev) con stato attuale (PORTB).
			; In questo modo i bit diversi hanno risultato 1
			xorwf	portBPrev, w		; XOR di W (che conteneva PORTB) con portBPrev ( 1 in corrispondenza dei bit che hanno cambiato stato)
			andlw	0x01			; mi testerà solo il pulsante sul bit 0
			btfsc	STATUS,STATUS_Z_POSITION		; se il risultato non e' nullo (Z=0): un pulsante ha cambiato stato
			goto	button_end		; altrimenti non fare niente
			
			; il pulsante e' stato premuto o rilasciato, iniziare il conteggio di debouncing
			bcf	INTCON,INTCON_RBIE_POSITION		; disabilita interrupt PORTB per durata debouncing
			banksel	TMR0
			movlw	tmr_10ms		; carica valore iniziale per il contatore di timer0
			movwf	TMR0
			bcf	INTCON,INTCON_T0IF_POSITION		; azzera evento interrupt di timer0
			bsf	INTCON,INTCON_T0IE_POSITION		; abilita interrupt di timer0
			
			; vieta lo sleep fino a termine debouncing
			bcf	canSleep, 0	
			
			
			; testa se il pulsante è stato premuto o rilasciato
			btfss	portBPrev, 0 ; se lo stato precedente era 1 -> premuto
			; dal codice precedente abbiamo capito se c'è stato un cambiamento di stato su RB0,
			; quindi, se c'è stato un cambiamento di stato e lo stato precedente era 1, significa che 
			; RB0 è passato da 1(rilasciato)-> 0(premuto) e cioè che il pulsante è stato premuto!
			; N.B.
			; se il bit 0 della porta B è = 0 (pulsante premuto)
			; se il bit 0 della porta B è = 1 (pulsante rilasciato)
			goto	button_end  ; altrimenti non fare niente
			; pulsante premuto: azione scorrimento LED
			pagesel INCREMENTO_CONTATORE
			call	INCREMENTO_CONTATORE
			
button_end:
			; salva nuovo stato di PORTB su portBPrev
			banksel PORTB
			movf	PORTB, w
			movwf	portBPrev
			goto	irq_end		; va a fine interrupt
			; eventuali altri eventi di interrupt
			; fine codice interrupt
    
test_timer1:
			; testa evento overflow timer1 (TMR1IF + TMR1IE)
			banksel	PIR1
			btfss	PIR1,PIR1_TMR1IF_POSITION
			goto	test_uart
			banksel	PIE1
			btfss	PIE1,PIE1_TMR1IE_POSITION
			goto	test_uart
			; avvenuto interrupt timer1: toggle LED1
			
			; da sostituire con comunicazione seriale
			call print_eusart	
			movlw	0x01	
			banksel	PORTD
			xorwf	PORTD,f  ;0x01 XOR PORTD -> 1 XOR 0(led off) = 1(led on); 1 XOR 1(led on) = 0(led off)
			
			; ricarica contatore timer1
			pagesel	reload_timer1
			call	reload_timer1
			; fine evento timer1
			goto	irq_end

			; eventuali altri eventi di interrupt

			
			
			
			
test_uart:	    
			banksel PIE1
			btfss PIE1, 4	; TXIE = 4
			goto irq_end
			banksel PIR1
			btfss PIR1, 4	; TXIF = 4
			goto irq_end
		    
			; seriale pronta per trasmissione nuovo byte
			
			movf usart_counter, w
			btfsc STATUS, 2   ; Z = 2
			; byte da inviare finiti
			goto usart_tx_end
			; altro byte da inviare
			movf INDF, w
			banksel TXREG
			movwf TXREG
			incf FSR
			decf usart_counter, f
			
			
wait_usart:		
			banksel TXSTA
			btfss TXSTA, 1    ; TMRT = 1
			goto wait_usart
			goto irq_end
			
usart_tx_end:
			; caso dati da trasmettere terminati 
			banksel PIE1
			bcf PIE1, 4
			bsf	canSleep, 0
			goto irq_end
			
			
			; ripristino stato registri CPU precedente all'interruzione:
irq_end:		movf	pclath_temp,w	; copia pclath_temp in W
			movwf	PCLATH			; copia W in PCLATH
			swapf	status_temp,w	; inverte i nibble di status_temp salvando il risultato in W
			; anche in questo caso serve a non alterare STATUS stesso
			movwf	STATUS			; copia W (che contiene lo STATUS originale ripristinato dopo 2 inversioni) in STATUS
			; per ripristinare W senza alterare STATUS appena ripristinato, si utilizza sempre swapf
			swapf	w_temp,f		; prima inversione di w_temp, risultato su se stesso
			swapf	w_temp,w		; seconda inversione di w_temp, risultato in W (W contiene il valore precedente all'interrupt)

			retfie				; uscita da interrupt e ritorno al punto in cui il programma era stato interrotto

			END resetVec