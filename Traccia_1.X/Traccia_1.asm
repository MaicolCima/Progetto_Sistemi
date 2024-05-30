; ******************************************************************************
; Autori: 
; Cimarelli Maicol & Di Carlo Gabriele
;
; Descrizione software:
; Realizzare un firmware che conti quante volte un pulsante viene premuto, 
; ed ogni 4 secondi scrivere su porta seriale (UART) il totale di pressioni 
; fino a quel momento, sotto forma di numero a due cifre decimali.
;
; - utilizzo del TIMER1 per conteggio dei 4 secondi
; - utilizzo del TIMER0 per debouncing pulsante
; - utilizzo del pulsante 1 collegato sul pin 0 di PORTB
; - utilizzo della comunicazione UART per inviare il conteggio delle pressioni max 99
; - micro in sleep quando possibile
;
; Descrizione hardware
; - scheda Cedar Pic Board (vedere schematico).
; - MCU: PIC16F887 (clock interno 4 MHz)
; - Pulsante: RB0
;
; Uso memoria:
; - ROM: ~ 8192 words
; - RAM: 368 bytes
; ******************************************************************************
;
; Direttiva PROCESSOR
    
    PROCESSOR 16F887				    // direttiva che definisce il tipo di processore
    
; Direttiva #include
    
    #include <xc.inc>				    // file che contiene le definizioni dei simboli (nomi registri, nomi bit dei registri, ecc).
    
; TEST: definizione per mostrare tramite LED4 lo stato di sleep o di CPU attiva.
; spento prima di entrare in sleep, acceso durante il periodo di risveglio
    
    #define SHOW_SLEEP
    
; Direttiva CONFIG
    
	    CONFIG "FOSC = INTRC_NOCLKOUT"	    // Oscillatore interno
	    CONFIG "WDTE = OFF"			    // Watchdog Timer disabilitato 
	    CONFIG "PWRTE = OFF"		    // Power-up Timer disabilitato 
	    CONFIG "CP = OFF"			    // Program memory code protection disabilitato
	    CONFIG "CPD = OFF"			    // Data memory code protection disabilitato
	    CONFIG "BOREN = OFF"		    // Brown-out Reset disabilitato 
	    CONFIG "LVP = OFF"			    // Low-Voltage Programming disabilitato
	    CONFIG "DEBUG = OFF"		    // Background debugger disabilitato
	    CONFIG  "BOR4V = BOR21V"		    // Brown-out Reset Selection bit (Brown-out Reset set to 2.1V)
	    CONFIG  "WRT = OFF"			    // Flash Program Memory Self Write Enable bits (Write protection off)
	
; Direttiva EQU
	    
; ***Definizione di costanti***

; TIMER0	    
; Costanti con cui settare il contatore del timer per contare un determinato intervallo di tempo.
; Poich� le impostazioni del timer (vedere subroutine INIT_HW) determinano un periodo per un singolo
; incremento di 256 us, un periodo di 10 ms (ad esempio) equivale a 39 incrementi (ticks).
; Il timer setta il flag di oveflow (bit T0IF del registro INTCON) al
; passaggio da 0xFF a 0x00, per cui occorre impostare il contatore TMR0
; al valore 256 - 39.
; (La direttiva EQU assegna una costante ad una label).

tmr_10ms	EQU		(256 - 39)	    ; valore iniziale del contatore di TMR0 per contare 10 ms
	
; TIMER1:
; In base alle impostazioni del timer, un periodo di 4 secondi, con prescaler uguale a 2, 
; corrisponde a 65536 incrementi. Il contatore di TIMER1 e' a 16 bit, per cui occorre
; impostare il contatore inizialmente a 65536 - 65536 = 0
	
tmr_4s		EQU		(0)		    ; valore iniziale del contatore di timer1 per contare 500 ms
		
; NOTA: e' una costante a 16 bit, il codice ne usera' 8 per volta per caricare
; le due meta' del contatore di TIMER1

; ***Definizione di variabili***

PSECT udata_shr
 
; Variabili utilizzate per salvare lo stato della CPU all'ingresso della routine di interrupt
w_temp:
    DS		1				    ; riserva un byte di memoria associato alla label w_temp
status_temp:
    DS		1				    ; riserva un byte di memoria associato alla label status_temp
pclath_temp:
    DS		1				    ; riserva un byte di memoria associato alla label pclath_temp
    
; Variabile per memorizzare lo stato precedente di PORTB
portBPrev:
    DS		1
; Flag che indica quando la CPU pu� andare in sleep (solo il bit 0 e' utilizzato)
canSleep:
    DS		1
; Variabile che indica il contatore
counter:
    DS		1
    
; Variabili USART:
; Contatore dell'USART
usart_counter:
    DS		1
; Contatore decine
contatoreD:
    DS		1
; Contatore unit�
contatoreU:
    DS		1

PSECT udata
print_buffer:
    DS		3

; Etichette resetVec e isr dichiarate come globali
    
GLOBAL resetVec,isr

; *** Vettore di reset ***

PSECT resetVec,class=CODE,delta=2
resetVec:
	pagesel	start				    ; Imposta la pagina della memoria di programma in cui si trova l'indirizzo della label start
	goto	start				    ; Salta all'indirizzo indicato dalla label start

; *** Programma principale ***

PSECT MainCode,global,class=CODE,delta=2
start:						    ; N.B: l'assembler non accetta una label sulla stessa riga di una direttiva
    
; Inizializzazione hardware
	pagesel	INIT_HW				    ; Direttiva che imposta la pagina della memoria di programma in cui risiede la subroutine INIT_HW
	call	INIT_HW				    ; Chiamata alla subroutine indicata dalla label INIT_HW
			
; Inizializzazione contatore a 0
	movlw 00000000B
	movwf counter
			
; Inizializzazione stato LED (tutti LED spenti)
	banksel	PORTD				    ; Selezione banco RAM di PORTD
	clrf	PORTD

; Inizializzazione stato precedente porta B (pulsanti non premuti)
	movlw	0x0F
	movwf	portBPrev

; Abilita interrupt on-change di PORTB per evento pulsante
	banksel	PORTB
	movf	PORTB, w			    ; Legge PORTB per azzerare condizione di mismatch (vedere datasheet)
	bcf	INTCON, INTCON_RBIF_POSITION	    ; Azzera flag di avvenuto interrupt
	bsf	INTCON, INTCON_RBIE_POSITION	    ; Abilita interrupt PORTB on-change

; Carica contatore TIMER1 con valore iniziale per contare 4 secondi
	pagesel	reload_timer1
	call	reload_timer1
			
; Abilita interrupt TIMER1
	banksel	PIE1
	bsf	PIE1,PIE1_TMR1IE_POSITION
			
; Abilita gli interrupt delle periferiche aggiuntive (tra cui TIMER1)
	bsf	INTCON,INTCON_PEIE_POSITION

; Abilita gli interrupt globalmente
	bsf	INTCON,INTCON_GIE_POSITION

; Inizialmente la CPU puo' andare in sleep
	bsf	canSleep,0
						
main_loop:
    
; Dato che tutto il lavoro e' svolto dalla routine di interrupt,
; il programma principale potrebbe mandare il microcontrollore
; in modalit� sleep per essere risvegliato dal successivo
; interrupt. Ma non tutte le periferiche funzionano durante lo
; sleep, come il TIMER0 (usato per il debouncing), che dipende
; dal clock della CPU.
; Utilizziamo perci� un bit che indica quando il programma puo'
; andare in sleep, che sara' settato dall'interrupt quando opportuno.
    
waitSleep:
	bcf	INTCON, INTCON_GIE_POSITION	    ; Disabilita interrupt globalmente
						    ; Stiamo iniziando una sezione critica, dunque
						    ; non possiamo interrompere la sequenzialita'
						    ; delle operazioni. 
	btfsc	canSleep, 0			    ; Sleep possibile?
	goto	goSleep
	bsf	INTCON, INTCON_GIE_POSITION
	goto	waitSleep
    
goSleep:
; Per vedere se va in sleep
	#ifdef SHOW_SLEEP
	banksel	PORTD
	bcf PORTD,3				    ; Spegne LED4 prima di sleep
	#endif
			
	sleep					    ; La CPU si ferma!
	bsf	INTCON, INTCON_GIE_POSITION
; A questo punto la CPU si e' risvegliata per via di un
; interrupt, che nel nostro caso puo' essere solo un pulsante
; (PORTB on-change interrupt) o il TIMER1.
; Avendo riabilitato gli interrupt (bit GIE), viene subito
; eseguita la routine di interrupt, quindi il programma continua.

; NOTA: abilitando una sorgente di interrupt (come RBIE) senza
; abilitare GIE, si ottiene il risveglio dallo sleep senza
; bisogno di avere una routine di interrupt.

	#ifdef SHOW_SLEEP
                    
	banksel	PORTD
	bsf		PORTD,3			    ; Accende LED4 dopo risveglio
	#endif

	goto	main_loop			    ; Ripete il loop principale del programma

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
    
	banksel OPTION_REG			    ; Banco di OPTION_REG
	movlw	00000111B		 
	movwf	OPTION_REG			    ; Copia W (00000111B) in OPTION_REG
			
; Registro INTCON:
; - tutti gli interrupt inzialmente disabilitati
; (verranno abilitati nel programma principale, quando tutte
; le periferiche saranno correttamente inizializzate)
	clrf	INTCON
			
; Porte I/O:
; - porte A,C,B,E settate come input digitali per tutti i pin
; - porta D: pin 0..3 settati come output (LED), pin 4..7 come input
	
	banksel	TRISA				    ; banco di TRISA, stesso banco anche per gli altri registri TRISx
	movlw	0xFF				    ; carica costante FF in W
	movwf	TRISA				    ; copia W (FF) in TRISA
	movwf	TRISC				    ; copia W (FF) in TRISC
	movwf	TRISB				    ; copia W (FF) in TRISB
	movwf	TRISE				    ; copia W (FF) in TRISE
	movlw	0xF0				    ; carica costante F0 in W
	movwf	TRISD				    ; copia W (F0) in TRISD
    

; Di default, tutti i pin connessi all'ADC sono settati come input analogici,
; impedendone l'uso come I/O digitali. L'impostazione seguente rende I/O digitali
; i pin RB0..RB3
	banksel ANSELH				    ; banco di ANSELH
	clrf	ANSELH				    ; AN8..AN13 disattivati

; PORTB: abilita interrupt on-change sui 4 bit pi� bassi 
; NOTA: 
; In realt� le specifiche ci richiedono di utilizzare solo RB0 quindi dovremmo
; scrivere 0x01 su IOCB ed abilitare l'IOC solo su RB0.
; Decidiamo di attivare RB<0-3> per mostrare come scrivere il codice non caso 
; pi� frequente di utilizzo di pi� pulsanti.		
	movlw	0x0F
	banksel	IOCB
	movwf	IOCB

; TIMER1
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
	movlw	00011110B			    ;TMR1 OFF
	movwf	T1CON
			
; Inizializzazione USART
	
; Set dei registri TXSTA, RCSTA, BAUDCTL e SPBRG	
; BRGH = 1, BRG16 = 0, SYNC = 0
; BAUD RATE 9600 -> SPBRG = 25 decimale

; TXSTA:
; - Bit 7: Clock Source Select asincrona don't care (0)
; - Bit 6: 9-bit trasmission enable (0 disabled)
; - Bit 5: Trasmit enable bit (1 enabled)
; - Bit 4: SYNC (0 asynchronous mode)
; - Bit 3: 0 non utilizzato
; - Bit 2: BRGH high baud rate select bit (1 high speed)
; - Bit 1: Trasmit shift register STATUS bit (1 empty / 0 full)
; - Bit 0: TX9D nono bit di trasmissione (non utilizzato in questo caso)
	
	banksel TXSTA				    ;
	movlw 00100100B
	movwf TXSTA
	
; RCSTA:
; - Bit 7: Serial port enable bit (1 abilitata)
; - Bit 6: nono bit attivo in ricezione (0 ricezione 8-bit)
; - Bit 5: SREN asynchronous don't care 
; - Bit 4: continous receive enable bit (0)
; - Bit 3: Address detect enable (0 disabilitato)
; - Bit 2: Framing error bit (0 no framming error)
; - Bit 1: Overrun error bit (0 no overrun error)
; - Bit 0: RX9D	nono bit ricevuto
	
	banksel RCSTA
	movlw 10000000B
	movwf RCSTA

; BAUDCTL:
; - Bit 7: auto-baud detect overflow bit (1 overflow / 0 no overflow)
; - Bit 6: recive idle flag (1 idle / 0 receiving)
; - Bit 5: unimplemented 
; - Bit 4: Synchronous clock polarity select (0 non inverted data)
; - Bit 3: 16-bit baud rate generator bit (0 8-bit baud rate generator)
; - Bit 2: unimplemented 
; - Bit 1: wake-up enable (0 receiver is operating normally)
; - Bit 0: auto baud detect enable bit (0 auto baud detect mode is disabled)
	
	banksel BAUDCTL
	movlw 00000000B
	movwf BAUDCTL

; SPBRG registro di settaggio del baud rate:
; con BRGH = 1 in TXSTA Baud rate = Fosc / (16*(SPBRG + 1))
; Con Fosc = 4MHz vogliamo Baud rate ~= 19200 quindi SPBRG = 12
	
	banksel SPBRG
	movlw 12
	movwf SPBRG
	banksel SPBRGH
	movlw 00000000B
	movwf SPBRGH
			
	return					    ; Uscita da subroutine e ritorno al punto del codice in cui era stata chiamata
			
			
INCREMENTO_CONTATORE:   
			incf counter, f		    ; Incremento del valore nel registro counter
			return   
			

reload_timer1:

; Ricarica contatore TIMER1 per ricominciare conteggio.
; In modalita' asincrona, occorre arrestare il timer prima di aggiornare i due registri del contatore

	banksel	T1CON
	bcf	T1CON,T1CON_TMR1ON_POSITION	    ; Arresta timer
; Le funzioni "low" e "high" forniscono il byte meno e piu'
; significativo di una costante maggiore di 8 bit
	banksel	TMR1L
	movlw	low  tmr_4s
	movwf	TMR1L
	movlw	high tmr_4s
	movwf	TMR1H
	banksel	PIR1
	bcf	PIR1,PIR1_TMR1IF_POSITION	    ; azzera flag interrupt
	banksel	T1CON
	bsf	T1CON,T1CON_TMR1ON_POSITION	    ; riattiva timer
	return

; Subroutine per la stampa del contatore sulla porta seriale UART	
	
print_eusart:
    
; Formattazione del contatore per la stampa
    
    formatta_contatore:	
    
		    movlw 00000000B		    ; Carico il valore zero in w
		    movwf contatoreD		    ; Azzero il contatore delle decine
		    movf  counter,w		    ; Carico il valore di counter in w
		    movwf contatoreU		    ; Carico il valore di counter nel contatore delle unit�
		    
		loop_div_10:			    ; Inizio della routine di divisione per 10 tramite sottrazione
		    movlw 10			    ; Carica il valore 10 nel registro w 
		    subwf contatoreU, w		    ; w = contatoreU - 10
		    btfss STATUS, 0		    ; Controlla il bit 0 (Carry flag) del registro STATUS. C = 0 indica un prestito quindi termino la divisione
		    goto end_div_10		    ; salta all'etichetta end_div_10, terminando il ciclo di divisione
		    movwf contatoreU		    ; Altrimenti, se C = 1 (non c'� stato prestito) contatoreU = contatoreU - 10
		    incf contatoreD, f		    ; Incrementa contatoreD, che conta il numero di sottrazioni
		    goto loop_div_10		    ; Ritorna all'inizio del loop per eseguire un'altra sottrazione
		end_div_10:			    ; Fine della routine di divisione
		
	
; Preparazione buffer di stampa
    
; Per trasmettere le cifre carico w con la cifra 0 e poi vado ad aggiungere la cifra corrispondente
; alla decina o all'unit�, sfruttando la sequenzialit� dei caratteri ASCII. 
    
		    banksel print_buffer	    ; Seleziona bank di memoria in cui � definito print_buffer
		    movlw   '0'			    ; Carica il carattere '0' nel registro w 
		    addwf   contatoreD, w	    
		    movwf   print_buffer	    
		    movlw   '0'			    
		    addwf   contatoreU, w	    
		    movwf   print_buffer + 1
		    movlw   10			    ; Carattere "a capo"
		    movwf   print_buffer + 2
		    
		    movlw   3			    ; Carico il numero di byte da stampare in w
		    movwf usart_counter		    
		    banksel print_buffer	    ; Seleziono il banco di RAM contentente il buffer di stampa
		    movlw print_buffer		    ; Carico il print buffer in w
		    movwf FSR			    ; carico l'indirizzo del print buffer nell'FSR
		    banksel PIE1		    
		    bsf PIE1, 4			    ; abilito l'interrupt della USART per trasmettere
		    bcf	canSleep, 0		    ; non pu� andare in sleep fino a quando non finisce la trasmissione
		    return
			
	
PSECT isrVec,class=CODE,delta=2
isr:			
; Salvataggio stato registri CPU (context saving).
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
    
	movwf	w_temp				    ; Copia w in w_temp
	swapf	STATUS,w			    ; Inverte i nibble di STATUS salvando il risultato in w.
						    ; In questo modo, si pu� copiare STATUS senza alterarlo
						    ; (swapf � una delle poche istruzioni che non alterano i bit di stato).
	movwf	status_temp			    ; copia w (= STATUS) in status_temp (con i nibble invertiti).
	movf	PCLATH,w			    ; copia il registro PCLATH in w (registro da salvare perche' contiene i
						    ; bit piu' significativi del program counter, usati da GOTO e CALL,
						    ; e settati dalla direttiva pagesel).
	movwf	pclath_temp			    ; copia w (= PCLATH) in pclath_temp.

; *** codice interrupt ***
; Poiche' nel PIC16 esiste una solo routine di interrupt per tutte le sorgenti, occorre
; testare tutti i flag che possono essere stati settati per vedere quale periferica
; ha generato l'interrupt. Una volta processato l'interrupt, occorre anche assicurarsi
; di azzerare tale flag, altrimenti la CPU entrerebbe in interrupt continuamente

	
test_timer0:
	btfss	INTCON,INTCON_T0IF_POSITION	    ; Se il bit T0IF = 1 (c'e' stato un interrupt del timer), salta istruzione seguente
	goto 	test_button			    ; Salta a test successivo
	btfss	INTCON,INTCON_T0IE_POSITION	    ; Controlla anche che l'interrupt fosse effettivamente abilitato
	goto	test_button
; Avvenuto interrupt timer0: termine debouncing
	bsf	INTCON,INTCON_RBIE_POSITION	    ; Riabilita interrupt porta B
	bcf	INTCON,INTCON_T0IF_POSITION	    ; Azzera flag interrupt timer
	bcf	INTCON,INTCON_T0IE_POSITION	    ; Disabilita interrupt timer, verr� abililato quando serve in test_button
; Se il debouncing e' seguito al rilascio del pulsante,
; Segnala al programma principale che puo' andare in sleep.
	bsf	canSleep, 0	; altrimenti abilita sleep
	goto	irq_end		; vai a fine routine di interrupt
			
	
test_button:
; Testa evento port-change di PORTB (RBIF + RBIE) altrimenti testo l'interrupt TIMER 1
	btfss	INTCON,INTCON_RBIF_POSITION	    ; Test flag interrupt PORTB
	goto	test_timer1
	btfss	INTCON,INTCON_RBIE_POSITION	    ; Test se l'interrupt PORTB � abilitato
	goto	test_timer1
			
; Avvenuto evento port-change di PORTB
	banksel	PORTB
	movf	PORTB, w			    ; Legge PORTB eliminando condizione di mismatch
	bcf	INTCON,INTCON_RBIF_POSITION	    ; Azzera flag evento interrupt
; XOR di stato precedente porta (portBPrev) con stato attuale (PORTB).
; In questo modo i bit diversi hanno risultato 1
	xorwf	portBPrev, w			    ; XOR di W (che conteneva PORTB) con portBPrev ( 1 in corrispondenza dei bit che hanno cambiato stato)
	andlw	0x01				    ; Mi tester� solo il pulsante sul bit 0
	btfsc	STATUS,STATUS_Z_POSITION	    ; se il risultato non e' nullo (Z=0): un pulsante ha cambiato stato
	goto	button_end			    ; altrimenti non fare niente
			
; Il pulsante e' stato premuto o rilasciato, iniziare il conteggio di debouncing
	bcf	INTCON,INTCON_RBIE_POSITION	    ; Disabilita interrupt PORTB per durata debouncing
	banksel	TMR0
	movlw	tmr_10ms			    ; Carica valore iniziale per il contatore di timer0
	movwf	TMR0
	bcf	INTCON,INTCON_T0IF_POSITION	    ; Azzera evento interrupt di timer0
	bsf	INTCON,INTCON_T0IE_POSITION	    ; Abilita interrupt di timer0
			
; Vieta lo sleep fino a termine debouncing
	bcf	canSleep, 0	
			
			
; Testa se il pulsante � stato premuto o rilasciato
	btfss	portBPrev, 0			    ; Se lo stato precedente era 1 -> premuto
; Dal codice precedente abbiamo capito se c'� stato un cambiamento di stato su RB0,
; Quindi, se c'� stato un cambiamento di stato e lo stato precedente era 1, significa che 
; RB0 � passato da 1(rilasciato) -> 0(premuto) e cio� che il pulsante � stato premuto!
; N.B. -> Se il bit 0 della porta B � = 0 (pulsante premuto)
;      -> Se il bit 0 della porta B � = 1 (pulsante rilasciato)
 
	goto	button_end			    ; Altrimenti non fare niente
; Pulsante premuto: incremento contatore
	pagesel INCREMENTO_CONTATORE
	call	INCREMENTO_CONTATORE
			
button_end:
; Salva nuovo stato di PORTB su portBPrev
	banksel PORTB
	movf	PORTB, w
	movwf	portBPrev
	goto	irq_end				    ; Va a fine interrupt
; Eventuali altri eventi di interrupt
; Fine codice interrupt
    
test_timer1:
; Testa evento overflow timer1 (TMR1IF + TMR1IE) altrimenti testo l'interrupt UART
	banksel	PIR1
	btfss	PIR1,PIR1_TMR1IF_POSITION
	goto	test_uart
	banksel	PIE1
	btfss	PIE1,PIE1_TMR1IE_POSITION
	goto	test_uart		    
; Avvenuto interrupt timer1: toggle LED1
	call print_eusart			    ; chiamata alla routine di trasmissione del numero di pressioni	
	movlw	0x01	
	banksel	PORTD				    ; cambio di stato del led 1 ogni volta che c'� overflow del timer
	xorwf	PORTD,f				    ; 0x01 XOR PORTD -> 1 XOR 0(led off) = 1(led on); 1 XOR 1(led on) = 0(led off)
			
; Ricarica contatore timer1
	pagesel	reload_timer1
	call	reload_timer1
; Fine evento timer1
	goto	irq_end

; Eventuali altri eventi di interrupt

					
test_uart:	    
	banksel PIE1
	btfss PIE1, 4				    ; controllo il bit TXIE (se l'interrupt della USART � abilitato)
	goto irq_end
	banksel PIR1
	btfss PIR1, 4				    ; controllo il bit TXIF (se � avvenuto un interrupt)
	goto irq_end
; se c'� stato l'interrupt	    
; la Seriale � pronta per la trasmissione di un nuovo byte
	movf usart_counter, w			    ; carico il contatore dei byte da trasmettere in W
	btfsc STATUS, 2				    ; controllo il bit Z del registro STATUS per vedere se il risultato dell'operazione � zero
; caso Byte da inviare finiti (risultato = 0) Z = 1
	goto usart_tx_end			    ; termino la trasmissione
; Byte da inviare (risultato !=0)Z = 0
	movf INDF, w				    ; carico il contenuto dell'indirizzo memorizzato nell'FSR (buffer_usart + incremento) in w			
	banksel TXREG				    
	movwf TXREG				    ; carico il contenuto di w nel registro di trasmissione
	incf FSR				    ; incremento l'indirizzo puntato da FSR
	decf usart_counter, f			    ; decremento il contatore di byte da trasmettere
			
			
wait_usart:		
	banksel TXSTA
	btfss TXSTA, 1				    ; controllo il bit TMRT
						    
	goto wait_usart				    ; attendo che il buffer di trasmissione si svuoti (se bit = 0)
	goto irq_end
			
usart_tx_end:
; Caso dati da trasmettere terminati 
	banksel PIE1				    
	bcf PIE1, 4				    ; disabilito l'interrupt della seriale
	bsf canSleep, 0				    ; il micro pu� andare in sleep
	goto irq_end
						
; Ripristino stato registri CPU precedente all'interruzione:
irq_end:		
	movf pclath_temp,w			    ; Copia pclath_temp in w
	movwf PCLATH				    ; Copia w in PCLATH
	swapf status_temp,w			    ; Inverte i nibble di status_temp salvando il risultato in w
; Anche in questo caso serve a non alterare STATUS stesso
	movwf	STATUS				    ; copia w (che contiene lo STATUS originale ripristinato dopo 2 inversioni) in STATUS
; Per ripristinare w senza alterare STATUS appena ripristinato, si utilizza sempre swapf
	swapf	w_temp,f			    ; prima inversione di w_temp, risultato su se stesso
	swapf	w_temp,w			    ; seconda inversione di w_temp, risultato in w (w contiene il valore precedente all'interrupt)

	retfie					    ; uscita da interrupt e ritorno al punto in cui il programma era stato interrotto

END resetVec