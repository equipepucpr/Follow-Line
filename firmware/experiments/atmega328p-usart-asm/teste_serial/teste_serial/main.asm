.macro	store
	ldi		R16,@1
	sts		@0,R16
.endm

	; Config Baudrate 4800bps
	store	UBRR0L,0xCF
	store	UBRR0H,0x00

	; Config USART0
	store	UCSR0A,0b00100000
	store	UCSR0B,0b00011000
	store	UCSR0C,0b00000110

	ldi R17,72
	call tx_R17
	ldi R17,73
	call tx_R17
	ldi R17,74
	call tx_R17
	ldi R17,75
	call tx_R17

	store	UDR0,72
loop:
	call rx_R17
	inc R17
	call tx_R17

	
	jmp		loop

tx_R17:
	lds R16, UCSR0A
	sbrs R16,5
	jmp tx_R17
	sts UDR0,R17
	ret

rx_r17
	lds R16, UCSR0A
	sbrs R16, 7
	jmp rx_R17
	lds R17, UDR0

	ret