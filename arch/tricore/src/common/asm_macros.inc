	.macro btv_vector_class class
	.p2align 5
	svlcx 
	mov %d4, \class
	mov %d5, %d15
	fcall tricore_trap_handler
	rslcx
	rfe
	.endm
