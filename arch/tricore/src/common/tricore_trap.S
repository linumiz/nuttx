	.section .traptab, "ax", %progbits
	.global	btv_vector
	.type   btv_vector, %object
	.p2align 5

btv_vector:
	.set N, 8
	.set I, 0

	.rept N
1:
	svlcx
	mov %d4, I
	mov %d5, %d15
	fcall tricore_trap_handler
	rslcx
	rfe

	.space (32 - (.-1b))
	.set I, I + 1
	.endr

tricore_trap_handler:
	fret
