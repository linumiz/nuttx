	.macro btv_vector_class class
	.p2align 5
	svlcx 
	mov %d4, \class
	mov %d5, %d15
	fcall tricore_trap_handler
	rslcx
	rfe
	.endm

    .macro csa_to_addr csa, addr
    mov     %d15, 0
    insert  %d15, %d15, \csa, 6, 16
    extr.u  %d14, \csa, 16, 4
    insert  %d15, %d15, %d14, 28, 4
    mov.a   \addr, %d15
    .endm

.macro aurix_prot_own prot
   ld.w     %d15, [\prot]
   mov      %d14, lo:(0xC000 | ((CONFIG_AURIX_CORE_ID*2) << 8))
   insert   %d15, %d15, %d14, 16, 16
   st.w     [\prot], %d15
   dsync
.endm

.macro aurix_prot_set_state prot state
   ld.w     %d15, [\prot]
   mov      %d14, lo:(0x8 | \state)
   insert   %d15, %d15, %d14, 0, 16
   st.w     [\prot], %d15
   dsync
.endm

.macro aurix_prot_try_config_state prot err
   ld.w     %d15, [\prot]
   extr.u   %d15, %d15, 0, 3
   jlt.u    %d15, 3, is_config
   jge.u    %d15, 6, \err
   aurix_prot_set_state \prot 1
is_config:
.endm

.macro aurix_prot_try_own prot err
   ld.w     %d15, [\prot]
   jz.t     %d15, 30, set_own
   extr.u   %d15, %d15, 24, 6
   mov      %d14, (CONFIG_AURIX_CORE_ID*2)
   jne      %d15, %d14, \err
   j        is_owned
set_own:
   aurix_prot_own \prot
is_owned:
.endm
