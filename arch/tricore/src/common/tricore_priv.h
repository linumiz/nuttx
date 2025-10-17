#ifndef TRICORE_PRIV_H
#define TRICORE_PRIV_H

#define getreg8(a)     (*(volatile uint8_t *)(a))
#define putreg8(v,a)   (*(volatile uint8_t *)(a) = (v))
#define getreg16(a)    (*(volatile uint16_t *)(a))
#define putreg16(v,a)  (*(volatile uint16_t *)(a) = (v))
#define getreg32(a)    (*(volatile uint32_t *)(a))
#define putreg32(v,a)  (*(volatile uint32_t *)(a) = (v))
#define getreg64(a)    (*(volatile uint64_t *)(a))
#define putreg64(v,a)  (*(volatile uint64_t *)(a) = (v))

/* Non-atomic, but more effective modification of registers */

#define modreg8(v,m,a)  putreg8((getreg8(a) & ~(m)) | ((v) & (m)), (a))
#define modreg16(v,m,a) putreg16((getreg16(a) & ~(m)) | ((v) & (m)), (a))
#define modreg32(v,m,a) putreg32((getreg32(a) & ~(m)) | ((v) & (m)), (a))
#define modreg64(v,m,a) putreg64((getreg64(a) & ~(m)) | ((v) & (m)), (a))

#endif
