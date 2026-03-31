//----------------------------------------------------------------------------
// prng.h
// =======
//
//----------------------------------------------------------------------------
//
#ifndef __prng_H
#define __prng_H

uint8_t get_random_bit(void);
uint8_t get_random_byte(void);
uint8_t get_random_byte_in_range(uint8_t low, uint8_t high);
void init_prng(uint8_t s1, uint8_t s2, uint8_t s3);

#endif /* __prng_H */