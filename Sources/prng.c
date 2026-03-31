//----------------------------------------------------------------------------
//                  Robokid
//----------------------------------------------------------------------------
// prng.c : generate pseudorandom values
// ======
//
// Description
//       X ABC Algorithm Random Number Generator for 8-Bit Devices
//       https://www.electro-tech-online.com/threads/ultra-fast-pseudorandom-number-generator-for-8-bit.124249/
//       Not safe for cryptographic use!
//
// Author                Date          Comment
//----------------------------------------------------------------------------
// Jim Herd    03/02/26      Simple random number generator based on above code
//             04/02/26      Checked to give reasonable results for this type of system 
//----------------------------------------------------------------------------

#include "global.h"

static uint8_t a, b, c, x;

//----------------------------------------------------------------------------
// get_random_byte : get a random 8-bit value
// ===============
//
// return 8-bit pseudorandom number
//
uint8_t get_random_byte(void) 
{
  x++;
  a = (a ^ c) ^ x;
  b = b + a;
  c = (c + ((b >> 1) | (b << 7))) ^ a;
  return c; 
}

//----------------------------------------------------------------------------
// get_random_byte_in_range : get random byte value in a defined range
// ========================
//
// Google Gemini generated code
//
uint8_t get_random_byte_in_range(uint8_t low, uint8_t high)
{
	uint8_t range = high - low + 1; // +1 to give inclusive range
//
// Check for edge cases
//
	if (range <= 0) {
		return low;
	}
//
// Right shift by 8 equivalent to division by 256
// No need for shift as upper 8-bits of multiply are held in X-register
//
	return low + (uint8_t)(((uint16_t)range * (get_random_byte()) >> 8));
}

//----------------------------------------------------------------------------
// get_random_bit : random 1-bit value
// ==============
//
// return 1-bit pseudorandom value */

uint8_t get_random_bit(void)
{
	return (get_random_byte() & 0x01);
}


//----------------------------------------------------------------------------
// init_prng : initialise pseudorandom number generator
// =========
//
// Add entropy into the state
//

void init_prng(uint8_t s1, uint8_t s2, uint8_t s3) 
{
  /* XOR new entropy into key state */
  a ^= s1;
  b ^= s2;
  c ^= s3;
  get_random_byte();
}




//----------------------------------------------------------------------------
// **** Old random number functions ***
//----------------------------------------------------------------------------
// get_random_bit : random bit generator
// ==============
//
// Notes
//      Uses background counter to generate some randomness
//
// uint8_t get_random_bit(void) {
//
//    return  (tick_count_8 & 0x01);
// }

//----------------------------------------------------------------------------
// get_random_byte : get a random 8-bit value
// ===============
//
// Notes
//      Use low 8-bits of background 8mS tick counter
//
// uint8_t get_random_byte(void) {
//
//    return  tick_count_8;
// }


