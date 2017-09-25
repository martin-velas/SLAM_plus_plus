/*
								+----------------------------------+
								|                                  |
								| ***   Mersenne twister rng   *** |
								|                                  |
				http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/MT2002/CODES/mt19937ar.c
								|                                  |
								|           Mersenne.cpp           |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file Mersenne.cpp
 *	@author -tHE SWINe-
 *	@date 2002
 *	@brief Mersenne twister random number generator
 *
 *	@date 2007-11-14
 *
 *	enhanced linux compatibility by adding unix integer types
 *
 *	@date 2008-03-04
 *
 *	now using Integer.h header
 *
 *	@date 2009-05-04
 *
 *	fixed mixed windows / linux line endings
 *
 */

#include "NewFix.h"

#include "CallStack.h"
#include <string.h>
#include "Mersenne.h"

#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(for) && _MSC_VER <= 1200
#define for if(0) {} else for
#endif // _MSC_VER && !__MWERKS__ && !for && _MSC_VER <= 1200
// msvc 'for' scoping hack

CMersenneTwister::CMersenneTwister()
{
	m_n_init_index = N + 1;
}

/* initializes m_n_state[N] with a seed */
void CMersenneTwister::init_genrand(unsigned long s)
{
	m_n_state[0]= s & 0xffffffffUL;
	for (m_n_init_index=1; m_n_init_index<N; m_n_init_index++) {
		m_n_state[m_n_init_index] = 
		(1812433253UL * (m_n_state[m_n_init_index-1] ^ (m_n_state[m_n_init_index-1] >> 30)) + m_n_init_index); 
		/* See Knuth TAOCP Vol2. 3rd Ed. P.106 for multiplier. */
		/* In the previous versions, MSBs of the seed affect   */
		/* only MSBs of the array m_n_state[].                        */
		/* 2002/01/09 modified by Makoto Matsumoto             */
		m_n_state[m_n_init_index] &= 0xffffffffUL;
		/* for >32 bit machines */
	}
}

/* initialize by an array with array-length */
/* init_key is the array for initializing keys */
/* key_length is its length */
/* slight change for C++, 2004/2/26 */
void CMersenneTwister::init_by_array(unsigned long *init_key, int key_length)
{
	int i, j, k;
	init_genrand(19650218UL);
	i=1; j=0;
	k = (N>key_length ? N : key_length);
	for (; k; k--) {
		m_n_state[i] = (m_n_state[i] ^ ((m_n_state[i-1] ^ (m_n_state[i-1] >> 30)) * 1664525UL))
		  + init_key[j] + j; /* non linear */
		m_n_state[i] &= 0xffffffffUL; /* for WORDSIZE > 32 machines */
		i++; j++;
		if (i>=N) { m_n_state[0] = m_n_state[N-1]; i=1; }
		if (j>=key_length) j=0;
	}
	for (k=N-1; k; k--) {
		m_n_state[i] = (m_n_state[i] ^ ((m_n_state[i-1] ^ (m_n_state[i-1] >> 30)) * 1566083941UL))
		  - i; /* non linear */
		m_n_state[i] &= 0xffffffffUL; /* for WORDSIZE > 32 machines */
		i++;
		if (i>=N) { m_n_state[0] = m_n_state[N-1]; i=1; }
	}

	m_n_state[0] = 0x80000000UL; /* MSB is 1; assuring non-zero initial array */ 
}

/* generates a random number on [0,0xffffffff]-interval */
unsigned long CMersenneTwister::genrand_int32() const
{
	unsigned long y;
	static unsigned long mag01[2]={0x0UL, MATRIX_A};
	/* mag01[x] = x * MATRIX_A  for x=0,1 */

	if (m_n_init_index >= N) { /* generate N words at one time */
		int kk;

		if (m_n_init_index == N+1)   /* if init_genrand() has not been called, */
			((CMersenneTwister*)this)->init_genrand(5489UL); /* a default initial seed is used */

		for (kk=0;kk<N-M;kk++) {
			y = (m_n_state[kk]&UPPER_MASK)|(m_n_state[kk+1]&LOWER_MASK);
			m_n_state[kk] = m_n_state[kk+M] ^ (y >> 1) ^ mag01[y & 0x1UL];
		}
		for (;kk<N-1;kk++) {
			y = (m_n_state[kk]&UPPER_MASK)|(m_n_state[kk+1]&LOWER_MASK);
			m_n_state[kk] = m_n_state[kk+(M-N)] ^ (y >> 1) ^ mag01[y & 0x1UL];
		}
		y = (m_n_state[N-1]&UPPER_MASK)|(m_n_state[0]&LOWER_MASK);
		m_n_state[N-1] = m_n_state[M-1] ^ (y >> 1) ^ mag01[y & 0x1UL];

		m_n_init_index = 0;
	}

	y = m_n_state[m_n_init_index++];

	/* Tempering */
	y ^= (y >> 11);
	y ^= (y << 7) & 0x9d2c5680UL;
	y ^= (y << 15) & 0xefc60000UL;
	y ^= (y >> 18);

	return y;
}

/* generates a random number on [0,0x7fffffff]-interval */
long CMersenneTwister::genrand_int31() const
{
	return (long)(genrand_int32() >> 1);
}

/* generates a random number on [0,1]-real-interval */
double CMersenneTwister::genrand_real1() const
{
	return genrand_int32() * (1.0 / 4294967295.0); 
	/* divided by 2^32-1 */ 
}

/* generates a random number on [0,1)-real-interval */
double CMersenneTwister::genrand_real2() const
{
	return genrand_int32() * (1.0 / 4294967296.0); 
	/* divided by 2^32 */
}

/* generates a random number on (0,1)-real-interval */
double CMersenneTwister::genrand_real3() const
{
	return (((double)genrand_int32()) + 0.5) * (1.0 / 4294967296.0); 
	/* divided by 2^32 */
}

/* generates a random number on [0,1) with 53-bit resolution */
double CMersenneTwister::genrand_res53() const
{
	unsigned long a = genrand_int32() >> 5, b = genrand_int32() >> 6;
	return (a * 67108864.0 + b) * (1.0 / 9007199254740992.0);
	// 67108864 = 2^(32 - 6)
	// 9007199254740992.0 = 2^53
}

/* generates a random number on [0,1] with 53-bit resolution */
double CMersenneTwister::genrand_res53_inclusive() const
{
	uint64_t a = ((uint64_t(genrand_int32()) << 32) | genrand_int32()) >> 10;
	while(a > (uint64_t(1) << 53)) // will loop less than twice on average
		a = ((uint64_t(genrand_int32()) << 32) | genrand_int32()) >> 10; // can generate again, the draws are independent
	_ASSERTE(a >= 0 && a <= (uint64_t(1) << 53));
	double f = double(int64_t(a)) * (1.0 / 9007199254740992.0); // msvc 6.0 doesn't implement conversion from uint64_t to double
	_ASSERTE(f >= 0 && f <= 1);
	return f;
	// 67108864 = 2^(32 - 6)
	// 9007199254740992.0 = 2^53
}

/* generates a random number on [0,1) with 24-bit resolution */
float CMersenneTwister::genrand_res24() const
{
	unsigned long a = genrand_int32() >> 8;
	return a * (1.0f / 16777216.0f);
	// 16777216 = 2^24
}

/* generates a random number on [0,1] with 24-bit resolution */
float CMersenneTwister::genrand_res24_inclusive() const
{
	unsigned long a = genrand_int32() >> 7;
	while(a > (1U << 24)) // will loop less than twice on average
		a = genrand_int32() >> 7; // can generate again, the draws are independent
	return a * (1.0f / 16777216.0f);
	// 16777216 = 2^24
}

/*
 *		~CMersenneTwister
 */
