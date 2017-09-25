/*
								+----------------------------------+
								|                                  |
								| ***   Mersenne twister rng   *** |
								|                                  |
				http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/MT2002/CODES/mt19937ar.c
								|                                  |
								|            Mersenne.h            |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __MERSENNE_INCLUDED
#define __MERSENNE_INCLUDED

/**
 *	@file Mersenne.h
 *	@author -tHE SWINe-
 *	@date 2002
 *	@brief Mersenne twister random number generator
 *
 *	This implementation is based on sources from:
 *		http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/MT2002/CODES/mt19937ar.c
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
 *	@date 2009-12-18
 *
 *	changed const semantic on CMersenneTwister, random number can now be generated
 *	using const CMersenneTwister (before const CMersenneTwister was useless).
 *
 *	@date 2012-06-19
 *
 *	Moved multiple inclusion guard before file documentation comment.
 *
 */

#include "Integer.h"

/**
 *	@brief Mersenne twister random number generator
 */
class CMersenneTwister {
private:
	/**
	 *	@brief internal constants, stored as enum
	 */
	enum {
		N = 624,
		M = 397,
		MATRIX_A = 0x9908b0dfUL,  /* constant vector a */
		UPPER_MASK = 0x80000000UL, /* most significant w-r bits */
		LOWER_MASK = 0x7fffffffUL /* least significant r bits */
	};

	mutable unsigned long m_n_state[N]; /* the array for the state vector  */
	mutable int m_n_init_index; /* m_n_init_index==N+1 means m_n_state[N] is not initialized */

public:
	/**
	 *	@brief default constructor
	 */
	CMersenneTwister();

	/**
	 *	@brief initializes m_n_state[N] with a seed
	 *	@param[in] s is seed value
	 */
	void init_genrand(unsigned long s);

	/**
	 *	@brief initialize by an array
	 *
	 *	@param[in] init_key is the array for initializing keys
	 *	@param[in] key_length is its length
	 *
	 *	@note: slightly changed for C++, 2004/2/26
	 */
	void init_by_array(unsigned long *init_key, int key_length);

	/**
	 *	@brief generates a random number on [0, 0xffffffff] interval
	 *	@return Returns random number in interval [0, 0xffffffff].
	 */
	unsigned long genrand_int32() const;

	/**
	 *	@brief generates a random number on [0, 0x7fffffff] interval
	 *	@return Returns random number in interval [0, 0x7fffffff].
	 */
	long genrand_int31() const;

	/**
	 *	@brief generates a random number on [0, 1] real interval
	 *	@return Returns floating-point random number in interval [0, 1].
	 */
	double genrand_real1() const;

	/**
	 *	@brief generates a random number on [0, 1) real interval
	 *	@return Returns floating-point random number in interval [0, 1).
	 */
	double genrand_real2() const;

	/**
	 *	@brief generates a random number on (0, 1) real interval
	 *	@return Returns floating-point random number in interval (0, 1).
	 */
	double genrand_real3() const;

	/**
	 *	@brief generates a random number on [0, 1) with 53-bit resolution
	 *	@return Returns floating-point random number in interval [0, 1).
	 */
	double genrand_res53() const;

	/**
	 *	@brief generates a random number on [0, 1) with 24-bit resolution
	 *	@return Returns floating-point random number in interval [0, 1).
	 */
	float genrand_res24() const;

	/**
	 *	@brief generates a random number on [0, 1] with 53-bit resolution
	 *	@return Returns floating-point random number in interval [0, 1].
	 */
	double genrand_res53_inclusive() const;

	/**
	 *	@brief generates a random number on [0, 1] with 24-bit resolution
	 *	@return Returns floating-point random number in interval [0, 1].
	 */
	float genrand_res24_inclusive() const;
};

#endif // !__MERSENNE_INCLUDED
