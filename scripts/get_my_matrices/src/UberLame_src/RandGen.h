/*
								+----------------------------------+
								|                                  |
								| ***  Random generator model  *** |
								|                                  |
								|   Copyright © -tHE SWINe- 2007   |
								|                                  |
								|            RandGen.h             |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __RANDOM_GENERATORS_INCLUDED
#define __RANDOM_GENERATORS_INCLUDED

/**
 *	@file RandGen.h
 *	@brief generic random generator models
 *	@date 2007
 *	@author -tHE SWINe-
 *
 *	@date 2007-10-17
 *
 *	added CMTGenerator::CMTGenerator(unsigned int n_seed = 0) constructor for greater convenience
 *
 */

#include "Integer.h"
#include "Mersenne.h" // Mersenne Twister
#include <stdlib.h> // srand(), rand()
#include <math.h> // floor()

/**
 *	@brief generic minimal random number generator class (intended
 *		for experiments with random number generation methods)
 *
 *	@tparam b_use_virtual_interface is common interface flag
 */
template <bool b_use_virtual_interface = true>
class CRandomGeneratorModel {
public:
	/**
	 *	@brief sets a new seed and restarts the generator with this seed
	 *	@param[in] n_seed is random generator seed
	 */
	virtual void Seed(unsigned int n_seed) = 0;

	/**
	 *	@brief generates a 32-bit unsigned random number
	 *	@return Returns the generated random number.
	 */
	virtual uint32_t n_Rand32() = 0;

	/**
	 *	@brief generates a random number on [0, 1] real interval
	 *	@return Returns floating-point random number in range 0 to 1 inclusive.
	 */
	virtual double f_Rand() = 0;

	/**
	 *	@brief generates a random number on [-1, 1] real interval
	 *	@return Returns floating-point random number in range -1 to 1 inclusive.
	 */
	virtual double f_SignedRand() = 0;

	/**
	 *	@brief generates a random number on [0, 1) real interval
	 *	@return Returns floating-point random number in range 0 to 1 non-inclusive on the right.
	 *	@note This will likely not stay in the correct range if rounded to a float.
	 */
	virtual double f_Rand_LeftInclusive() = 0;

	/**
	 *	@brief generates a random number on (0, 1) real interval
	 *	@return Returns floating-point random number in range 0 to 1 non-inclusive.
	 *	@note This will likely not stay in the correct range if rounded to a float.
	 */
	virtual double f_Rand_NonInclusive() = 0;

	/**
	 *	@brief generates a random number on (-1, 1) real interval
	 *	@return Returns floating-point random number in range -1 to 1 non-inclusive.
	 *	@note This will likely not stay in the correct range if rounded to a float.
	 */
	virtual double f_SignedRand_NonInclusive() = 0;
};

/**
 *	@brief generic minimal random number generator class (specialization for no common interface)
 */
template <>
class CRandomGeneratorModel<false> {};

/**
 *	@brief random number generator traits
 *	@tparam CGenerator is random number genetator type
 */
template <class CGenerator>
class CRandomGeneratorTraits {
public:
	/**
	 *	@brief random number generator traits, stored as enum
	 */
	enum {
		b_seedable = CGenerator::b_seedable, /**< @brief seed not ignored flag */
		b_64bit_rand = CGenerator::b_64bit_rand, /**< @brief 64-bit random number capability flag */
		b_crypto_safe = CGenerator::b_crypto_safe, /**< @brief cryptographically safe flag */
		b_fast_floats = CGenerator::b_fast_floats /**< @brief fast (possibly lower quality) floating point random number generation flag */
	};
};

/**
 *	@brief utility to generate 64-bit random numbers using any generator
 *	@tparam CRandomNumberGenerator is random number generator model
 */
template <class CRandomNumberGenerator>
class CRand64Caller {
public:
	/**
	 *	@brief intermediates stored as enum
	 */
	enum {
		b_has_64 = CRandomGeneratorTraits<CRandomNumberGenerator>::b_64bit_rand /**< @brief 64-bit rand support */
	};

protected:
	template <const bool b_has64 MSVC_OMMIT_ARG(class GppDummy)>
	class CCall {
	public:
		static uint64_t n_Rand64(CRandomNumberGenerator &r_gen)
		{
			return r_gen.n_Rand64();
		}
	};

	template <MSVC_OMMIT(class GppDummy)>
	class CCall<false MSVC_OMMIT_ARG(GppDummy)> {
	public:
		static uint64_t n_Rand64(CRandomNumberGenerator &r_gen)
		{
			return (uint64_t(r_gen.n_Rand32()) << 32) | r_gen.n_Rand32();
		}
	};

public:
	/**
	 *	@brief generates a 64-bit unsigned random number
	 *	@param[in,out] r_gen is reference to the generator state
	 *	@return Returns the generated random number.
	 */
	static uint64_t n_Rand64(CRandomNumberGenerator &r_gen)
	{
		return CCall<b_has_64 MSVC_OMMIT_ARG(void)>::n_Rand64(r_gen);
	}
};

/**
 *	@brief correct implementation of 53-bit floating point random number generation
 *
 *	@tparam CDerived is type name of the resulting generator, which should inherit from this class
 *	@tparam b_use_virtual_interface is common interface flag
 *
 *	@todo This uses 64-bit numbers but only really needs 53 or 54 bits, which can save a few rounds
 *		for some of the shorter generators. Could add a functions to generate 54-bit numbers.
 */
template <class CDerived, bool b_use_virtual_interface>
class CRandomGeneratorFloatImpl : public CRandomGeneratorModel<b_use_virtual_interface> {
public:
	/**
	 *	@copydoc CRandomGeneratorModel::f_Rand()
	 */
	double f_Rand()
	{
		int64_t n_rand = CRand64Caller<CDerived>::n_Rand64(*static_cast<CDerived*>(this)) >> 10; // msvc 6.0 does not implement conversion from uint64_t to double
		while(n_rand > (int64_t(1) << 53)) // generates 54-bit numbers, but we're interested only in 1 + 53 zeroes or below
			n_rand = CRand64Caller<CDerived>::n_Rand64(*static_cast<CDerived*>(this)) >> 10; // can try again, the samples are independent
		return double(n_rand) * (1.0 / 9007199254740992.0); // n_rand is [0, 9007199254740992] inclusive
		// more uniformly distributed floats
		// the division is precise, 9007199254740992.0 is a power of two

/*#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER <= 1200
		return (int64_t)(n_Rand64() & INT64_MAX) / double(INT64_MAX); // msvc 6.0 does not implement conversion from uint64_t to double
#else // _MSC_VER) && !__MWERKS__ && _MSC_VER <= 1200
		return n_Rand64() / double(UINT64_MAX);
#endif // _MSC_VER) && !__MWERKS__ && _MSC_VER <= 1200*/
		//return double(n_Rand64() * 5.42101086242752217e-20);
	}

	/**
	 *	@copydoc CRandomGeneratorModel::f_SignedRand()
	 */
	double f_SignedRand()
	{
		int64_t n_rand = CRand64Caller<CDerived>::n_Rand64(*static_cast<CDerived*>(this)) >> 9; // msvc 6.0 does not implement conversion from uint64_t to double
		while(n_rand > (int64_t(1) << 54)) // generates 54-bit numbers, but we're interested only in 1 + 53 zeroes or below
			n_rand = CRand64Caller<CDerived>::n_Rand64(*static_cast<CDerived*>(this)) >> 9; // can try again, the samples are independent
		if(n_rand & 1) // avoid remembering the old value, save a register
			n_rand = -(n_rand >> 1);
		else
			n_rand >>= 1;
		// use it as sign; note that this bit is drawn independently of the number so it does
		// not increase chances of yielding zero (rather it splits zero's chances between zero and negative zero)

		return double(n_rand) * (1.0 / 9007199254740992.0); // n_rand is [0, 9007199254740992] inclusive
		// more uniformly distributed floats
		// the division is precise, 9007199254740992.0 is a power of two

/*#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER <= 1200
		return (int64_t)(n_Rand64() & INT64_MAX) / double(INT64_MAX); // msvc 6.0 does not implement conversion from uint64_t to double
#else // _MSC_VER) && !__MWERKS__ && _MSC_VER <= 1200
		return n_Rand64() / double(UINT64_MAX);
#endif // _MSC_VER) && !__MWERKS__ && _MSC_VER <= 1200*/
		//return double(n_Rand64() * 5.42101086242752217e-20);
	}

	/**
	 *	@copydoc CRandomGeneratorModel::f_Rand_LeftInclusive()
	 */
	double f_Rand_LeftInclusive()
	{
		int64_t n_rand = CRand64Caller<CDerived>::n_Rand64(*static_cast<CDerived*>(this)) >> 11; // msvc 6.0 does not implement conversion from uint64_t to double
		return double(n_rand) * (1.0 / 9007199254740992.0); // n_rand is [0, 9007199254740992) right-exclusive
		// more uniformly distributed floats
		// the division is precise, 9007199254740992.0 is a power of two

		//return f_Rand() * (1 - 1 / double(UINT64_MAX)); // this is not good, 1 / UINT64_MAX will be below epsilon, will return the same numbers as f_Rand()
		//return double(n_Rand32() / (double(UINT32_MAX) + 1)); // this throws away too much precision
		/*const uint64_t n_max = (uint64_t(1) << 52) - 1; // use 52 bits
		_ASSERTE(1.0 + 1.0 / double(n_max + 1) != 1.0);
		return (n_Rand64() & n_max) / double(n_max + 1);*/
	}

	/**
	 *	@copydoc CRandomGeneratorModel::f_Rand_NonInclusive()
	 */
	double f_Rand_NonInclusive()
	{
		int64_t n_rand = CRand64Caller<CDerived>::n_Rand64(*static_cast<CDerived*>(this)) >> 11; // msvc 6.0 does not implement conversion from uint64_t to double
		while(!n_rand) // don't want to generate zero
			n_rand = CRand64Caller<CDerived>::n_Rand64(*static_cast<CDerived*>(this)) >> 11; // can try again, the samples are independent
		return double(n_rand) * (1.0 / 9007199254740992.0); // n_rand is (0, 9007199254740992) exclusive
		// more uniformly distributed floats
		// the division is precise, 9007199254740992.0 is a power of two

		//return f_Rand() * (1 - 2 / double(UINT64_MAX)) + 1 / double(UINT64_MAX); // this is not good, 1 / UINT64_MAX will be below epsilon, will return the same numbers as f_Rand()
		//return double((double(n_Rand32()) + 1) / (double(UINT32_MAX) + 2)); // this throws away too much precision
		/*const uint64_t n_max = (uint64_t(1) << 52) - 1; // use 52 bits
		_ASSERTE(1.0 + 1.0 / double(n_max + 1) != 1.0);
		return ((n_Rand64() & n_max) + 1) / double(n_max + 2);*/
	}

	/**
	 *	@copydoc CRandomGeneratorModel::f_SignedRand_NonInclusive()
	 */
	double f_SignedRand_NonInclusive()
	{
		uint64_t n_orig = CRand64Caller<CDerived>::n_Rand64(*static_cast<CDerived*>(this));
		int64_t n_rand = n_orig >> 11; // msvc 6.0 does not implement conversion from uint64_t to double
		if(n_orig & 512) // don't use the lowest bit in case this is the lame CCongruentialGenerator
			n_rand = -n_rand;
		// use it as sign; note that this bit is drawn independently of the number so it does
		// not increase chances of yielding zero (rather it splits zero's chances between zero and negative zero)
		return double(n_rand) * (1.0 / 9007199254740992.0); // n_rand is (0, 9007199254740992) exclusive
		// more uniformly distributed floats
		// the division is precise, 9007199254740992.0 is a power of two
	}
};

/**
 *	@brief Mersenne Twister-based random number generator
 *	@tparam b_use_virtual_interface is common interface flag (default true)
 */
template <bool b_use_virtual_interface = true>
class CMTGenerator : public CRandomGeneratorModel<b_use_virtual_interface> {
public:
	/**
	 *	@brief random number generator traits, stored as enum
	 */
	enum {
		b_seedable = true, /**< @brief seed not ignored flag */
		b_64bit_rand = false, /**< @brief 64-bit random number capability flag */
		b_crypto_safe = false, /**< @brief cryptographically safe flag */
		b_fast_floats = false /**< @brief fast (possibly lower quality) floating point random number generation flag */
	};

protected:
	CMersenneTwister m_twister; /**< @brief Mersenne Twister instance */

public:
	/**
	 *	@brief default constructor; has no effect
	 */
	inline CMTGenerator()
	{}

	/**
	 *	@brief constructor; initializes the random generator
	 */
	inline CMTGenerator(unsigned int n_seed)
	{
		Seed(n_seed);
	}

	/**
	 *	@copydoc CRandomGeneratorModel::Seed()
	 */
	void Seed(unsigned int n_seed)
	{
		m_twister.init_genrand(n_seed);
	}

	/**
	 *	@brief generates a 32-bit unsigned random number
	 *	@return Returns the generated random number.
	 */
	uint32_t n_Rand32()
	{
		return m_twister.genrand_int32();
	}

	/**
	 *	@copydoc CRandomGeneratorModel::f_Rand()
	 */
	double f_Rand()
	{
		return m_twister.genrand_res53_inclusive(); // uniformly distributed
	}

	/**
	 *	@copydoc CRandomGeneratorModel::f_SignedRand()
	 */
	double f_SignedRand()
	{
		uint64_t a = ((uint64_t(n_Rand32()) << 32) | n_Rand32()) >> 9;
		while(a > (uint64_t(1) << 54)) // will loop less than twice on average
			a = ((uint64_t(n_Rand32()) << 32) | n_Rand32()) >> 9; // can generate again, the draws are independent
		// generate one more bit

		int n_sign = a & 1;
		a >>= 1;
		// use it as sign; note that this bit is drawn independently of the number so it does
		// not increase chances of yielding zero (rather it splits zero's chances between zero and negative zero)

		_ASSERTE(a >= 0 && a <= (uint64_t(1) << 53));
		double f = double(int64_t(a)) * (1.0 / 9007199254740992.0); // msvc 6.0 doesn't implement conversion from uint64_t to double
		_ASSERTE(f >= 0 && f <= 1);
		return (n_sign)? -f : f;
		// return signed
	}

	/**
	 *	@copydoc CRandomGeneratorModel::f_Rand_LeftInclusive()
	 */
	double f_Rand_LeftInclusive()
	{
		return m_twister.genrand_res53(); // uniformly distributed
	}

	/**
	 *	@copydoc CRandomGeneratorModel::f_Rand_NonInclusive()
	 */
	double f_Rand_NonInclusive()
	{
		int64_t n_rand = ((uint64_t(n_Rand32()) << 32) | n_Rand32()) >> 11; // msvc 6.0 does not implement conversion from uint64_t to double
		while(!n_rand) // don't want to generate zero
			n_rand = ((uint64_t(n_Rand32()) << 32) | n_Rand32()) >> 11; // can try again, the samples are independent
		return double(n_rand) * (1.0 / 9007199254740992.0); // n_rand is (0, 9007199254740992) exclusive
		// more uniformly distributed floats
		// the division is precise, 9007199254740992.0 is a power of two

		//return m_twister.genrand_real3(); // not enough resolution
	}

	/**
	 *	@copydoc CRandomGeneratorModel::f_SignedRand_NonInclusive()
	 */
	double f_SignedRand_NonInclusive()
	{
		unsigned long a = n_Rand32() >> 5, b = n_Rand32();
		int n_sign = b & 32; // don't use the lowest bit, it consistently generates more negative values
		// use it as sign; note that this bit is drawn independently of the number so it does
		// not increase chances of yielding zero (rather it splits zero's chances between zero and negative zero)
		b >>= 6;
		double f = (uint64_t(a) * 67108864U + b) * (1.0 / 9007199254740992.0);
		return (n_sign)? -f : f;
	}
};

/**
 *	@brief concatenates outputs of a poor random number generator into a n-bit number
 *
 *	@tparam CResultType is type of result (e.g. uint32_t or uint64_t; must be unsigned)
 *	@tparam CIntermediateType is intermediate type, must be large enough to hold
 *		the maximum random number and must be unsigned
 *	@tparam n_rand_max is the maximum random number
 */
template <class CResultType, class CIntermediateType, const uint64_t n_rand_max>
class CPoorGenerator_Concatenator {
public:
	/**
	 *	@brief generates a single n-bit random number by concatenating the poor generator outputs
	 *	@tparam CRandGen is random number generator type
	 *	@param[in] r is random number generator instance
	 *	@return Returns a random number, up to CResultType wide.
	 *	@note This may invoke the poor random number generator several times
	 *		and will generate uniformly distributed random numbers, based on
	 *		the premise that the output of the poor generator is not correlated.
	 */
	template <class CRandGen>
	static inline CResultType n_Concatenate(CRandGen r)
	{
		//_ASSERTE(sizeof(CIntermediateType) >= sizeof(CResultType)); // make sure the intermediate type is big enough // nope, a smaller type is ok, just the below one must fit
		_ASSERTE(n_Bit_Width/*_Static*/(n_rand_max) <= sizeof(CIntermediateType) * 8); // make sure the intermediate type can fit the maximum
		_ASSERTE(CIntermediateType(-1) > 0); // make sure the intermediate type is unsigned
		_ASSERTE(CResultType(-1) > 0); // make sure the result type is unsigned
		// todo - make those compile-time asserts

		enum {
			b_is_pot_minus_1 = uint64_t(n_rand_max) == UINT64_MAX ||
				b_Is_POT_Static(uint64_t(n_rand_max) + 1), // can't vouch for larger powers of two
			n_generated_bit_num = sizeof(CResultType) * 8
		};

		static const uint64_t n_rand_max_4 = uint64_t(n_rand_max) | (uint64_t(n_rand_max) >> 1) |
			(uint64_t(n_rand_max) >> 2) | (uint64_t(n_rand_max) >> 3);
		static const uint64_t n_rand_max_8 = n_rand_max_4 | (n_rand_max_4 >> 4);
		static const uint64_t n_rand_max_16 = n_rand_max_8 | (n_rand_max_8 >> 8);
		static const uint64_t n_rand_max_32 = n_rand_max_16 | (n_rand_max_16 >> 16);
		static const uint64_t n_rand_max_64 = n_rand_max_32 | (n_rand_max_32 >> 32);
		static const CIntermediateType n_ones = CIntermediateType(n_rand_max_64);
			/*n_RightFill_Ones_Static(
			n_Make_Lower_POT_Static(uint64_t(n_rand_max)))*/; // those macros overflow with a 64-bit operand
		static const CIntermediateType n_mask = 
			(n_rand_max == n_ones)? n_rand_max : (((n_ones >> 1) <=
			CIntermediateType(-1))? n_ones >> 1 : CIntermediateType(-1));
		static const CIntermediateType n_bit_num = n_SetBit_Num_Static(n_mask); // not sure msvc 6.0 can instantiate this, maybe drop the _Static part

		enum {
			n_iteration_num = (n_generated_bit_num + n_bit_num - 1) / n_bit_num - 1
		};
		// so that the loop unrolls more easily

		/*CIntermediateType n_ones_dbg = n_ones,
			n_mask_dbg = n_mask,
			n_bit_num_dbg = n_bit_num,
			n_iteration_num_dbg = n_iteration_num;
		// can't see the values in msvc debugger*/

		if(b_is_pot_minus_1 && n_bit_num >= n_generated_bit_num) // compile-time expression
			return CResultType(r());
		// in case the generator maximum + 1 is a power of two and it generates enough bits, use it directly

		CIntermediateType n_result; // must be wide enough, so that we can loop in the while loop below
		if(b_is_pot_minus_1)
			n_result = r() & n_mask; // can't do that if not POT, would introduce nonuniformity
		else {
			n_result = r(); // initialize!
			while(n_result > n_mask)
				n_result = r(); // can do that, the draws are supposed to be independent
		}
		for(int i = 0; i < n_iteration_num; ++ i) {
			n_result <<= n_bit_num;
			if(b_is_pot_minus_1)
				n_result |= r() & n_mask;
			else {
				CIntermediateType n_next = r(); // must be wide enough, so that we can loop in the while loop below
				while(n_next > n_mask)
					n_next = r(); // can do that, the draws are supposed to be independent
				n_result |= n_next;
			}
		}
		// accumulate enough bits of the result

		return CResultType(n_result);
	}
};

/**
 *	@brief random number generator, based on standard library
 *	@tparam b_use_virtual_interface is common interface flag (default true)
 *	@note Only a single instance of this class may exist at a time. Note that this is not enforced by any checks.
 */
template <bool b_use_virtual_interface = true>
class CCLibGenerator : public CRandomGeneratorFloatImpl<
	CCLibGenerator<b_use_virtual_interface>, b_use_virtual_interface> {
public:
	/**
	 *	@brief random number generator traits, stored as enum
	 */
	enum {
		b_seedable = true, /**< @brief seed not ignored flag */
		b_64bit_rand = true, /**< @brief 64-bit random number capability flag */
		b_crypto_safe = false, /**< @brief cryptographically safe flag */
		b_fast_floats = true /**< @brief fast (possibly lower quality) floating point random number generation flag */
	};

public:
	/**
	 *	@brief default constructor; has no effect
	 */
	inline CCLibGenerator()
	{}

	/**
	 *	@brief constructor; initializes the random generator
	 */
	inline CCLibGenerator(unsigned int n_seed)
	{
		Seed(n_seed);
	}

	/**
	 *	@copydoc CRandomGeneratorModel::Seed()
	 */
	void Seed(unsigned int n_seed)
	{
		srand(n_seed);
	}

	/**
	 *	@brief generates a 32-bit unsigned random number
	 *	@return Returns the generated random number.
	 */
	uint32_t n_Rand32()
	{
		if(RAND_MAX <= UINT32_MAX) // compile-time expression
			return CPoorGenerator_Concatenator<uint32_t, uint32_t, RAND_MAX>::n_Concatenate(rand);
		else
			return CPoorGenerator_Concatenator<uint32_t, uint64_t, RAND_MAX>::n_Concatenate(rand);
	}

	/**
	 *	@brief generates a 64-bit unsigned random number
	 *	@return Returns the generated random number.
	 */
	uint64_t n_Rand64()
	{
		if(RAND_MAX <= UINT32_MAX) // compile-time expression
			return CPoorGenerator_Concatenator<uint64_t, uint64_t, RAND_MAX>::n_Concatenate(rand);
		else
			return CPoorGenerator_Concatenator<uint64_t, uint64_t, RAND_MAX>::n_Concatenate(rand);
		// this can save one rand() iteration compared to calling n_Rand32() twice 
	}

	/**
	 *	@copydoc CRandomGeneratorModel::f_Rand()
	 *	@note This is fast, rather than uniformly distributed.
	 */
	double f_FastRand()
	{
		return rand() / double(RAND_MAX);
	}

	/**
	 *	@copydoc CRandomGeneratorModel::f_Rand_LeftInclusive()
	 *	@note This is fast, rather than uniformly distributed.
	 */
	double f_FastRand_LeftInclusive()
	{
		_ASSERTE(RAND_MAX < INT_MAX);
		return rand() / double(uint64_t(RAND_MAX) + 1); // probably ok, just low resolution (if RAND_MAX is all ones)
	}

	/**
	 *	@copydoc CRandomGeneratorModel::f_Rand_NonInclusive()
	 *	@note This is fast, rather than uniformly distributed.
	 */
	double f_FastRand_NonInclusive()
	{
		_ASSERTE(RAND_MAX < INT_MAX - 1);
		return (rand() + 1.0) / double(uint64_t(RAND_MAX) + 2); // all wrong
	}
};

/**
 *	@brief a simple and fast congruential random nunber generator
 *	@tparam b_use_virtual_interface is common interface flag (default true)
 *	@note This generator is taken from Wikipedia and was found to generate rather
 *		poor random numbers; the lowest bit flips in regular intervals and as such
 *		it is not suitable for some tasks. This also means that depending on seed,
 *		all the 64-bit random numbers will be even or all will be odd.
 */
template <bool b_use_virtual_interface = true>
class CCongruentialGenerator : public CRandomGeneratorFloatImpl<
	CCongruentialGenerator<b_use_virtual_interface>, b_use_virtual_interface> {
public:
	/**
	 *	@brief random number generator traits, stored as enum
	 */
	enum {
		b_seedable = true, /**< @brief seed not ignored flag */
		b_64bit_rand = false, /**< @brief 64-bit random number capability flag */
		b_crypto_safe = false, /**< @brief cryptographically safe flag */
		b_fast_floats = true /**< @brief fast (possibly lower quality) floating point random number generation flag */
	};

protected:
	uint32_t m_n_x; /**< @brief random generator state */

public:
	/**
	 *	@brief default constructor; has no effect
	 */
	inline CCongruentialGenerator()
	{}

	/**
	 *	@brief constructor; initializes the random generator
	 */
	inline CCongruentialGenerator(unsigned int n_seed)
	{
		Seed(n_seed);
	}

	/**
	 *	@copydoc CRandomGeneratorModel::Seed()
	 */
	void Seed(unsigned int n_seed)
	{
		m_n_x = n_seed;
		n_Rand32(); // do not return seed
	}

	/**
	 *	@brief generates a 32-bit unsigned random number
	 *	@return Returns the generated random number.
	 */
	uint32_t n_Rand32()
	{
		m_n_x = m_n_x * 1664525U + 1013904223U;
		return m_n_x;
	}

	/**
	 *	@copydoc CRandomGeneratorModel::f_Rand()
	 *	@note This only uses 31 bits of randomness.
	 */
	double f_FastRand()
	{
		uint32_t n_rand = n_Rand32();
		while(n_rand > (1U << 31)) // not zero
			n_rand = n_Rand32(); // can choose again, the draws are independent
		return n_rand * (1.0 / (1U << 31)); // totally ok, division by POT
		//return n_Rand32() / double(UINT32_MAX); // imprecise division, likely nonuniform
		// could sacrifice one bit of precision for uniform numbers (did that), or take two passes of n_Rand32()
	}

	/**
	 *	@copydoc CRandomGeneratorModel::f_Rand_LeftInclusive()
	 *	@note This only uses 32 bits of randomness.
	 */
	double f_FastRand_LeftInclusive()
	{
		return n_Rand32() * (1.0 / (double(UINT32_MAX) + 1)); // totally ok, division by POT
	}

	/**
	 *	@copydoc CRandomGeneratorModel::f_Rand_NonInclusive()
	 *	@note This only uses 32 bits of randomness.
	 */
	double f_FastRand_NonInclusive()
	{
		uint32_t n_rand = n_Rand32();
		while(!n_rand) // not zero
			n_rand = n_Rand32(); // can choose again, the draws are independent
		return n_rand * (1.0 / (double(UINT32_MAX) + 1)); // totally ok, division by POT
		//return (double(n_Rand32()) + 1) / (double(UINT32_MAX) + 2); // imprecise division, likely nonuniform
	}
};

/**
 *	@brief Numerical Recipes' long generator (period 3.138 * 10^57)
 *	@tparam b_use_virtual_interface is common interface flag (default true)
 */
template <bool b_use_virtual_interface = true>
class CNRLongGenerator : public CRandomGeneratorFloatImpl<
	CNRLongGenerator<b_use_virtual_interface>, b_use_virtual_interface> {
public:
	/**
	 *	@brief random number generator traits, stored as enum
	 */
	enum {
		b_seedable = true, /**< @brief seed not ignored flag */
		b_64bit_rand = true, /**< @brief 64-bit random number capability flag */
		b_crypto_safe = false, /**< @brief cryptographically safe flag */
		b_fast_floats = false /**< @brief fast (possibly lower quality) floating point random number generation flag */
	};

protected:
	uint64_t m_n_u; /**< @brief random generator state */
	uint64_t m_n_v; /**< @brief random generator state */
	uint64_t m_n_w; /**< @brief random generator state */

public:
	/**
	 *	@brief constructor; initializes the random generator
	 */
	CNRLongGenerator(uint64_t n_seed = 0)
	{
		Seed64(n_seed);
	}

	/**
	 *	@copydoc CRandomGeneratorModel::Seed()
	 */
	void Seed(unsigned int n_seed)
	{
		Seed64(n_seed);
	}

	/**
	 *	@copydoc CRandomGeneratorModel::Seed()
	 */
	void Seed64(uint64_t n_seed)
	{
		m_n_v = (uint64_t(0x38ecac5f) << 32) |	0xb3251641U; // 4101842887655102017LL
		m_n_w = 1;
		//_ASSERTE(n_seed != m_n_v); // can happen, don't want this to kill my process in the unlikely event
		m_n_u = n_seed ^ m_n_v;
		n_Rand64();
		m_n_v = m_n_u;
		n_Rand64();
		m_n_w = m_n_v;
		n_Rand64();
	}

	/**
	 *	@brief generates a 64-bit unsigned random number
	 *	@return Returns the generated random number.
	 */
	uint64_t n_Rand64()
	{
		m_n_u = m_n_u * ((uint64_t(0x27bb2ee6U) << 32) | 0x87b0b0fdU)/*2862933555777941757LL*/ +
			((uint64_t(0x61c88646U) << 32) | 0x80b583bfU)/*7046029254386353087LL*/;
		m_n_v ^= m_n_v >> 17;
		m_n_v ^= m_n_v << 31;
		m_n_v ^= m_n_v >> 8;
		m_n_w = 4294957665U * (m_n_w & 0xffffffff) + (m_n_w >> 32);
		uint64_t x = m_n_u ^ (m_n_u << 21);
		x ^= x >> 35;
		x ^= x << 4;
		return (x + m_n_v) ^ m_n_w;
	}

	/**
	 *	@brief generates a 32-bit unsigned random number
	 *	@return Returns the generated random number.
	 */
	uint32_t n_Rand32()
	{
		return uint32_t(n_Rand64());
	}
};

/**
 *	@brief Numerical Recipes' fast generator Ranq1 (period 1.8 * 10^19)
 *	@tparam b_use_virtual_interface is common interface flag (default true)
 */
template <bool b_use_virtual_interface = true>
class CNRFastGenerator : public CRandomGeneratorFloatImpl<
	CNRFastGenerator<b_use_virtual_interface>, b_use_virtual_interface> {
public:
	/**
	 *	@brief random number generator traits, stored as enum
	 */
	enum {
		b_seedable = true, /**< @brief seed not ignored flag */
		b_64bit_rand = true, /**< @brief 64-bit random number capability flag */
		b_crypto_safe = false, /**< @brief cryptographically safe flag */
		b_fast_floats = false /**< @brief fast (possibly lower quality) floating point random number generation flag */
	};

protected:
	uint64_t m_n_v; /**< @brief random generator state */

public:
	/**
	 *	@brief constructor; initializes the random generator
	 */
	CNRFastGenerator(uint64_t n_seed = 0)
	{
		Seed64(n_seed);
	}

	/**
	 *	@copydoc CRandomGeneratorModel::Seed()
	 */
	void Seed(unsigned int n_seed)
	{
		Seed64(n_seed);
	}

	/**
	 *	@copydoc CRandomGeneratorModel::Seed()
	 */
	void Seed64(uint64_t n_seed)
	{
		m_n_v = (uint64_t(0x38ecac5f) << 32) |	0xb3251641U; // 4101842887655102017LL
		//_ASSERTE(n_seed != m_n_v); // can happen, don't want this to kill my process in the unlikely event
		m_n_v ^= n_seed;
		m_n_v = n_Rand64();
	}

	/**
	 *	@brief generates a 64-bit unsigned random number
	 *	@return Returns the generated random number.
	 */
	uint64_t n_Rand64()
	{
		m_n_v ^= m_n_v >> 21;
		m_n_v ^= m_n_v << 35;
		m_n_v ^= m_n_v >> 4;
		return m_n_v * ((uint64_t(0x2545f491) << 32) | 0x4f6cdd1dU); // 2685821657736338717LL;
	}

	/**
	 *	@brief generates a 32-bit unsigned random number
	 *	@return Returns the generated random number.
	 */
	uint32_t n_Rand32()
	{
		return uint32_t(n_Rand64());
	}
};

/**
 *	@brief uniform integer distribution
 *	@tparam CIntType is integer data type
 */
template <class CIntType>
class CUniformIntegerDistribution {
protected:
	CIntType m_n_low; /**< @brief low bound */
	CIntType m_n_high; /**< @brief high bound */

public:
	/**
	 *	@brief default constructor
	 *
	 *	@param[in] n_low is low bound (inclusive)
	 *	@param[in] n_high is high bound (inclusive)
	 */
	CUniformIntegerDistribution(CIntType n_low, CIntType n_high)
		:m_n_low(n_low), m_n_high(n_high)
	{
		_ASSERTE(m_n_low <= m_n_high); // would break overflow checks
	}

	/**
	 *	@brief gets a random number in the given distribution, using the specified generator
	 *	@tparam CRandomNumberGenerator is random number generator model
	 *	@param[in,out] r_gen is random number generator instance
	 *	@return Returns a random number in the given distribution.
	 */
	template <class CRandomNumberGenerator>
	CIntType operator ()(CRandomNumberGenerator &r_gen)
	{
#if 1
		_ASSERTE(m_n_high <= UINT64_MAX);
		uint64_t n_range = m_n_high;
		_ASSERTE((m_n_low < 0 && m_n_high <= UINT64_MAX + m_n_low) ||
			(m_n_low >= 0 && m_n_high - m_n_low <= UINT64_MAX));
		n_range -= m_n_low;
		_ASSERTE(n_range <= UINT64_MAX); // but is still exclusive
		// calculate the required (exclusive) range of values (note that we cant use
		// CIntType here, as the range might not fit in it if it is signed)
		// also note that inclusive range might not be representable

		if(n_range <= UINT32_MAX) {
			uint32_t n_range32 = uint32_t(n_range);
			uint32_t n_highest_modulus32 = (n_range32 < UINT32_MAX)?
				UINT32_MAX - ((UINT32_MAX % (n_range32 + 1)) + 1) % (n_range32 + 1) : UINT32_MAX; // so that it is inclusive
			_ASSERTE(n_highest_modulus32 == n_range32 || (n_highest_modulus32 + 1) % (n_range32 + 1) == 0); // make sure that the highest modulus is indeed divisible by range (all sizes are inclusive so need to add +1 everywhere)
			_ASSERTE(n_highest_modulus32 == UINT32_MAX || n_highest_modulus32 > UINT32_MAX - (n_range32 + 1)); // make sure there is no higher modulus
			for(;;) {
				uint32_t n_rand = r_gen.n_Rand32();
				if(n_rand <= n_highest_modulus32) {
					CIntType n_result = m_n_low + ((n_range32 < UINT32_MAX)? CIntType(n_rand % (n_range32 + 1)) : CIntType(n_rand)); // avoid overflow
					_ASSERTE(n_result >= m_n_low && n_result <= m_n_high);
					/*if(n_result == m_n_low)
						printf("debug: low hit\n");
					else if(n_result == m_n_high)
						printf("debug: high hit\n");*/ // debug
					return n_result;
				}
			}
		} else {
			uint64_t n_highest_modulus = (n_range < UINT64_MAX)?
				UINT64_MAX - ((UINT64_MAX % (n_range + 1)) + 1) % (n_range + 1) : UINT64_MAX; // so that it is inclusive
			_ASSERTE(n_highest_modulus == n_range || (n_highest_modulus + 1) % (n_range + 1) == 0); // make sure that the highest modulus is indeed divisible by range (all sizes are inclusive so need to add +1 everywhere)
			_ASSERTE(n_highest_modulus == UINT64_MAX || n_highest_modulus > UINT64_MAX - (n_range + 1)); // make sure there is no higher modulus
			for(;;) {
				uint64_t n_rand = CRand64Caller<CRandomNumberGenerator>::n_Rand64(r_gen);
				if(n_rand <= n_highest_modulus) {
					CIntType n_result = m_n_low + ((n_range < UINT64_MAX)? CIntType(n_rand % (n_range + 1)) : CIntType(n_rand));
					_ASSERTE(n_result >= m_n_low && n_result <= m_n_high);
					return n_result;
				}
			}
		}
		// generate a number and see if it falls under the largest modulus (so that
		// unbiased distribution is generated), otherwise generate one more random number
#else // 1
		// note that this is flawed; should use random integers, either uint32_t or uint64_t
		// and use modulo with repeated generation

		double f_random = r_gen.f_Rand_LeftInclusive();
		// get a random number in [0, 1)

		/*bool a = m_n_low < INT64_MIN || m_n_low > INT64_MAX,
			b = m_n_high < INT64_MIN || m_n_high > INT64_MAX,
			c = m_n_low < INT64_MIN + 1,
			d = (int64_t(m_n_low) - 1) < 0 && m_n_high > INT64_MAX + (int64_t(m_n_low) - 1),
			e = (int64_t(m_n_low) - 1) > 0 && m_n_high < INT64_MIN + (int64_t(m_n_low) - 1),
			f = floor(m_n_high - (double(m_n_low) - 1)) <= m_n_high - int64_t(m_n_low) + 1;*/ // debug
		_ASSERTE(m_n_low < INT64_MIN || m_n_low > INT64_MAX || // m_n_low doesn't fit signed 64-bit integer
			m_n_high < INT64_MIN || m_n_high > INT64_MAX || // m_n_high doesn't fit signed 64-bit integer
			m_n_low < INT64_MIN + 1 || // m_n_low - 1 underflows
			((int64_t(m_n_low) - 1) < 0 && m_n_high > INT64_MAX + (int64_t(m_n_low) - 1) || // m_n_high - (m_n_low - 1) overflows
			(int64_t(m_n_low) - 1) > 0 && m_n_high < INT64_MIN + (int64_t(m_n_low) - 1)) || // m_n_high - (m_n_low - 1) underflows
			floor(m_n_high - (double(m_n_low) - 1)) <= m_n_high - int64_t(m_n_low) + 1); // the scaler is greater than the integer version
		// make sure that the float value is smaller than the integer value, if the integer value can be calculated

		f_random *= floor(m_n_high - (double(m_n_low) - 1)); // watch out for overflow (both integer and float)
		// scale the random number to the number of integers in the specified interval

		CIntType n_result = CIntType(f_random) + m_n_low;
		_ASSERTE(n_result >= m_n_low && n_result <= m_n_high); // make sure it is in the bounds
		// round down, add low

		return n_result;
#endif // 1
	}
};

/**
 *	@brief a simple generator of random permutation of unique bounded values
 */
class CUniqueRandomPermutation {
public:
	/**
	 *	@brief custom comparison operator for std::lower_bound / std::upper_bound
	 *
	 *	Compares a reference value to the value in the vector minus its index. Use as follows:
	 *
	 *	@code
	 *	std::vector<int> sequence = ...; // a strictly increasing sequence
	 *	int n = ...; // needle
	 *
	 *	typedef CCompareWithOffset<std::vector<int> > Comparator;
	 *
	 *	std::vector<int>::iterator iter = std::upper_bound(sequence.begin(), sequence.end(),
	 *		Comparator::TNeedle(n), Comparator(sequence.begin()));
	 *	// finds the first i, such that sequence[i] - i > n
	 *	@endcode
	 *
	 *	@tparam CScalar is scalar type
	 */
	template <class CContainer>
	class CCompareWithOffset {
	public:
		typedef typename CContainer::value_type CScalar;
		typedef typename CContainer::const_iterator CConstIter;

		/**
		 *	@brief a dummy type that unambiguously marks the needle
		 */
		struct TNeedle {
			CScalar n_value; /**< @brief value of the needle */

			/**
			 *	@brief default constructor
			 *	@param[in] r_n is value of the needle
			 */
			inline TNeedle(const CScalar &r_n)
				:n_value(r_n)
			{}
		};

	protected:
		CConstIter m_p_begin_it; /**< @brief iterator pointing to the first element of the vector */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] p_begin_it is iterator pointing to the first element of the vector
		 */
		inline CCompareWithOffset(CConstIter p_begin_it)
			:m_p_begin_it(p_begin_it)
		{}

		/**
		 *	@brief less-than comparison operator; compares a value from the vector to a reference
		 *
		 *	@param[in] r_value is const reference to a values in the vector
		 *	@param[in] n_reference is reference value
		 *
		 *	@return Returns true if the left value minus its index is lower than
		 *		the reference value, otherwise returns false.
		 */
		inline bool operator ()(const CScalar &r_value, TNeedle n_reference) const
		{
			size_t n_index = &r_value - &*m_p_begin_it;
			// calculate index in the vector

			return r_value < CScalar(n_reference.n_value + n_index);
		}

		/**
		 *	@brief less-than comparison operator; compares a reference to a value from the vector
		 *
		 *	@param[in] n_reference is reference value
		 *	@param[in] r_value is const reference to a values in the vector
		 *
		 *	@return Returns true if the reference value is lower than
		 *		the right value minus its index, otherwise returns false.
		 */
		inline bool operator ()(TNeedle n_reference, const CScalar &r_value) const
		{
			size_t n_index = &r_value - &*m_p_begin_it;
			// calculate index in the vector

			return CScalar(n_reference.n_value + n_index) < r_value;
		}

		/**
		 *	@brief less-than comparison operator; compares two values from the vector
		 *
		 *	@param[in] r_value_l is const reference to a values in the vector
		 *	@param[in] r_value_r is const reference to a values in the vector
		 *
		 *	@return Returns true if the left value minus its index is lower than
		 *		the right value minus its index, otherwise returns false.
		 *
		 *	@note This is required by Visual Studio for debug checking that
		 *		the sequence in the vector is indeed sorted.
		 */
		inline bool operator ()(const CScalar &r_value_l, const CScalar &r_value_r) const
		{
			size_t n_index_l = &r_value_l - &*m_p_begin_it;
			size_t n_index_r = &r_value_r - &*m_p_begin_it;
			// calculate indices of both elements in the vector

			return CScalar(r_value_l + n_index_r) < CScalar(r_value_r + n_index_l);
			// or equivalently r_value_l - n_index_l < r_value_r - n_index_r
		}
	};

public:
	template <class CContainer, class CRandomNumberGenerator>
	static void Generate(CContainer &r_permutation, size_t n_permutation_size,
		size_t n_set_size, CRandomNumberGenerator &random) // throw(std::bad_alloc)
	{
		typedef typename CContainer::value_type CScalar;
		typedef typename CContainer::iterator CIterator;

		r_permutation.clear(); // !!
		if(n_permutation_size < 75) {
			// b1: 3187.000 msec (the fastest)
			// b2: 3734.000 msec
			for(size_t i = 0; i < n_permutation_size; ++ i) {
				CUniformIntegerDistribution<CScalar> distribution(CScalar(0), CScalar(n_set_size - i - 1)); // explicit cast to avoid some x64 warnings
				CScalar n = distribution(random);
				// get a random number

				CIterator p_where_it = r_permutation.begin();
				for(size_t j = 0; j < i; ++ j, ++ p_where_it, ++ n) {
					if(n < *p_where_it)
						break;
				}
				// see where it should be inserted

				r_permutation.insert(p_where_it, 1, n);
				// insert it, maintain a sorted sequence
			}
		} else {
			// b1: 3578.000 msec
			// b2: 1703.000 msec (the fastest)
			for(size_t i = 0; i < n_permutation_size; ++ i) {
				CUniformIntegerDistribution<CScalar> distribution(CScalar(0), CScalar(n_set_size - i - 1)); // explicit cast to avoid some x64 warnings
				CScalar n = distribution(random);
				// get a random number

				typedef CCompareWithOffset<CContainer> TCompare;
				CIterator p_begin_it = r_permutation.begin();
				CIterator p_where_it = std::upper_bound(p_begin_it, r_permutation.end(),
					typename TCompare::TNeedle(n), TCompare(p_begin_it));
				// see where it should be inserted

				r_permutation.insert(p_where_it, 1, CScalar(n + (p_where_it - p_begin_it))); // needs to be in parentheses for VS 2008, otherwise the safe iterators go crazy // explicit cast to avoid some x64 warnings
				// insert it in the list, maintain a sorted sequence
			}

			// this does binary search with the comparison "n + j < rand_num[j]" with non-constant needle,
			// which can be rewritten to one with constant needle and a modified sequence "n < rand_num[j] - j".
			// it is easily shown that since the lowest distance between two elements of the original
			// sequence "rand_num[j]" is:
			//
			//		rand_num[j] - rand_num[j - 1] >= 1
			//
			// (the generated numbers are unique), and at the same time, the difference
			// between different elements of "-j" is:
			//
			//		-j - -(j - 1) = -1
			//
			// The difference between elements of their sum "rand_num[j] - j" is:
			//
			//		(rand_num[j] - j) - (rand_num[j - 1] - (j - 1)) >= 0
			//
			// therefore, the sequence is nondecreasing and this comparison can be used.
		}
	}
};

#endif // !__RANDOM_GENERATORS_INCLUDED
