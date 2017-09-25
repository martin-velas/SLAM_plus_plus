/*
								+--------------------------------+
								|                                |
								| ***   Segregated storage   *** |
								|                                |
								| Copyright (c) -tHE SWINe- 2012 |
								|                                |
								|          Segregated.h          |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __SEGREGATED_STORAGE_TEMPLATES_INCLUDED
#define __SEGREGATED_STORAGE_TEMPLATES_INCLUDED

/**
 *	@file include/slam/Segregated.h
 *	@date 2012
 *	@author -tHE SWINe-
 *	@brief simple segregated storage templates
 *
 *	t_odo test the doxygen documentation
 *	@todo retain pages parameter (including null being valid value for no retain)
 *	@todo test insert with iterators from the pool itself
 *	@todo mark operations that invalidate the iterators (more than you think, actually - std::vector iterators invalidate easily)
 *
 *	@date 2012-09-12
 *
 *	Fixed linux build issues, namely added enclosing class template parameters to all the
 *	member class templates, requiring partial or full specialization (then g++ stops yapping
 *	about putting stuff inside namespaces), adding some 'typename' here and there, adding
 *	CConstIterator:: prefix before members, inherited in CIterator, and creating a small
 *	namespace for static assert (unused at the moment).
 *
 *	Fixed a bug in forward_allocated_pool::CConstIterator::operator +(), which didn't correctly
 *	decrement over page boundaries (correct code was in CIterator, but wasn't copied here). Now
 *	there's just a single version of the code.
 *
 *	@date 2012-12-04
 *
 *	Added another "&& _MSC_VER < 1700" to the __SEGREGATED_MAKE_CHECKED_ITERATORS ifdef as it
 *	is causing trouble when building with visual studio 2012 (not sure if the max _MSC_VER is
 *	low enough, did not try VS 2010; if you get errors, try decrementing it.
 *
 *	@date 2017-04-04
 *
 *	Added _Unchecked_type to the __SEGREGATED_MAKE_CHECKED_ITERATORS section for _MSC_VER >= 1900
 *	(VS 2015 or above), to avoid warnings about iterating on unchecked iterators. This also
 *	requires implementing conversion to/from unchecked iterators that is now implemented but the
 *	current code does not seem to ever call it.
 *
 *	If you experience problems, please disable __SEGREGATED_MAKE_CHECKED_ITERATORS (will cause
 *	a warning but will work). Any feedback on this is generally appreciated.
 *
 */

/**
 *	@def __SEGREGATED_MAKE_CHECKED_ITERATORS
 *	@brief if defined, forward_allocated_pool iterators will behave as STL checked iterators
 *		(this is only to avoid some warnings when compiling with MSVC, no debugging functionality
 *		is actually implemented)
 *	@note Using this on different versions of STL than the MSVC 90s one may cause compatibility
 *		problems, undefine the macro in such case.
 */
#define __SEGREGATED_MAKE_CHECKED_ITERATORS

/**
 *	@def __SEGREGATED_COMPILE_FAP_TESTS
 *	@brief if defined, the test case CFAP_Test is compiled
 */
//#define __SEGREGATED_COMPILE_FAP_TESTS

#ifdef __SEGREGATED_MAKE_CHECKED_ITERATORS
/**
 *	@def _SCL_SECURE_NO_WARNINGS
 *	@brief a define for MSVC, to avoid warnings about unsecure STL functions and iteratiors
 */
#ifndef _SCL_SECURE_NO_WARNINGS // might be declared via commandline
#define _SCL_SECURE_NO_WARNINGS
#endif // _SCL_SECURE_NO_WARNINGS
#endif // __SEGREGATED_MAKE_CHECKED_ITERATORS

#include "Unused.h"
#ifndef _ASSERTE
#include <assert.h>
/**
 *	@brief basic debug assertion macro
 *	@param[in] x is the condition that is supposed to hold
 */
#define _ASSERTE(x) assert(x)
#endif // !_ASSERTE
#include <vector>
#include <iterator> // std::random_access_iterator_tag
#include <stdlib.h> // posix_memalign()

/**
 *	@brief internal implementation of forward allocated pool
 */
namespace fap_detail {

/**
 *	@brief compile-time assertion helper
 *	@tparam b_expression is assertion value
 */
template <const bool b_expression>
class CStaticAssert {
public:
	typedef void PAGE_SIZE_MISMATCH; /**< @brief assertion tag (used in fap_base::Swap() when pools of different page size are swapped) */
};

/**
 *	@brief compile-time assertion helper (specialization for assertion failed)
 */
template <>
class CStaticAssert<false> {};

/**
 *	@brief simple segregated storage class
 *
 *	This type of storage is very fast, there is O(1) insertion time,
 *	inserting a new elements doesn't change the addresses of the old elements,
 *	nor does it invalidate the iterators (under certain conditions).
 *	The limitation is that the new elements can only be inserted to or erased
 *	from the end of the storage. Erasing should, in general, never invalidate
 *	the iterators.
 *
 *	Another limitation of the current implementation is that there is an aggressive
 *	allocation policy, causing only the immediately required memory to remain
 *	allocated. Repeated erasing and inserting of elements may therefore lead
 *	to considerable overheads due to memory allocation. This is, however, not
 *	a problem for some applications (those, where the elements are only inserted
 *	and never/only scarcely erased). Note that this problem could be easily solved
 *	by retaining the pages, at the cost of slightly complicating the allocation code
 *	and additional space requirements on the stack.
 *
 *	@tparam T is data type being stored
 *	@tparam _n_page_size_elems is page size (in elements) at compile-time, or 0 for dynamic
 *	@tparam _n_memory_align is memory alignment (in bytes)
 *
 *	@note This implementation only allows for allocation
 *		and deallocation at the end of storage (hence "forward allocated").
 */
template <class T, const int _n_page_size_elems = 0, const int _n_memory_align = 0> // todo - add the retain pages parameter to compensate for the aggressive deallocation policy
class fap_base {
public:
	/**
	 *	@brief template parameters, stored as enum
	 */
	enum {
		n_compile_time_page_size_elems = _n_page_size_elems, /**< @brief number of elements in a single page at compile-time (or 0 for dynamic) */
		n_memory_alignment = _n_memory_align /**< @brief memory alignment, in bytes */
	};

	/**
	 *	@brief allocates a buffer with the same memory alignment as the pool
	 *	@param[in] n_size is size of the buffer in bytes
	 *	@return Returns pointer to the newly allocated buffer;
	 *		the caller is responsible for deleting it.
	 *	@note This function throws std::bad_alloc.
	 */
	static void *aligned_alloc(size_t n_size) // throw(std::bad_alloc)
	{
		return CAllocNewPage<uint8_t, size_t, 0, n_memory_alignment>(n_size)();
	}

	/**
	 *	@brief deletes a buffer previously allocated using aligned_alloc()
	 *	@param[in] p_ptr is pointer to the buffer to be deleted
	 */
	static void aligned_free(void *p_ptr)
	{
		CAllocNewPage<uint8_t, size_t, 0, n_memory_alignment>::DeletePage((uint8_t*)p_ptr);
	}

protected:
	/**
	 *	@brief page size, known at compile-time
	 *	@tparam __n_page_size_elems is page size (in elements) at compile-time (guaranteed nonzero)
	 */
	template <class _T, const int __n_page_size_elems, const int __n_memory_align>
	struct TPageSize {
		/**
		 *	@brief template parameters, stored as enum
		 */
		enum {
			n_page_size_elems = __n_page_size_elems /**< @brief page size (in elements) at compile-time (guaranteed nonzero) */
		};

		/**
		 *	@brief dummy function for setting page size (no-op)
		 *	@param[in] n_new_size is new page size in elements (ignored)
		 */
		inline void Set(size_t UNUSED(n_new_size))
		{}

		/**
		 *	@brief gets current page size
		 *	@return Returns the current page size, in elements.
		 */
		inline operator size_t() const
		{
			return n_page_size_elems;
		}
	};

	/**
	 *	@brief page size, set at run-time
	 */
	template <class _T, const int __n_memory_align>
	struct TPageSize<_T, 0, __n_memory_align> {
		size_t n_page_size_elems; /**< @brief page size (in elements) at run-time */

		/**
		 *	@brief sets page size
		 *	@param[in] n_new_size is new page size in elements (must be greater than zero)
		 */
		inline void Set(size_t n_new_size)
		{
			n_page_size_elems = n_new_size;
		}

		/**
		 *	@brief gets current page size
		 *	@return Returns the current page size, in elements.
		 */
		inline operator size_t() const
		{
			return n_page_size_elems;
		}
	};

	typedef TPageSize<T, _n_page_size_elems, _n_memory_align> _TyPageSize; /**< @brief page size data type, based on either compile-time or dynamic allocation */

	/**
	 *	@brief a simple functor, designed for allocating new pages
	 *
	 *	@tparam _T is data type being allocated
	 *	@tparam _TyPageSize is page size data type
	 *	@tparam n_compile_time_page_size_elems is page size at compile time
	 *	@tparam n_mem_align is memory alignment (in bytes)
	 */
	template <class _T, class _TyPageSize, const int n_compile_time_page_size_elems, const int n_mem_align>
	class CAllocNewPage {
	protected:
		_TyPageSize m_t_page_size; /**< @brief required page size, in elements */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] t_page_size is the required page size, in elements
		 */
		inline CAllocNewPage(_TyPageSize t_page_size)
			:m_t_page_size(t_page_size)
		{}

		/**
		 *	@brief allocates a new page
		 *	@return Returns pointer to the new page.
		 *	@note This function throws std::bad_alloc.
		 */
		inline _T *operator ()() // throw(std::bad_alloc)
		{
			void *p_result;
#if defined(_MSC_VER) && !defined(__MWERKS__)
			p_result = _aligned_malloc(m_t_page_size * sizeof(_T), n_mem_align);
#elif (defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 200112L) || (defined(_XOPEN_SOURCE) && _XOPEN_SOURCE >= 600)
			if(posix_memalign(&p_result, n_mem_align, m_t_page_size * sizeof(_T)))
				p_result = 0; // to mark error
#else // _MSC_VER && !__MWERKS__
			p_result = _mm_malloc(m_t_page_size * sizeof(_T), n_mem_align);
#endif // _MSC_VER && !__MWERKS__
			if(!p_result && m_t_page_size * sizeof(_T) != 0)
				throw std::bad_alloc();
			return (_T*)p_result;
		}

		/**
		 *	@brief deletes a page from memory
		 *	@param[in] p_page is the pointer to the page to be deleted
		 */
		static inline void DeletePage(_T *p_page)
		{
#if defined(_MSC_VER) && !defined(__MWERKS__)
			_aligned_free(p_page);
#elif (defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 200112L) || (defined(_XOPEN_SOURCE) && _XOPEN_SOURCE >= 600)
			free(p_page); // posix_memalign() is compatible with free()
#else // _MSC_VER && !__MWERKS__
			_mm_free(p_page);
#endif // _MSC_VER && !__MWERKS__
		}
	};

	/**
	 *	@brief a simple functor, designed for allocating new pages
	 *		(specialization for page size, set at runtime)
	 *
	 *	@tparam _T is data type being allocated
	 *	@tparam _TyPageSize is page size data type
	 *	@tparam n_compile_time_page_size_elems is page size at compile time
	 */
	template <class _T, class _TyPageSize, const int n_compile_time_page_size_elems>
	class CAllocNewPage<_T, _TyPageSize, n_compile_time_page_size_elems, 0> {
	protected:
		_TyPageSize m_t_page_size; /**< @brief required page size, in elements */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] t_page_size is the required page size, in elements
		 */
		inline CAllocNewPage(_TyPageSize t_page_size)
			:m_t_page_size(t_page_size)
		{}

		/**
		 *	@brief allocates a new page
		 *	@return Returns pointer to the new page.
		 *	@note This function throws std::bad_alloc.
		 */
		inline _T *operator ()() // throw(std::bad_alloc)
		{
			return new _T[m_t_page_size];
		}

		/**
		 *	@brief deletes a page from memory
		 *	@param[in] p_page is the pointer to the page to be deleted
		 */
		static inline void DeletePage(_T *p_page)
		{
			delete[] p_page;
		}
	};

	typedef CAllocNewPage<T, _TyPageSize, _n_page_size_elems, _n_memory_align> _CPageAlloc; /**< @brief page allocator object */

public:
	typedef T _Ty; /**< @brief data type being stored */
	typedef std::vector<_Ty*> _TyPageList; /**< @brief type for the list of pages */

	/**
	 *	@brief constant iterator type
	 */
#if defined(__SEGREGATED_MAKE_CHECKED_ITERATORS) && defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400 && _MSC_VER < 1600 // not sure if the max _MSC_VER is low enough; if you get errors on the line below, decrement it
	class CConstIterator : public std::_Ranit<_Ty, ptrdiff_t, const _Ty*, const _Ty&> { // MSVC secure iterator bullshit
#else // __SEGREGATED_MAKE_CHECKED_ITERATORS && _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400 && _MSC_VER < 1600
	class CConstIterator {
#endif // __SEGREGATED_MAKE_CHECKED_ITERATORS && _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400 && _MSC_VER < 1600
	friend class fap_base;
	public:
		typedef _Ty value_type; /**< @brief data type being accessed */
		typedef ptrdiff_t difference_type; /**< @brief data type for measuring difference between iterators */
		typedef const _Ty *pointer; /**< @brief pointer to the payload type */
		typedef const _Ty &reference; /**< @brief reference to the payload type */

#if defined(__APPLE__)
		typedef std::random_access_iterator_tag iterator_category; /**< @brief iterator category (MAC) */
#elif defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		typedef std::random_access_iterator_tag iterator_category; /**< @brief iterator category (MSVC) */
#if defined(__SEGREGATED_MAKE_CHECKED_ITERATORS) && _SECURE_SCL
#if _MSC_VER < 1600 // not sure if the max _MSC_VER is low enough; if you get errors on the line below, decrement it
		typedef std::_Range_checked_iterator_tag _Checked_iterator_category; /**< @brief checked iterator category (MSVC 90) */
#else //  _MSC_VER < 1600
#if 0
		typedef pointer _Unchecked_type; /**< @brief unchecked iterator type, marks a checked iterator (VS 2015) */

		/**
		 *	@brief reset from unchecked iterator (VS 2015)
		 *	@param[in] _Right is unchecked iterator type
		 *	@return Returns reference to this.
		 */
		CConstIterator &_Rechecked(_Unchecked_type _Right)
		{
			difference_type n_offset = _Right - &**this;
			_ASSERTE(n_offset >= 0 || n_offset + m_n_page_off >= 0); // won't skip a page back
			_ASSERTE(n_offset < 0 || n_offset < m_t_page_size - m_n_page_off); // won't skip a page ahead
			// this is experimental; if this fails, it might be needed to avoid using pointer as unchecked type and
			// instead use the iterator type itself as unchecked type, this function and the below then become no-op
			*this += n_offset;
			return *this;
		} // this would fail e.g. in std::copy as it first converts the checked iterator to unchecked and then goes back (duh)

		/**
		 *	@brief make an unchecked iterator (VS 2015)
		 *	@return Returns a pointer to array data.
		 */
		_Unchecked_type _Unchecked() const
		{
			return &**this; // pointer to the element
		}
#else // 0
		typedef CConstIterator _Unchecked_type; /**< @brief unchecked iterator type, marks a checked iterator (VS 2015) */
		
		/**
		 *	@brief reset from unchecked iterator (VS 2015)
		 *	@param[in] _Right is unchecked iterator type
		 *	@return Returns reference to this.
		 */
		CConstIterator &_Rechecked(_Unchecked_type _Right)
		{
			*this = _Right;
			return *this;
		}

		/**
		 *	@brief make an unchecked iterator (VS 2015)
		 *	@return Returns a pointer to array data.
		 */
		_Unchecked_type _Unchecked() const
		{
			return *this; // pointer to the element
		}
#endif // 0
#endif //  _MSC_VER < 1600
#endif // __SEGREGATED_MAKE_CHECKED_ITERATORS && _SECURE_SCL
#else // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		typedef std::random_access_iterator_tag iterator_category; /**< @brief iterator category (*nix) */
		// added 2017-02-15 (got some errors on Salomon, did not seem to be an issue before)
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		// more MSVC secure iterator bullshit

	protected:
		typename _TyPageList::iterator m_p_page_it; /**< @brief page iterator */
		_TyPageSize m_t_page_size; /**< @brief current page size */
		ptrdiff_t m_n_page_off; /**< @brief offset in the current page */

	private:
		/**
		 *	@brief gets page iterator
		 *	@return Returns page iterator (used by the insert() and erase() functions).
		 */
		inline typename _TyPageList::iterator p_Page_Iter() const
		{
			return m_p_page_it;
		}

		/**
		 *	@brief gets sub-page offset
		 *	@return Returns sub-page offset (used by the insert() and erase() functions).
		 */
		inline ptrdiff_t n_Page_Offset() const
		{
			return m_n_page_off;
		}

	public:
		/**
		 *	@brief default constructor; initializes a new iterator
		 *
		 *	@param[in] p_page_it is the page iterator, poiniting to the page the element pointed to resides
		 *	@param[in] t_page_size is the page size, in elements
		 *	@param[in] n_page_off is sub-page offset of the element pointed to
		 */
		inline CConstIterator(typename _TyPageList::iterator p_page_it,
			_TyPageSize t_page_size, ptrdiff_t n_page_off)
			:m_p_page_it(p_page_it), m_t_page_size(t_page_size),
			m_n_page_off(n_page_off)
		{}

		/**
		 *	@brief calculates distance between two iterators
		 *	@param[in] r_t_iter is the second iterator
		 *	@return Returns distance between this and r_t_iter, in elements.
		 */
		difference_type operator -(const CConstIterator &r_t_iter) const
		{
			_ASSERTE(m_t_page_size == r_t_iter.m_t_page_size); // iterators should belong to the same pool
			ptrdiff_t n_page_difference = m_p_page_it - r_t_iter.m_p_page_it;
			ptrdiff_t n_subpage_difference = m_n_page_off - r_t_iter.m_n_page_off;
			return n_subpage_difference + n_page_difference * m_t_page_size;
			/*
			have page size = 10

			5 - 4 = 1 (page diff = 0, subpage diff = 1)
			15 - 4 = 11 (page diff = 1, subpafe diff = 1)
			4 - 5 = -1 (page diff = 0, subpage diff = -1)
			14 - 5 = -11 (page diff = -1, subpafe diff = -1) // t_odo - test this

			seems that this should work then ...
			*/
		}

		/**
		 *	@brief compares two iterators for equality
		 *	@param[in] r_t_other is the second iterator
		 *	@return Returns true in case this and r_t_iter point
		 *		to the same element, otherwise returns false.
		 */
		inline bool operator ==(const CConstIterator &r_t_other) const
		{
			_ASSERTE(m_t_page_size == r_t_other.m_t_page_size); // iterators should belong to the same pool
			return m_n_page_off == r_t_other.m_n_page_off && // presumably cheaper
				m_p_page_it == r_t_other.m_p_page_it;
		}

		/**
		 *	@brief compares two iterators for inequality
		 *	@param[in] r_t_other is the second iterator
		 *	@return Returns true in case this iterator points to element that comes
		 *		before the one, pointed to by r_t_iter, otherwise returns false.
		 */
		inline bool operator <(const CConstIterator &r_t_other) const
		{
			_ASSERTE(m_t_page_size == r_t_other.m_t_page_size); // iterators should belong to the same pool
			return m_p_page_it < r_t_other.m_p_page_it ||
				(m_p_page_it == r_t_other.m_p_page_it &&
				m_n_page_off < r_t_other.m_n_page_off);
		}

		/**
		 *	@brief compares two iterators for inequality
		 *	@param[in] r_t_other is the second iterator
		 *	@return Returns false in case this and r_t_iter point
		 *		to the same element, otherwise returns true.
		 */
		inline bool operator !=(const CConstIterator &r_t_other) const
		{
			return !(*this == r_t_other);
		}

		/**
		 *	@brief compares two iterators for inequality
		 *	@param[in] r_t_other is the second iterator
		 *	@return Returns true in case this iterator points to element that comes
		 *		after the one, pointed to by r_t_iter, otherwise returns false.
		 */
		inline bool operator >(const CConstIterator &r_t_other) const
		{
			return (r_t_other < *this);
		}

		/**
		 *	@brief compares two iterators for inequality
		 *	@param[in] r_t_other is the second iterator
		 *	@return Returns true in case this iterator points to element that comes
		 *		not after the one, pointed to by r_t_iter, otherwise returns false.
		 */
		inline bool operator <=(const CConstIterator &r_t_other) const
		{
			return !(*this > r_t_other);
		}

		/**
		 *	@brief compares two iterators for inequality
		 *	@param[in] r_t_other is the second iterator
		 *	@return Returns true in case this iterator points to element that comes
		 *		not before the one, pointed to by r_t_iter, otherwise returns false.
		 */
		inline bool operator >=(const CConstIterator &r_t_other) const
		{
			return !(*this < r_t_other);
		}

		/**
		 *	@brief shifts the iterator to the next element
		 *	@return Returns reference to this iterator.
		 */
		CConstIterator &operator ++()
		{
			_ASSERTE(m_n_page_off >= 0);
			if(unsigned(++ m_n_page_off) == m_t_page_size) {
				m_n_page_off = 0; // equivalent to -= m_t_page_size;
				++ m_p_page_it;
			}
			return *this;
		}

		/**
		 *	@brief shifts the iterator to the previous element
		 *	@return Returns reference to this iterator.
		 */
		CConstIterator &operator --()
		{
			if(-- m_n_page_off == -1) {
				m_n_page_off += m_t_page_size;
				-- m_p_page_it;
			}
			return *this;
		}

		/**
		 *	@brief shifts the iterator to the next element
		 *	@return Returns reference to this iterator.
		 *	@note This is the post-increment function.
		 */
		CConstIterator operator ++(int)
		{
			CConstIterator copy = *this;
			++ *this;
			return copy;
		}

		/**
		 *	@brief shifts the iterator to the previous element
		 *	@return Returns reference to this iterator.
		 *	@note This is the post-increment function.
		 */
		CConstIterator operator --(int)
		{
			CConstIterator copy = *this;
			-- *this;
			return copy;
		}

		/**
		 *	@brief calculates an iterator pointing to an element, specified distance away
		 *	@param[in] n_offset is offset, in elements
		 *	@return Returns iterator pointing to the element n_offset elements away from
		 *		the element this iterator points to.
		 */
		CConstIterator operator +(difference_type n_offset) const
		{
			CConstIterator copy = *this;
			return (copy += n_offset);
		}

		/**
		 *	@brief calculates an iterator pointing to an element, specified distance away
		 *	@param[in] n_offset is negative offset, in elements
		 *	@return Returns iterator pointing to the element -n_offset elements away from
		 *		the element this iterator points to.
		 */
		CConstIterator operator -(difference_type n_offset) const
		{
			CConstIterator copy = *this;
			return (copy -= n_offset);
		}

		/**
		 *	@brief shifts this iterator to point to an element, specified distance away
		 *	@param[in] n_offset is offset, in elements
		 *	@return Returns reference to this.
		 */
		CConstIterator &operator +=(difference_type n_offset)
		{
			/*if(m_n_page_off + n_offset >= 0) {
				m_n_page_off += n_offset;
				m_p_page_it += m_n_page_off / m_t_page_size;
				m_n_page_off %= m_t_page_size;
			} else {
				m_n_page_off += n_offset;
				m_p_page_it += m_n_page_off / m_t_page_size - 1;
				m_n_page_off = m_n_page_off % m_t_page_size + m_t_page_size;
			}*/ // flawed original
			if(m_n_page_off + n_offset >= 0) {
				m_n_page_off += n_offset;
				m_p_page_it += m_n_page_off / m_t_page_size;
				m_n_page_off %= m_t_page_size;
			} else {
				m_n_page_off += n_offset;
				m_p_page_it += (m_n_page_off - difference_type(m_t_page_size) + 1) / difference_type(m_t_page_size);
				m_n_page_off = m_n_page_off % difference_type(m_t_page_size) +
					((m_n_page_off % difference_type(m_t_page_size))? difference_type(m_t_page_size) : 0);
			} // copied from CIterator
			return *this;
		}

		/**
		 *	@brief shifts this iterator to point to an element, specified distance away
		 *	@param[in] n_offset is negative offset, in elements
		 *	@return Returns reference to this.
		 */
		inline CConstIterator &operator -=(difference_type n_offset)
		{
			return (*this += -n_offset);
		}

		/**
		 *	@brief dereferences the iterator
		 *	@return Returns a reference to the element this iterator points to.
		 */
		inline reference operator *() const
		{
			_ASSERTE(m_n_page_off >= 0 && size_t(m_n_page_off) < m_t_page_size);
			return (*m_p_page_it)[m_n_page_off];
		}

		/**
		 *	@brief dereferences the iterator
		 *	@return Returns a pointer to the element this iterator points to.
		 */
		inline pointer operator->() const
		{
			return &**this;
		}
	};

	/**
	 *	@brief iterator type
	 */
	class CIterator : public CConstIterator {
	public:
		typedef _Ty *pointer; /**< @brief pointer to the payload type */
		typedef _Ty &reference; /**< @brief reference to the payload type */

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		typedef std::random_access_iterator_tag iterator_category; /**< @brief iterator category (MSVC) */
#if defined(__SEGREGATED_MAKE_CHECKED_ITERATORS) && _SECURE_SCL
#if _MSC_VER < 1600 // not sure if the max _MSC_VER is low enough; if you get errors on the line below, decrement it
		typedef std::_Range_checked_iterator_tag _Checked_iterator_category; /**< @brief checked iterator category (MSVC 90) */
#else //  _MSC_VER < 1600
#if 0
		typedef pointer _Unchecked_type; /**< @brief unchecked iterator type, marks a checked iterator (VS 2015) */

		/**
		 *	@brief reset from unchecked iterator (VS 2015)
		 *	@param[in] _Right is unchecked iterator type
		 *	@return Returns reference to this.
		 */
		CConstIterator &_Rechecked(_Unchecked_type _Right)
		{
			difference_type n_offset = _Right - &**this;
			_ASSERTE(n_offset >= 0 || n_offset + m_n_page_off >= 0); // won't skip a page back
			_ASSERTE(n_offset < 0 || n_offset < m_t_page_size - m_n_page_off); // won't skip a page ahead
			// this is experimental; if this fails, it might be needed to avoid using pointer as unchecked type and
			// instead use the iterator type itself as unchecked type, this function and the below then become no-op
			*this += n_offset;
			return *this;
		} // this would fail e.g. in std::copy as it first converts the checked iterator to unchecked and then goes back (duh)

		/**
		 *	@brief make an unchecked iterator (VS 2015)
		 *	@return Returns a pointer to array data.
		 */
		_Unchecked_type _Unchecked() const
		{
			return &**this; // pointer to the element
		}
#else // 0
		typedef CIterator _Unchecked_type; /**< @brief unchecked iterator type, marks a checked iterator (VS 2015) */
		
		/**
		 *	@brief reset from unchecked iterator (VS 2015)
		 *	@param[in] _Right is unchecked iterator type
		 *	@return Returns reference to this.
		 */
		CConstIterator &_Rechecked(_Unchecked_type _Right)
		{
			*this = _Right;
			return *this;
		}

		/**
		 *	@brief make an unchecked iterator (VS 2015)
		 *	@return Returns a pointer to array data.
		 */
		_Unchecked_type _Unchecked() const
		{
			return *this; // pointer to the element
		}
#endif // 0
#endif //  _MSC_VER < 1600
#endif // __SEGREGATED_MAKE_CHECKED_ITERATORS && _SECURE_SCL
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	public:
		/**
		 *	@copydoc CConstIterator::CConstIterator()
		 */
		inline CIterator(typename _TyPageList::iterator p_page_it,
			_TyPageSize t_page_size, ptrdiff_t n_page_off)
			:CConstIterator(p_page_it, t_page_size, n_page_off)
		{}

		/**
		 *	@brief calculates distance between two iterators
		 *	@param[in] r_t_iter is the second iterator
		 *	@return Returns distance between this and r_t_iter, in elements.
		 */
		inline typename CConstIterator::difference_type operator -(const CConstIterator &r_t_iter) const
		{
			/*_ASSERTE(m_t_page_size == r_t_iter.m_t_page_size); // iterators should belong to the same pool
			ptrdiff_t n_page_difference = CConstIterator::m_p_page_it - r_t_iter.m_p_page_it;
			ptrdiff_t n_subpage_difference = CConstIterator::m_n_page_off - r_t_iter.m_n_page_off;
			return n_subpage_difference + n_page_difference * m_t_page_size;*/
			return CConstIterator::operator -(r_t_iter); // don't repeat code
		}

		/**
		 *	@copydoc CConstIterator::operator ==()
		 */
		inline bool operator ==(const CConstIterator &r_t_other) const
		{
			_ASSERTE(CConstIterator::m_t_page_size == r_t_other.m_t_page_size); // iterators should belong to the same pool
			/*return CConstIterator::m_n_page_off == r_t_other.m_n_page_off && // presumably cheaper
				CConstIterator::m_p_page_it == r_t_other.m_p_page_it;*/
			return CConstIterator::operator ==(r_t_other); // don't repeat code
		}

		/**
		 *	@copydoc CConstIterator::operator <()
		 */
		inline bool operator <(const CConstIterator &r_t_other) const
		{
			_ASSERTE(CConstIterator::m_t_page_size == r_t_other.m_t_page_size); // iterators should belong to the same pool
			/*return CConstIterator::m_p_page_it < r_t_other.m_p_page_it ||
				(CConstIterator::m_p_page_it == r_t_other.m_p_page_it &&
				CConstIterator::m_n_page_off < r_t_other.m_n_page_off);*/
			return CConstIterator::operator <(r_t_other); // don't repeat code
		}

		/**
		 *	@copydoc CConstIterator::operator !=()
		 */
		inline bool operator !=(const CConstIterator &r_t_other) const
		{
			return !(*this == r_t_other);
		}

		/**
		 *	@copydoc CConstIterator::operator >()
		 */
		inline bool operator >(const CConstIterator &r_t_other) const
		{
			return (r_t_other < *this);
		}

		/**
		 *	@copydoc CConstIterator::operator <=()
		 */
		inline bool operator <=(const CConstIterator &r_t_other) const
		{
			return !(*this > r_t_other);
		}

		/**
		 *	@copydoc CConstIterator::operator >=()
		 */
		inline bool operator >=(const CConstIterator &r_t_other) const
		{
			return !(*this < r_t_other);
		}

		/**
		 *	@copydoc CConstIterator::operator ++()
		 */
		inline CIterator &operator ++()
		{
			/*if(++ CConstIterator::m_n_page_off == m_t_page_size) {
				CConstIterator::m_n_page_off = 0; // equivalent to -= m_t_page_size;
				++ CConstIterator::m_p_page_it;
			}*/
			CConstIterator::operator ++(); // don't repeat code
			return *this;
		}

		/**
		 *	@brief shifts the iterator to the previous element
		 *	@return Returns reference to this iterator.
		 */
		inline CIterator &operator --() // copydoc fails here
		{
			/*if(-- CConstIterator::m_n_page_off == -1) {
				CConstIterator::m_n_page_off += m_t_page_size;
				-- CConstIterator::m_p_page_it;
			}*/
			CConstIterator::operator --(); // don't repeat code
			return *this;
		}

		/**
		 *	@copydoc CConstIterator::operator ++(int)
		 */
		CIterator operator ++(int)
		{
			CIterator copy = *this;
			++ *this;
			return copy;
		}

		/**
		 *	@brief shifts the iterator to the previous element
		 *	@return Returns reference to this iterator.
		 *	@note This is the post-increment function.
		 */
		CIterator operator --(int) // copydoc fails here
		{
			CIterator copy = *this;
			-- *this;
			return copy;
		}

		/**
		 *	@copydoc CConstIterator::operator +()
		 */
		CIterator operator +(typename CConstIterator::difference_type n_offset) const
		{
			CIterator copy = *this;
			return (copy += n_offset);
		}

		/**
		 *	@brief calculates an iterator pointing to an element, specified distance away
		 *	@param[in] n_offset is negative offset, in elements
		 *	@return Returns iterator pointing to the element -n_offset elements away from
		 *		the element this iterator points to.
		 */
		CIterator operator -(typename CConstIterator::difference_type n_offset) const
		{
			CIterator copy = *this;
			return (copy -= n_offset);
		}

		/**
		 *	@copydoc CConstIterator::operator +=()
		 */
		CIterator &operator +=(typename CConstIterator::difference_type n_offset)
		{
			/*if(CConstIterator::m_n_page_off + n_offset >= 0) {
				CConstIterator::m_n_page_off += n_offset;
				CConstIterator::m_p_page_it += CConstIterator::m_n_page_off / CConstIterator::m_t_page_size;
				CConstIterator::m_n_page_off %= CConstIterator::m_t_page_size;
			} else {
				CConstIterator::m_n_page_off += n_offset;
				CConstIterator::m_p_page_it += (CConstIterator::m_n_page_off - typename CConstIterator::difference_type(CConstIterator::m_t_page_size) + 1) / typename CConstIterator::difference_type(CConstIterator::m_t_page_size);
				CConstIterator::m_n_page_off = CConstIterator::m_n_page_off % typename CConstIterator::difference_type(CConstIterator::m_t_page_size) +
					((CConstIterator::m_n_page_off % typename CConstIterator::difference_type(CConstIterator::m_t_page_size))? typename CConstIterator::difference_type(CConstIterator::m_t_page_size) : 0);
			}*/
			CConstIterator::operator +=(n_offset); // g++ makes my life hell, I call my brother ...
			return *this;
		}

		/**
		 *	@copydoc CConstIterator::operator -=()
		 */
		inline CIterator &operator -=(typename CConstIterator::difference_type n_offset)
		{
			return (*this += -n_offset);
		}

		/**
		 *	@copydoc CConstIterator::operator *()
		 */
		inline reference operator *()
		{
			_ASSERTE(CConstIterator::m_n_page_off >= 0 &&
				unsigned(CConstIterator::m_n_page_off) < CConstIterator::m_t_page_size);
			return (*CConstIterator::m_p_page_it)[CConstIterator::m_n_page_off];
		}

		/**
		 *	@copydoc CConstIterator::operator ->()
		 */
		inline pointer operator->()
		{
			return &**this;
		}
	};

	typedef CConstIterator const_iterator; /**< @brief constant iterator type name */
	typedef CIterator iterator; /**< @brief iterator type name */

protected:
	_TyPageSize m_t_page_size; /**< @brief current page size */
	size_t m_n_last_page_used; /**< @brief number of elements used in the last page (values 1 to m_t_page_size) */
	mutable _TyPageList m_page_list; /**< @brief list of currently allocated pages @note it's mutable because
		i'm trying to save code in iterators and using _TyPageList::iterator even for CConstIterator */

public:
	/**
	 *	@brief default constructor
	 *	@param[in] n_page_size_elems is run-time page size setting
	 *		(ignored in case compile-time page size was nonzero)
	 */
	inline fap_base(size_t n_page_size_elems)
		:m_n_last_page_used(n_page_size_elems)
	{
		m_t_page_size.Set(n_page_size_elems); // ignored if set at compile-time
		_ASSERTE(n_page_size_elems >= 1); // page size must be nonzero in order to be able to store data
	}

	/**
	 *	@brief copy-constructor
	 *	@param[in] r_t_pool is the other pool to be copied
	 *	@note This function throws std::bad_alloc.
	 */
	inline fap_base(const fap_base &r_t_pool) // throw(std::bad_alloc)
	{
		m_t_page_size.Set(r_t_pool.m_t_page_size);
		insert_back(r_t_pool.begin(), r_t_pool.end());
		// set page size and copy data
	}

	/**
	 *	@brief copy-operator
	 *	@param[in] r_t_pool is the other pool to be copied
	 *	@return Returns reference to this.
	 *	@note This function throws std::bad_alloc.
	 */
	inline fap_base &operator =(const fap_base &r_t_pool) // throw(std::bad_alloc)
	{
		clear();
		// clear first

		m_t_page_size.Set(r_t_pool.m_t_page_size);
		insert_back(r_t_pool.begin(), r_t_pool.end());
		// set page size and copy data

		return *this;
	}

	/**
	 *	@brief destructor
	 */
	~fap_base()
	{
		std::for_each(m_page_list.begin(), m_page_list.end(), _CPageAlloc::DeletePage);
	}

	/**
	 *	@brief empty pool predicate
	 *	@return Returns true in case the pool is empty, otherwise returns false.
	 */
	inline bool empty() const
	{
		_ASSERTE(m_n_last_page_used > 0);
		return m_page_list.empty();
	}

	/**
	 *	@brief gets number of elements in the pool
	 *	@return Returns the number of elements in the pool.
	 */
	inline size_t size() const
	{
		_ASSERTE(m_n_last_page_used > 0);
		return (!m_page_list.empty())? (m_page_list.size() - 1) * m_t_page_size + m_n_last_page_used : 0;
	}

	/**
	 *	@brief gets pool capacity, in elements
	 *	@return Returns number of elements that can be stored in the pool without allocating new pages
	 *		(this value ranges from size() up to size() + page_size() - 1).
	 */
	inline size_t capacity() const
	{
		return m_page_list.size() * m_t_page_size;
	}

	/**
	 *	@brief gets page size
	 *	@return Returns page size, in elements.
	 */
	inline size_t page_size() const
	{
		return m_t_page_size;
	}

	/**
	 *	@brief gets number of pages
	 *	@return Returns the number of currently allocated pages.
	 */
	inline size_t page_num() const
	{
		return m_page_list.size();
	}

	/**
	 *	@brief gets last page usage
	 *	@return Returns the number of elements, allocated in the last page.
	 */
	inline size_t last_page_usage() const
	{
		return m_n_last_page_used;
	}

	/*template <const int n_ctpse>
	void swap(fap_base<_Ty, n_ctpse> &r_t_other) // throw(std::runtime_error)
	{
		{
			typedef typename fap_detail::CStaticAssert<!n_ctpse || !n_compile_time_page_size_elems ||
				n_ctpse == n_compile_time_page_size_elems>::PAGE_SIZE_MISMATCH CAssert0;
		}
		// compile-time check on page size (if available; note it will fail on static page size
		// in both operands (if it was equal, the other version of swap() would be called))

		if(!n_compile_time_page_size_elems || !n_ctpse) { // should compile away
			_ASSERTE(n_compile_time_page_size_elems || n_ctpse); // otherwise the other version of swap() would be called
			// one of the operands has static size and the other has not

			if(size_t(m_t_page_size) != r_t_other.page_size())
				throw std::runtime_error("page size check failed in fap_base::swap()");
			// unable to swap unless the runitme page size equals

			// no need to swap ... those values are equal
		}
		// in case page size of either operand is dynamic, need to do the swap and perform a runtime check

		std::swap(m_n_last_page_used, r_t_other.m_n_last_page_used);
		m_page_list.swap(r_t_other.m_page_list);
	}*/ // template can't access template with different template parameters. what to do here? // t_odo // no need to mix, just leave it as is

	/**
	 *	@brief swaps two pools
	 *	@param[in,out] r_t_other is the other pool to be swapped with this
	 */
	//template <> // no template (since the above wouldn't compile)
	void swap(fap_base<_Ty, n_compile_time_page_size_elems, n_memory_alignment> &r_t_other)
	{
		if(!n_compile_time_page_size_elems) { // should compile away
			size_t n_page_size_tmp = m_t_page_size;
			m_t_page_size.Set(r_t_other.m_t_page_size);
			r_t_other.m_t_page_size.Set(n_page_size_tmp);
		}
		std::swap(m_n_last_page_used, r_t_other.m_n_last_page_used);
		m_page_list.swap(r_t_other.m_page_list);
	}

	/**
	 *	@brief element accessor
	 *	@param[in] n_index is zero-based index of the element to be returned
	 *	@return Returns reference to the element with index n_index.
	 */
	inline _Ty &operator[](size_t n_index)
	{
		_ASSERTE(n_index / m_t_page_size < m_page_list.size()); // make sure page index is valid
		_ASSERTE(n_index / m_t_page_size != m_page_list.size() - 1 ||
			n_index % m_t_page_size < m_n_last_page_used); // make sure subpage index is valid
		return m_page_list[n_index / m_t_page_size][n_index % m_t_page_size];
	}

	/**
	 *	@brief element accessor
	 *	@param[in] n_index is zero-based index of the element to be returned
	 *	@return Returns const reference to the element with index n_index.
	 */
	inline const _Ty &operator[](size_t n_index) const
	{
		_ASSERTE(n_index / m_t_page_size < m_page_list.size()); // make sure page index is valid
		_ASSERTE(n_index / m_t_page_size != m_page_list.size() - 1 ||
			n_index % m_t_page_size < m_n_last_page_used); // make sure subpage index is valid
		return m_page_list[n_index / m_t_page_size][n_index % m_t_page_size];
	}

	/**
	 *	@brief determines index of an element based on it's address in memory
	 *	@param[in] p_ptr is pointer to the element
	 *	@return Returns zero-based index of the element, or -1 in case
	 *		the element is not allocated inside this pool.
	 *	@note This is a linear-time operation in number of pages.
	 *	@note This does not check for the last page fill at run-time in release.
	 *		In case the last page is not allocated completely, and an address of an
	 *		out-of-range element from the last page is passed, this still returns
	 *		it's index, even though -1 should be returned. Applications that require
	 *		this type of check need to implement it in the caller.
	 */
	size_t index_of(const _Ty *p_ptr) const
	{
		typename _TyPageList::const_iterator p_page_it = std::find_if(m_page_list.begin(),
			m_page_list.end(), CFindPage(p_ptr, m_t_page_size));
		if(p_page_it != m_page_list.end()) {
			size_t n_index = (p_page_it - m_page_list.begin()) * m_t_page_size + (p_ptr - *p_page_it);
			_ASSERTE(n_index < size()); // make sure the element is in the valid allocated range
			_ASSERTE(&((*this)[n_index]) == p_ptr); // make sure that the index found points to the same element
			return n_index; // found
		}
		return size_t(-1); // not found
	}

	/**
	 *	@brief gets constant iterator, pointing to the first element in the pool
	 *	@return Returns constant iterator, pointing to the first element in the pool.
	 */
	inline const_iterator begin() const
	{
		return CConstIterator(m_page_list.begin(), m_t_page_size, 0);
	}

	/**
	 *	@brief gets constant iterator, pointing to one past the last element in the pool
	 *	@return Returns constant iterator, pointing to one past the last element in the pool.
	 *	@note This iterator can not be dereferenced.
	 */
	inline const_iterator end() const
	{
		return (m_n_last_page_used == m_t_page_size)?
			CConstIterator(m_page_list.end(), m_t_page_size, 0) :
			CConstIterator(m_page_list.end() - 1, m_t_page_size, m_n_last_page_used);
	}

	/**
	 *	@brief gets iterator, pointing to the first element in the pool
	 *	@return Returns iterator, pointing to the first element in the pool.
	 */
	inline iterator begin()
	{
		return CIterator(m_page_list.begin(), m_t_page_size, 0);
	}

	/**
	 *	@brief gets iterator, pointing to one past the last element in the pool
	 *	@return Returns iterator, pointing to one past the last element in the pool.
	 *	@note This iterator can not be dereferenced.
	 */
	inline iterator end()
	{
		return (m_n_last_page_used == m_t_page_size)?
			CIterator(m_page_list.end(), m_t_page_size, 0) :
			CIterator(m_page_list.end() - 1, m_t_page_size, m_n_last_page_used);
	}

	/**
	 *	@brief deletes all the elements in the pool
	 *	@note In the current implementation, this also deletes
	 *		all the memory, effectively setting capacity() to zero.
	 */
	inline void clear()
	{
		erase_back(begin());
	}

	/**
	 *	@brief deletes a range of elements
	 *
	 *	@param[in] p_begin_it is an iterator that points to the first element to be deleted
	 *	@param[in] p_end_it is an iterator that points to one past the last element to be deleted
	 *		(must point to the end of the pool)
	 *
	 *	@note This is provided just for compatibility, the elements must always
	 *		be erased from the end of the storage (meaning p_end_it must equal end()).
	 *	@note Where possible, erase_back() should be used instead.
	 */
	inline void erase(const_iterator p_begin_it, const_iterator p_end_it)
	{
		_ASSERTE(p_end_it == end());
		erase_back(p_begin_it);
	}

	/**
	 *	@brief deletes a range of elements between a given iterator and the end of the storage
	 *	@param[in] p_begin_it is an iterator that points to the first element to be deleted
	 */
	void erase_back(const_iterator p_begin_it)
	{
		_ASSERTE(p_begin_it >= begin() && p_begin_it <= end()); // make sure it's in the valid range

		{
			typename _TyPageList::iterator p_page_iter = p_begin_it.p_Page_Iter();
			_ASSERTE(p_page_iter >= m_page_list.begin() && p_page_iter <= m_page_list.end()); // must point to a valid page

			m_n_last_page_used = p_begin_it.n_Page_Offset();
			if(m_n_last_page_used) {
				_ASSERTE(p_page_iter < m_page_list.end());
				++ p_page_iter;
			} else
				m_n_last_page_used = m_t_page_size;
			// in case the last page still contains used elements, do not erase it
			// otherwise erase it and set the last page as fully used
			// mark pages after n_Page_Offset() as unused

			_ASSERTE(m_n_last_page_used > 0 && m_n_last_page_used <= m_t_page_size);
			// make sure the last page is not empty (note that can lead to some aggressive memory
			// reallocations if rapidly deleting and adding elements)

			std::for_each(p_page_iter, m_page_list.end(), _CPageAlloc::DeletePage);
			m_page_list.erase(p_page_iter, m_page_list.end());
			// delete pages from page list (and physically free the memory)
		}
		// erase all the elements from p_begin_it to the end
	}

	/**
	 *	@brief inserts a range of elements
	 *
	 *	@tparam CInputIterator is input iterator type
	 *	@param[in] p_where_it is the position before which to insert the elements
	 *		(must point to the end of the pool)
	 *	@param[in] p_begin_it is an iterator that points to the first element to be inserted
	 *	@param[in] p_end_it is an iterator that points one past the last element to be inserted
	 *
	 *	@note This is provided just for compatibility, the elements must always
	 *		be inserted at the end of the storage (meaning p_where_it must equal end()).
	 *	@note Where possible, insert_back() should be used instead.
	 *	@note This function throws std::bad_alloc.
	 *
	 *	@todo What if the input iterators point to this pool (copy of sub-range)?
	 */
	template <class CInputIterator>
	inline void insert(const_iterator UNUSED(p_where_it), CInputIterator p_begin_it, CInputIterator p_end_it) // throw(std::bad_alloc)
	{
		_ASSERTE(p_where_it == end());
		insert_back(p_begin_it, p_end_it);
	}

	/**
	 *	@brief inserts a number of copies of a single element
	 *
	 *	@param[in] p_where_it is the position before which to insert the elements
	 *		(must point to the end of the pool)
	 *	@param[in] n_count is number of copies to be inserted
	 *	@param[in] r_t_value is the value of the element(s) to be inserted
	 *
	 *	@note This is provided just for compatibility, the elements must always
	 *		be inserted at the end of the storage (meaning p_where_it must equal end()).
	 *	@note Where possible, insert_back_n() should be used instead.
	 *	@note This function throws std::bad_alloc.
	 */
	inline void insert_n(const_iterator UNUSED(p_where_it), size_t n_count, const _Ty &r_t_value) // throw(std::bad_alloc)
	{
		_ASSERTE(p_where_it == end());
		insert_back_n(n_count, r_t_value);
	}

	/**
	 *	@brief inserts a range of elements at the end of the storage
	 *
	 *	@tparam CInputIterator is input iterator type
	 *	@param[in] p_begin_it is an iterator that points to the first element to be inserted
	 *	@param[in] p_end_it is an iterator that points one past the last element to be inserted
	 *
	 *	@note This function throws std::bad_alloc.
	 *
	 *	@todo What if the input iterators point to this pool (copy of sub-range)?
	 */
	template <class CInputIterator>
	void insert_back(CInputIterator p_begin_it, CInputIterator p_end_it) // throw(std::bad_alloc)
	{
		_ASSERTE(p_end_it >= p_begin_it); // begin before end
		size_t n_count = p_end_it - p_begin_it;

		size_t n_old_size;
		grow_to((n_old_size = size()) + n_count);
		std::copy(p_begin_it, p_end_it, begin() + n_old_size); // todo - might not be optimal due to iterator arithmetic

		_ASSERTE(size() - n_old_size == n_count);
	}

	/**
	 *	@brief inserts a number of copies of a single element at the end of the storage
	 *
	 *	@param[in] n_count is number of copies to be inserted
	 *	@param[in] r_t_value is the value of the element(s) to be inserted
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	void insert_back_n(size_t n_count, const _Ty &r_t_value) // throw(std::bad_alloc)
	{
		size_t n_old_size;
		grow_to((n_old_size = size()) + n_count);
		std::fill(begin() + n_old_size, end(), r_t_value); // todo - might not be optimal due to iterator arithmetic

		_ASSERTE(size() - n_old_size == n_count);
	}

	/**
	 *	@brief inserts a single element at the end of the storage
	 *	@param[in] r_t_value is the value of the element to be inserted
	 *	@note This function throws std::bad_alloc.
	 */
	inline void push_back(const _Ty &r_t_value) // throw(std::bad_alloc)
	{
		if(!empty() && m_n_last_page_used != m_t_page_size) {
			m_page_list.back()[m_n_last_page_used] = r_t_value;
			++ m_n_last_page_used;
		} else {
			insert_back_n(1, r_t_value);
		}
	}

	/**
	 *	@brief resizes the pool to a desired number of elements
	 *
	 *	@param[in] n_new_size is the new size, in elements
	 *	@param[in] r_t_initializer is the value for the new elements,
	 *		in case n_new_size is greater than the current size
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	void resize(size_t n_new_size, const _Ty &r_t_initializer) // throw(std::bad_alloc)
	{
		size_t n_old_size;
		if((n_old_size = size()) < n_new_size) {
			grow_to(n_new_size);
			std::fill(begin() + n_old_size, end(), r_t_initializer); // todo - might not be optimal due to iterator arithmetic
		} else
			shrink_to(n_new_size);

		_ASSERTE(size() == n_new_size);
	}

	/**
	 *	@brief resizes the pool to a desired number of elements
	 *	@param[in] n_new_size is the new size, in elements
	 *	@note This function throws std::bad_alloc.
	 */
	void resize(size_t n_new_size) // throw(std::bad_alloc)
	{
		size_t n_old_size;
		if((n_old_size = size()) < n_new_size)
			grow_to(n_new_size);
		else
			shrink_to(n_new_size);

		_ASSERTE(size() == n_new_size);
	}

protected:
	/**
	 *	@brief a simple functor for searching for pages by a contained element
	 */
	class CFindPage {
	protected:
		const _Ty *m_p_ptr; /**< @brief the needle */
		_TyPageSize m_t_page_size; /**< @brief current page size, in elements */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] p_ptr is the needle
		 *	@param[in] t_page_size is current page size, in elements
		 */
		inline CFindPage(const _Ty *p_ptr, _TyPageSize t_page_size)
			:m_p_ptr(p_ptr), m_t_page_size(t_page_size)
		{}

		/**
		 *	@brief determines whether the page contains the needle
		 *	@param[in] p_ptr is pointer to the first element of the page
		 *	@return Returns true if the page contains the needle, otherwise returns false.
		 */
		inline bool operator ()(const _Ty *p_ptr) const
		{
			return m_p_ptr >= p_ptr && m_p_ptr < p_ptr + m_t_page_size;
		}
	};

	/**
	 *	@brief resizes the pool to a size, smaller than the current size
	 *	@param[in] n_new_size is the new size to resize to
	 */
	inline void shrink_to(size_t n_new_size)
	{
		_ASSERTE(n_new_size <= size());

		size_t n_new_page_num = n_new_size / m_t_page_size;
		size_t n_last_page_used = n_new_size % m_t_page_size;
		if(!n_last_page_used)
			n_last_page_used = m_t_page_size;
		else
			++ n_new_page_num;
		// calculate number of pages and number of elements in the last one

		// n_new_page_num points one past the last page that should remain allocated

		{
			std::for_each(m_page_list.begin() + n_new_page_num, m_page_list.end(),
				_CPageAlloc::DeletePage);
			m_page_list.erase(m_page_list.begin() + n_new_page_num, m_page_list.end());
			// delete pages
		}
		m_n_last_page_used = n_last_page_used;
		// resize

		_ASSERTE(size() == n_new_size);
	}

	/**
	 *	@brief resizes the pool to a size, greater than the current size
	 *	@param[in] n_new_size is the new size to resize to
	 *	@note This function throws std::bad_alloc.
	 */
	inline void grow_to(size_t n_new_size) // throw(std::bad_alloc)
	{
		_ASSERTE(n_new_size >= size());

		size_t n_new_page_num = n_new_size / m_t_page_size;
		size_t n_last_page_used = n_new_size % m_t_page_size;
		if(!n_last_page_used)
			n_last_page_used = m_t_page_size;
		else
			++ n_new_page_num;
		// calculate number of pages and number of elements in the last one

		// n_new_page_num points one past the last page that should be allocated

		{
			size_t n_old_page_num = m_page_list.size();
			m_page_list.resize(n_new_page_num);
			std::generate(m_page_list.begin() + n_old_page_num,
				m_page_list.end(), _CPageAlloc(m_t_page_size));
			// alloc new pages

			m_n_last_page_used = n_last_page_used;
		}
		// resize

		_ASSERTE(size() == n_new_size);
	}
};

} // ~fap_detail

/**
 *	@brief simple segregated storage class (a specialization for page size set at compile-time)
 *
 *	This type of storage is very fast, there is O(1) insertion time,
 *	inserting a new elements doesn't change the addresses of the old elements,
 *	nor does it invalidate the iterators (under certain conditions).
 *	The limitation is that the new elements can only be inserted to or erased
 *	from the end of the storage. Erasing should, in general, never invalidate
 *	the iterators.
 *
 *	Another limitation of the current implementation is that there is an aggressive
 *	allocation policy, causing only the immediately required memory to remain
 *	allocated. Repeated erasing and inserting of elements may therefore lead
 *	to considerable overheads due to memory allocation. This is, however, not
 *	a problem for some applications (those, where the elements are only inserted
 *	and never/only scarcely erased). Note that this problem could be easily solved
 *	by retaining the pages, at the cost of slightly complicating the allocation code
 *	and additional space requirements on the stack.
 *
 *	@tparam T is data type being stored
 *	@tparam _n_page_size_elems is page size (in elements) at compile-time, or 0 for dynamic
 *	@tparam _n_memory_align is memory alignment (in bytes)
 *
 *	@note This implementation only allows for allocation
 *		and deallocation at the end of storage (hence "forward allocated").
 */
template <class T, const int _n_page_size_elems = 0, const int _n_memory_alignment = 0> // todo - add the retain pages parameter to compensate for the aggressive deallocation policy
class forward_allocated_pool : public fap_detail::fap_base<T, _n_page_size_elems, _n_memory_alignment> {
public:
	/**
	 *	@brief default constructor; does nothing (page size was set at compile-time)
	 */
	inline forward_allocated_pool()
		:fap_detail::fap_base<T, _n_page_size_elems, _n_memory_alignment>(static_cast<size_t>(_n_page_size_elems))
	{}
};

/**
 *	@brief simple segregated storage class (a specialization for page size set at run-time)
 *
 *	This type of storage is very fast, there is O(1) insertion time,
 *	inserting a new elements doesn't change the addresses of the old elements,
 *	nor does it invalidate the iterators (under certain conditions).
 *	The limitation is that the new elements can only be inserted to or erased
 *	from the end of the storage. Erasing should, in general, never invalidate
 *	the iterators.
 *
 *	Another limitation of the current implementation is that there is an aggressive
 *	allocation policy, causing only the immediately required memory to remain
 *	allocated. Repeated erasing and inserting of elements may therefore lead
 *	to considerable overheads due to memory allocation. This is, however, not
 *	a problem for some applications (those, where the elements are only inserted
 *	and never/only scarcely erased). Note that this problem could be easily solved
 *	by retaining the pages, at the cost of slightly complicating the allocation code
 *	and additional space requirements on the stack.
 *
 *	@tparam T is data type being stored
 *	@tparam _n_page_size_elems is page size (in elements) at compile-time, or 0 for dynamic
 *	@tparam _n_memory_alignment is memory alignment (in bytes)
 *
 *	@note This implementation only allows for allocation
 *		and deallocation at the end of storage (hence "forward allocated").
 */
template <class T, const int _n_memory_alignment> // todo - add the retain pages parameter to compensate for the aggressive deallocation policy
class forward_allocated_pool<T, 0, _n_memory_alignment> : public fap_detail::fap_base<T, 0, _n_memory_alignment> {
public:
	/**
	 *	@copydoc fap_base::fap_base()
	 */
	inline forward_allocated_pool(size_t n_page_size_elems)
		:fap_detail::fap_base<T, 0, _n_memory_alignment>(n_page_size_elems)
	{}
};

/**
 *	@brief swaps values of two bit references (not the references themselves)
 *
 *	Overrides global ::swap to swap two forward allocated pools using the swap() method.
 *
 *	@param[in] r_a is the first pool reference
 *	@param[in] r_b is the second pool reference
 */
template <class T, const int n_page_size_elems, const int n_memory_alignment>
inline void swap(forward_allocated_pool<T, n_page_size_elems, n_memory_alignment> &r_a,
	forward_allocated_pool<T, n_page_size_elems, n_memory_alignment> &r_b)
{
	r_a.swap(r_b);
}

namespace std {

/**
 *	@brief swaps values of two bit references (not the references themselves)
 *
 *	Overrides std::swap to swap two forward allocated pools using the swap() method.
 *
 *	@param[in] r_a is the first pool reference
 *	@param[in] r_b is the second pool reference
 */
template <class T, const int n_page_size_elems, const int n_memory_alignment>
inline void swap(forward_allocated_pool<T, n_page_size_elems, n_memory_alignment> &r_a,
	forward_allocated_pool<T, n_page_size_elems, n_memory_alignment> &r_b)
{
	r_a.swap(r_b);
}

} // ~std

#ifdef __SEGREGATED_COMPILE_FAP_TESTS

/**
 *	@brief a collection of simple test cases for forward_allocated_pool
 *
 *	@tparam n_page_size is page size, in elements
 *	@tparam b_dynamic_page_size is dynamic page size flag
 *		(if set, page size is set at run-time, otherwise it's set at compile-time)
 *
 *	@note This is only compiled if the __SEGREGATED_COMPILE_FAP_TESTS macro is defined.
 */
template <const int n_page_size = 0, const bool b_dynamic_page_size = false>
class CFAP_Test {
public:
	typedef forward_allocated_pool<int, (b_dynamic_page_size)? 0 : n_page_size> _fap; /**< @brief forward allocated pool specialization */

	/**
	 *	@brief forward allocated pool factory
	 *	@tparam b_dynamic_page_size is dynamic page size flag
	 *		(if set, page size is set at run-time, otherwise it's set at compile-time)
	 */
	template <const int _n_page_size, const bool _b_dynamic_page_size>
	class CFapFactory {
	public:
		/**
		 *	@brief gets a new instance of forward allocated pool
		 *	@return Returns a new instance of forward allocated pool.
		 */
		static _fap get()
		{
			return _fap(_n_page_size);
		}
	};

	/**
	 *	@brief forward allocated pool factory (specializatzion for compile-time set page size)
	 */
	template <const int _n_page_size>
	class CFapFactory<_n_page_size, false> {
	public:
		/**
		 *	@brief gets a new instance of forward allocated pool
		 *	@return Returns a new instance of forward allocated pool.
		 */
		static _fap get()
		{
			return _fap();
		}
	};

	typedef CFapFactory<n_page_size, b_dynamic_page_size> _fap_factory; /**< @brief forward allocated pool factory type */

	typedef typename _fap::_Ty _Ty; /**< @brief stored elements data type */

	/**
	 *	@brief runs the test cases
	 *	@param[in] n is the size of tests (the complexity of tests is approximately O(n^2))
	 *	@return Returns true in case the tests passed, otherwise returns false.
	 */
	static bool FAP_Test(const ptrdiff_t n = 1000)
	{
		printf("empty test ...\n");
		{
			_fap fap = _fap_factory::get();
			if(!fap.empty())
				return false;
			if(fap.size() != 0)
				return false;
			if(fap.begin() != fap.end())
				return false;
		}
		// empty test

		printf("(initialized) resize / size / capacity / empty test ...\n");
		for(ptrdiff_t i = 0; i < n; ++ i) {
			{
				_fap fap = _fap_factory::get();
				fap.resize(i);
				if(fap.size() != i)
					return false;
				if(fap.capacity() < fap.size())
					return false;
				if(fap.empty() != !i)
					return false;
				if(fap.end() - fap.begin() != i)
					return false;
			}
			{
				_fap fap = _fap_factory::get();
				fap.resize(i, 0);
				if(fap.size() != i)
					return false;
				if(fap.capacity() < fap.size())
					return false;
				if(fap.empty() != !i)
					return false;
			}
			{
				_fap fap = _fap_factory::get();
				fap.insert_back_n(i, 0);
				if(fap.size() != i)
					return false;
				if(fap.capacity() < fap.size())
					return false;
				if(fap.empty() != !i)
					return false;
			}
			{
				_fap fap = _fap_factory::get();
				for(ptrdiff_t j = 0; j < i; ++ j)
					fap.push_back(0);
				if(fap.size() != i)
					return false;
				if(fap.capacity() < fap.size())
					return false;
				if(fap.empty() != !i)
					return false;
			}
			{
				_fap fap = _fap_factory::get();
				fap.insert_n(fap.end(), i, 0);
				if(fap.size() != i)
					return false;
				if(fap.capacity() < fap.size())
					return false;
				if(fap.empty() != !i)
					return false;
			}
			{
				_fap fap = _fap_factory::get();
				std::vector<_Ty> v;
				v.resize(i, 0);
				fap.insert(fap.end(), v.begin(), v.end());
				if(fap.size() != i)
					return false;
				if(fap.capacity() < fap.size())
					return false;
				if(fap.empty() != !i)
					return false;
			}
			{
				_fap fap = _fap_factory::get();
				std::vector<_Ty> v;
				v.resize(i, 0);
				fap.insert_back(v.begin(), v.end());
				if(fap.size() != i)
					return false;
				if(fap.capacity() < fap.size())
					return false;
				if(fap.empty() != !i)
					return false;
			}
		}
		// (initialized) resize / size / capacity / empty test

		printf("resize from not empty / size / capacity / empty test ...\n");
		for(ptrdiff_t i = 0; i < n; ++ i) {
			for(ptrdiff_t j = 0; j < n; ++ j) {
				{
					_fap fap = _fap_factory::get();
					fap.resize(i);
					if(fap.size() != i)
						return false;
					if(fap.capacity() < fap.size())
						return false;
					if(fap.empty() != !i)
						return false;

					fap.resize(j);
					if(fap.size() != j)
						return false;
					if(fap.capacity() < fap.size())
						return false;
					if(fap.empty() != !j)
						return false;
				}
			}
		}
		// resize from not empty / size / capacity / empty test

		printf("erase / capacity / empty test ...\n");
		for(ptrdiff_t i = 0; i < n; ++ i) {
			for(ptrdiff_t j = 0; j <= i; ++ j) {
				{
					_fap fap = _fap_factory::get();
					fap.resize(i);
					if(fap.size() != i)
						return false;
					if(fap.capacity() < fap.size())
						return false;
					if(fap.empty() != !i)
						return false;

					fap.erase_back(fap.begin() + j);
					if(fap.size() != j)
						return false;
					if(fap.capacity() < fap.size())
						return false;
					if(fap.empty() != !j)
						return false;
				}
				{
					_fap fap = _fap_factory::get();
					fap.resize(i);
					if(fap.size() != i)
						return false;
					if(fap.capacity() < fap.size())
						return false;
					if(fap.empty() != !i)
						return false;

					fap.erase(fap.begin() + j, fap.end());
					if(fap.size() != j)
						return false;
					if(fap.capacity() < fap.size())
						return false;
					if(fap.empty() != !j)
						return false;
				}
			}
			{
				_fap fap = _fap_factory::get();
				fap.resize(i);
				if(fap.size() != i)
					return false;
				if(fap.capacity() < fap.size())
					return false;
				if(fap.empty() != !i)
					return false;

				fap.clear();
				if(fap.size() != 0)
					return false;
				if(fap.capacity() != 0)
					return false;
				if(!fap.empty())
					return false;
			}
			{
				_fap fap = _fap_factory::get();
				fap.resize(i);
				if(fap.size() != i)
					return false;
				if(fap.capacity() < fap.size())
					return false;
				if(fap.empty() != !i)
					return false;

				fap.erase(fap.begin(), fap.end());
				if(fap.size() != 0)
					return false;
				if(fap.capacity() != 0)
					return false;
				if(!fap.empty())
					return false;
			}
			{
				_fap fap = _fap_factory::get();
				fap.resize(i);
				if(fap.size() != i)
					return false;
				if(fap.capacity() < fap.size())
					return false;
				if(fap.empty() != !i)
					return false;

				fap.erase_back(fap.begin());
				if(fap.size() != 0)
					return false;
				if(fap.capacity() != 0)
					return false;
				if(!fap.empty())
					return false;
			}
		}
		// erase / capacity / empty test

		printf("iterator / index_of test ...\n");
		for(ptrdiff_t i = 0; i < n; ++ i) {
			{
				_fap fap = _fap_factory::get();
				fap.resize(i);

				ptrdiff_t m = 0;
				typename _fap::iterator p_it = fap.begin();
				while(p_it != fap.end()) {
					++ p_it;
					++ m;
					if(m > i)
						return false;
				}

				if(m != i)
					return false;
			}
			{
				_fap fap = _fap_factory::get();
				fap.resize(i);

				ptrdiff_t m = 0;
				typename _fap::iterator p_it = fap.begin();
				if(p_it != fap.begin())
					return false;
				while(p_it != fap.end()) {
					if(p_it != p_it)
						return false;
					if(p_it == fap.end())
						return false;
					if(p_it > fap.end())
						return false;
					if(p_it >= fap.end())
						return false;
					if(p_it < fap.begin())
						return false;
					++ p_it;
					++ m;
					if(m > i)
						return false;
				}
				if(p_it != fap.end())
					return false;

				if(m != i)
					return false;
			}
			{
				_fap fap = _fap_factory::get();
				fap.resize(i);

				ptrdiff_t m = 0;
				typename _fap::iterator p_it = fap.begin();
				while(p_it != fap.end()) {
					{
						typename _fap::iterator p_it2 = fap.begin();
						p_it2 += m;
						if(p_it2 != p_it || p_it2 < p_it || p_it2 > p_it ||
						   !(p_it2 == p_it && p_it2 <= p_it && p_it2 >= p_it))
							return false;
					}
					{
						typename _fap::iterator p_it2 = fap.begin();
						for(int j = 0; j < m; ++ j)
							p_it2 ++;
						if(p_it2 != p_it || p_it2 < p_it || p_it2 > p_it ||
						   !(p_it2 == p_it && p_it2 <= p_it && p_it2 >= p_it))
							return false;
					}
					{
						typename _fap::iterator p_it2 = fap.begin();
						for(int j = 0; j < m; ++ j)
							++ p_it2;
						if(p_it2 != p_it || p_it2 < p_it || p_it2 > p_it ||
						   !(p_it2 == p_it && p_it2 <= p_it && p_it2 >= p_it))
							return false;
					}
					++ p_it;
					++ m;
					if(m > i)
						return false;
				}

				if(m != i)
					return false;
			}
			{
				_fap fap = _fap_factory::get();
				fap.resize(i);

				ptrdiff_t m = 0;
				typename _fap::const_iterator p_it = fap.begin();
				while(p_it != fap.end()) {
					{
						typename _fap::const_iterator p_it2 = fap.begin();
						p_it2 += m;
						if(p_it2 - p_it != 0 || p_it - p_it2 != 0)
							return false;
					}
					{
						typename _fap::iterator p_it2 = fap.begin();
						for(ptrdiff_t j = 0; j < m; ++ j) {
							if(p_it - p_it2 != m - j)
								return false;
							if(p_it2 - p_it != -(m - j))
								return false;
							if(p_it - p_it2 ++ != m - j)
								return false;
							if(p_it - p_it2 != m - j - 1)
								return false;
						}
					}
					{
						typename _fap::iterator p_it2 = fap.begin();
						for(ptrdiff_t j = 0; j < m; ++ j) {
							if(p_it - p_it2 != m - j)
								return false;
							if(p_it - ++ p_it2 != m - j - 1)
								return false;
							if(p_it - p_it2 != m - j - 1)
								return false;
						}
					}
					++ p_it;
					++ m;
					if(m > i)
						return false;
				}

				if(m != i)
					return false;
			}

			{
				_fap fap = _fap_factory::get();
				fap.resize(i);

				ptrdiff_t m = 0;
				typename _fap::iterator p_it = fap.end();
				while(p_it != fap.begin()) {
					-- p_it;
					++ m;
					if(m > i)
						return false;
				}

				if(m != i)
					return false;
			}
			{
				_fap fap = _fap_factory::get();
				fap.resize(i);

				ptrdiff_t m = 0;
				typename _fap::iterator p_it = fap.end();
				if(p_it != fap.end())
					return false;
				while(p_it != fap.begin()) {
					if(p_it != p_it)
						return false;
					if(p_it == fap.begin())
						return false;
					if(p_it < fap.begin())
						return false;
					if(p_it <= fap.begin())
						return false;
					if(p_it > fap.end())
						return false;
					-- p_it;
					++ m;
					if(m > i)
						return false;
				}
				if(p_it != fap.begin())
					return false;

				if(m != i)
					return false;
			}
			{
				_fap fap = _fap_factory::get();
				fap.resize(i);

				ptrdiff_t m = 0;
				typename _fap::iterator p_it = fap.end();
				while(p_it != fap.begin()) {
					{
						if(i == 100 && m == 1)
							m = 1;
						typename _fap::iterator p_it2 = fap.end();
						p_it2 -= m;
						if(p_it2 != p_it || p_it2 < p_it || p_it2 > p_it ||
						   !(p_it2 == p_it && p_it2 <= p_it && p_it2 >= p_it))
							return false;
					}
					{
						typename _fap::iterator p_it2 = fap.end();
						for(ptrdiff_t j = 0; j < m; ++ j)
							p_it2 --;
						if(p_it2 != p_it || p_it2 < p_it || p_it2 > p_it ||
						   !(p_it2 == p_it && p_it2 <= p_it && p_it2 >= p_it))
							return false;
					}
					{
						typename _fap::iterator p_it2 = fap.end();
						for(ptrdiff_t j = 0; j < m; ++ j)
							-- p_it2;
						if(p_it2 != p_it || p_it2 < p_it || p_it2 > p_it ||
						   !(p_it2 == p_it && p_it2 <= p_it && p_it2 >= p_it))
							return false;
					}
					-- p_it;
					++ m;
					if(m > i)
						return false;
				}

				if(m != i)
					return false;
			}
			{
				_fap fap = _fap_factory::get();
				fap.resize(i);

				ptrdiff_t m = 0;
				typename _fap::iterator p_it = fap.end();
				while(p_it != fap.begin()) {
					{
						typename _fap::iterator p_it2 = fap.end();
						p_it2 -= m;
						if(p_it2 - p_it != 0 || p_it - p_it2 != 0)
							return false;
					}
					{
						typename _fap::iterator p_it2 = fap.end();
						for(ptrdiff_t j = 0; j < m; ++ j) {
							if(p_it2 - p_it != m - j)
								return false;
							if(p_it - p_it2 != -(m - j))
								return false;
							if(p_it2 -- - p_it != m - j)
								return false;
							if(p_it2 - p_it != m - j - 1)
								return false;
						}
					}
					{
						typename _fap::iterator p_it2 = fap.end();
						for(ptrdiff_t j = 0; j < m; ++ j) {
							if(p_it2 - p_it != m - j)
								return false;
							if(-- p_it2 - p_it != m - j - 1)
								return false;
							if(p_it2 - p_it != m - j - 1)
								return false;
						}
					}
					-- p_it;
					++ m;
					if(m > i)
						return false;
				}

				if(m != i)
					return false;
			}
			{
				_fap fap = _fap_factory::get();
				fap.resize(i);

				if(i == 100)
					i = 100;

				for(ptrdiff_t j = 0; j < i; ++ j) {
					typename _fap::iterator p_it = fap.begin() + j;
					if(p_it - fap.begin() != j)
						return false;
					if(p_it < fap.begin())
						return false;
					if(j && (p_it == fap.begin() || p_it <= fap.begin() /*|| p_it >= fap.begin()*/))
						return false;
					if(!j && (p_it != fap.begin() || p_it < fap.begin() || p_it > fap.begin()))
						return false;
					typename _fap::iterator p_it2 = fap.end() + (j - i);
					if(p_it2 != p_it || !(p_it2 == p_it && p_it2 >= p_it && p_it2 <= p_it))
						return false;
					if(p_it2 - p_it != 0 || p_it - p_it2 != 0)
						return false;
				}
			}
			{
				_fap fap = _fap_factory::get();
				fap.resize(i);
				for(ptrdiff_t j = 0; j < i; ++ j)
					fap[j] = j;
				for(ptrdiff_t j = 0; j < i; ++ j) {
					if(fap[j] != j)
						return false;
				}

				for(ptrdiff_t j = 0; j < i; ++ j) {
					typename _fap::iterator p_it = fap.begin() + j;
					if(*p_it != j)
						return false;
					if(fap.index_of(&*p_it) != j)
						return false;
				}
			}
			{
				_fap fap = _fap_factory::get();
				fap.resize(i);
				std::generate(fap.begin(), fap.end(), CIota());
				for(ptrdiff_t j = 0; j < i; ++ j) {
					if(fap[j] != j)
						return false;
				}
			}
		}
		// iterator / index_of test

		printf("fap test passed ...\n");

		return true;
	}

protected:
	/**
	 *	@brief the iota function (generating of increasing sequence of integers)
	 */
	class CIota {
	protected:
		int m_n_counter; /**< @brief sequence counter */

	public:
		/**
		 *	@brief default constructor; initializes the counter to zero
		 */
		inline CIota()
			:m_n_counter(0)
		{}

		/**
		 *	@brief generates a new number in the sequence
		 *	@return Returns the value of the counter, before incrementing it.
		 */
		inline int operator ()()
		{
			int n_result = m_n_counter;
			++ m_n_counter;
			return n_result;
		}
	};
};

#endif // __SEGREGATED_COMPILE_FAP_TESTS

#endif // !__SEGREGATED_STORAGE_TEMPLATES_INCLUDED
