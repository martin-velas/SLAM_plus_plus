/**
 *	@file csparse/cs.hpp
 *	@brief this small file ensures the correct calling convention for csparse (it is compiled as "C")
 *	@note The CSparse library was not modified in any way, with the exception of addition of this file.
 */

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "csparse/cs.h"

#ifdef __cplusplus
}
#endif // __cplusplus
