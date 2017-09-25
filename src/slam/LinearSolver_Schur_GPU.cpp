/*
								+-----------------------------------+
								|                                   |
								|   ***  Schur linear solver  ***   |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2015  |
								|                                   |
								|    LinearSolver_Schur_GPU.cpp     |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/slam/LinearSolver_Schur_GPU.cpp
 *	@brief GPU routines for Schur complement solver
 *	@author -tHE SWINe-
 *	@date 2015-03-09
 */

#include "slam/LinearSolver_Schur.h"
#include "slam/MemUsage.h" // PRIsizeB

//#define __SCHUR_PROFILING // defined in slam/LinearSolver_Schur.h
// want to see profile info?

/**
 *	@def __GPU_DENSE_FALLBACK_TO_LU
 *	@brief if defined and GPU dense Cholesky fails, will try to decompose and
 *		solve using LU instead (aimed to help in case of some numerical issues,
 *		probably bad if it happens every time)
 */
#define __GPU_DENSE_FALLBACK_TO_LU

#if !defined(__DISABLE_GPU)

#include <cula.hpp>
#include <cuda.h>
#include <cublas_v2.h>
#include <cublas_api.h>
#include <cusparse_v2.h>
#include <signal.h>

#if (defined(_WIN32) || defined(_WIN64)) && defined(__GPU_EXPLICIT_LINKAGE)

//#pragma comment(lib, "cula.lib") // r3.1
//#pragma comment(lib, "cula_lapack.lib") // r17
#pragma comment(lib, "cula_core.lib") // r16a
#pragma comment(lib, "cula_lapack.lib") // r16a
// link with CULA (windows only)

#pragma comment(lib, "cuda.lib") // cuda
#pragma comment(lib, "cublas.lib") // cublas
#pragma comment(lib, "cusparse.lib") // cusparse

#endif // (_WIN32 || _WIN64) && __GPU_EXPLICIT_LINKAGE

/**
 *	@def __GPU_SCHUR_VERIFY_RESULT
 *	@brief calculates Schur on both GPU and CPU and compares them (makes it slower)
 */
//#define __GPU_SCHUR_VERIFY_RESULT

/**
 *	@def __GPU_SCHUR_EASY_PROD_ONLY
 *	@brief this only runs \f$U*(C^{-1})\f$ on the GPU, leaving \f$U*(C^{-1})*V\f$ to the CPU
 *
 *	Both branches are functional, but the full product on GPU will likely
 *	be slower than the SSE optimized implementation unless running
 *	on a high-end GPU.
 */
#define __GPU_SCHUR_EASY_PROD_ONLY

/**
 *	@def MAKE_FILE_LINE_DEBUG_STR1
 *
 *	@brief concatenates filename and line to a single "C" string
 *
 *	@param[in] a is the opening bracket
 *	@param[in] b is file name
 *	@param[in] c is comma separator
 *	@param[in] d is line number
 *	@param[in] e is the closing bracket
 */
#define MAKE_FILE_LINE_DEBUG_STR1(a,b,c,d,e) a b c #d e

/**
 *	@def MAKE_FILE_LINE_DEBUG_STR0
 *
 *	@brief concatenates filename and line to a single "C" string
 *
 *	@param[in] a is the opening bracket
 *	@param[in] b is file name
 *	@param[in] c is comma separator
 *	@param[in] d is line number
 *	@param[in] e is the closing bracket
 */
#define MAKE_FILE_LINE_DEBUG_STR0(a,b,c,d,e) MAKE_FILE_LINE_DEBUG_STR1(a, b, c, d, e)

/**
 *	@def MAKE_FILE_LINE_DEBUG_STR
 *
 *	@brief concatenates filename and line to a single "C" string
 *
 *	@param[in] f is file name
 *	@param[in] l is line number
 */
#define MAKE_FILE_LINE_DEBUG_STR(f,l) MAKE_FILE_LINE_DEBUG_STR0("(", f, ", line ", l, ")")

/**
 *	@def FILE_LINE
 *	@brief expands to a string, containing parenthesized file and line as a string literal
 */
#define FILE_LINE MAKE_FILE_LINE_DEBUG_STR(__FILE__, __LINE__)

/*
 *								=== CGPUGuard ===
 */

/**
 *	@brief a simple helper that helps to avoid crashes caused by interrupting the program while calling GPU kernels
 *
 *	At the beginning, call CGPUGuard::Register_SignalHandlers() to register signal handlers.
 *	In your program, place __GPU_Function; at the first line of each function / at the beginning
 *	of each block that is using GPU. That will mark the regions where the program should not
 *	be terminated. If e.g. SIGINT is caught, the program quits as soon as it exits all the regions
 *	that are using GPU.
 *
 *	@note This is OpenMP aware and should work even if multiple threads are using the GPU.
 *	@note The __GPU_Function presents a small overhead, and should be placed reasonably. If, on the
 *		other hand, there is a single function looping on data and using GPU, it might be a good
 *		idea to make the scope the loop iteration instead of the whole function. It is possible
 *		to use __GPU_ScopeCheck in short functions to avoid __GPU_Function overhead but to still
 *		make sure that the caller used __GPU_Function.
 */
class CGPUGuard {
protected:
	static bool b_quit; /**< @brief quit flag; set once interrupted in a GPU scope (when not possible to quit right away) */
	static size_t n_using_GPU; /**< @brief number of function recursions / blocks that are marked as using GPU */

public:
	/**
	 *	@brief default constructor; marks the beginning of a GPU scope
	 */
	CGPUGuard()
	{
		if(b_quit)
			SafeExit();
		Set_UsingGPU(true);
	}

	/**
	 *	@brief destructor; marks the end of a GPU scope
	 */
	~CGPUGuard()
	{
		Set_UsingGPU(false);
		if(b_quit)
			SafeExit();
	}

	/**
	 *	@brief GPU-aware safe exit function; quits when the GPU is not in use
	 *	@note If the GPU is in use right now, it schedules exit at the end of the current GPU scope.
	 */
	static void SafeExit()
	{
		Set_UsingGPU(false, true); // the first arg is ignored in this case
		b_quit = true; // were not able to quit now? nevermind, we'll get there
	}

	/**
	 *	@brief gets GPU scope flag
	 *	@return Returns true if the GPU is currently in use, otherwise returns false.
	 *	@note This is not thread-safe: the value might have changed since this function returned.
	 */
	static bool b_Using_GPU()
	{
		return n_using_GPU != 0;
	}

	/**
	 *	@brief registers the common program termination signal handlers
	 */
	static void Register_SignalHandlers()
	{
		signal(SIGINT, &SigInt_Handler); // ctrl+c
		signal(SIGTERM, &SigInt_Handler);
#if defined(_WIN32) || defined(_WIN64)
		signal(SIGBREAK, &SigInt_Handler); // on closing console window
#else // _WIN32 || _WIN64
		signal(SIGQUIT, &SigInt_Handler); // ctrl+\, ctrl+d
		signal(SIGSTOP, &SigInt_Handler); // ctrl+z
		signal(SIGKILL, &SigInt_Handler); // ctrl+break
#endif // _WIN32 || _WIN64
		// avoid letting the user quit the program in the middle
		// of some computation, as that sometimes crashes the GPU
	}

protected:
	/**
	 *	@brief interrupt signal handler; quits if not using GPU, or schedules a quit for when done
	 *	@param[in] n_signal is signal number (unused)
	 */
	static void SigInt_Handler(int UNUSED(n_signal))
	{
		SafeExit(); // see if we are inside a GPU function right now - if not, may as well just quit
		printf("\ninterrupted: will quit as soon as GPU processing finishes\n");
		b_quit = true;
	}

	/**
	 *	@brief the only function that accesses the GPU use counter
	 *
	 *	This is a slightly awkward design of a single function that does two things.
	 *	This is dictated by the use of a critical section instead of a mutex (all the code
	 *	that reads / writes needs to be inside this section). This makes the code
	 *	slightly simpler.
	 *
	 *	@param[in] b_using_GPU is GPU use flag (if set, the counter is incremented, otherwise decremented)
	 *	@param[in] b_quit_if_not_using is quit flag (if set, b_using_GPU is *ignored* and the program
	 *		exits if the counter is zero)
	 *
	 *	@note This does not read (or write) the b_quit flag.
	 */
	static void Set_UsingGPU(bool b_using_GPU, bool b_quit_if_not_using = false)
	{
#ifdef _OPENMP
		#pragma omp critical
#endif // _OPENMP
		{
			if(b_quit_if_not_using) {
				if(!n_using_GPU)
					exit(-1);
				// just see if GPU is in use, don't increment / decrement
			} else {
				n_using_GPU += (b_using_GPU)? 1 : -1;
				_ASSERTE(n_using_GPU != size_t(-1)); // make sure it did not underflow
			}
		}
	}
};

bool CGPUGuard::b_quit = false;
size_t CGPUGuard::n_using_GPU = 0;

/**
 *	@def GPU_FUNCGUARD_PRETTYNAME_CAT30
 *	@brief helper macro for making the names of the guard variables more random in order to avoid name colissions
 */
#define GPU_FUNCGUARD_PRETTYNAME_CAT30(a,b,c) a##b##c

/**
 *	@def GPU_FUNCGUARD_PRETTYNAME_CAT3
 *	@brief helper macro for making the names of the guard variables more random in order to avoid name colissions
 */
#define GPU_FUNCGUARD_PRETTYNAME_CAT3(a,b,c) GPU_FUNCGUARD_PRETTYNAME_CAT30(a,b,c)

/**
 *	@def __GPU_Function
 *	@brief GPU usage scope marker
 */
#define __GPU_Function CGPUGuard GPU_FUNCGUARD_PRETTYNAME_CAT3(__gpu_function_guard_, __LINE__, __)

/**
 *	@def __GPU_ScopeCheck
 *	@brief GPU usage scope checker: asserts that it is in GPU scope, only in debug
 */
#define __GPU_ScopeCheck do { _ASSERTE(CGPUGuard::b_Using_GPU()); } while(0)

/*
 *								=== ~CGPUGuard ===
 */

/*
 *								=== TSharedGPUData ===
 */

/**
 *	@brief GPU data shared by multiple instances of the solvers
 */
struct TSharedGPUData { // basically a member storage of CLinearSolver_DenseGPU, except that I don't want to modify CLinearSolver_DenseGPU itself to avoid recompiling, and also it is practically a singleton
	int n_device; /**< @brief chosen CUDA device index */
	CUdevice t_device; /**< @brief chosen CUDA device handle */
	CUcontext t_context; /**< @brief CUDA context, using the chosen device */ // simplify management by using a single context: only one context is active in SLAM++ and no context switching needs to take place
	std::vector<CUcontext> omp_context_list; /**< @brief additional CUDA contexts, using the same device, one per each thread; the first one is the same as t_context; use COMPCUDAContextGuard() to place them */
	// needed for both CUDA and CULA

	/*CUdeviceptr dense_p_lambda_storage, dense_p_rhs_storage;
	size_t dense_n_lambda_size; // affects p_lambda_storage and p_rhs_storage*/
	// CULA does not have a context like CUDA / CUBLAS / CUsparse do

	/*cusparseHandle_t t_cusparse;
	cublasHandle_t t_cublas;
	cusparseMatDescr_t t_matrix_descr, t_sym_matrix_descr;
	cs *p_A, *p_B;
	int *csrRowPtrD, csrRowPtrD_size, nnzD;*/
	// moved to CLinearSolver_Schur_GPUBase members

	/**
	 *	@brief gets an instance of the object
	 *	@return Returns a reference to the instance of the object.
	 */
	static TSharedGPUData &r_GetInstance()
	{
		static TSharedGPUData instance; // singleton
		return instance;
	}

	/**
	 *	@brief inizialize CUDA and choose a device
	 *	@note This function throws std::runtime_error.
	 */
	void CUInit() // throw(std::runtime_error)
	{
		__GPU_Function;

#ifdef _OPENMP
		#pragma omp critical
#endif // _OPENMP
		{
			do {
				if(n_device != -1)
					break;
				// already inizialized

				int n_dev_num;
				int result, dn_result;
				if((result = cuInit(0)) != CUDA_SUCCESS || (dn_result =
				   cuDeviceGetCount(&n_dev_num)) != CUDA_SUCCESS || !n_dev_num) {
					if(result != CUDA_SUCCESS)
						fprintf(stderr, "error: cuInit() returns %d\n", result);
					else if(dn_result != CUDA_SUCCESS)
						fprintf(stderr, "error: cuDeviceGetCount() returns %d\n", dn_result);
					else
						fprintf(stderr, "error: cuDeviceGetCount() found no devices\n");
					throw std::runtime_error("cuInit() failed");
				}

				size_t n_max_mem;
				for(int i = 0; i < n_dev_num; ++ i) {
					if(cuDeviceGet(&t_device, i) != CUDA_SUCCESS)
						throw std::runtime_error("cuDeviceGet() failed");
					size_t n_mem;
					if(cuDeviceTotalMem(&n_mem, t_device) != CUDA_SUCCESS)
						throw std::runtime_error("cuDeviceTotalMem() failed");
					if(!i || n_mem > n_max_mem) {
						n_max_mem = n_mem;
						n_device = i;
					}
				}
				// choose the best device, based on memory size (tends to prefer Tesla devices before GTX ones)

				if(cuDeviceGet(&t_device, n_device) != CUDA_SUCCESS)
					throw std::runtime_error("cuDeviceGet() failed");
				// get handle to the chosen device

				char p_s_dev_name[1024];
				if(cuDeviceGetName(p_s_dev_name, sizeof(p_s_dev_name) / sizeof(char) - 1, t_device) != CUDA_SUCCESS)
					throw std::runtime_error("cuDeviceGetName() failed");
				printf("debug: chosen GPU device: \'%s\' (" PRIsizeB "B RAM)\n", p_s_dev_name, PRIsizeBparams(n_max_mem));
				// print the chosen device name

				if(cuCtxCreate(&t_context, CU_CTX_SCHED_AUTO, t_device) != CUDA_SUCCESS ||
				   cuCtxSetCurrent(t_context) != CUDA_SUCCESS)
					throw std::runtime_error("cuCtxCreate() failed");
				if(cuCtxSetSharedMemConfig(CU_SHARED_MEM_CONFIG_EIGHT_BYTE_BANK_SIZE) != CUDA_SUCCESS)
					throw std::runtime_error("cuCtxSetSharedMemConfig() failed");
				// create context and request 8-byte memory banks: will work mostly with doubles

				omp_context_list.clear();
				omp_context_list.push_back(t_context);
#ifdef _OPENMP
				omp_context_list.resize(omp_get_max_threads());
				#pragma omp parallel
				{
					int tid = omp_get_thread_num();
					if(tid) {
						CUcontext &r_t_context = omp_context_list[tid];
						if(cuCtxCreate(&r_t_context, CU_CTX_SCHED_AUTO, t_device) != CUDA_SUCCESS ||
						   cuCtxSetCurrent(r_t_context) != CUDA_SUCCESS)
							throw std::runtime_error("cuCtxCreate() failed");
						if(cuCtxSetSharedMemConfig(CU_SHARED_MEM_CONFIG_EIGHT_BYTE_BANK_SIZE) != CUDA_SUCCESS)
							throw std::runtime_error("cuCtxSetSharedMemConfig() failed");
						// create context and request 8-byte memory banks: will work mostly with doubles
					}
				}
#endif // _OPENMP
			} while(0);
		}
	}

protected:
	/**
	 *	@brief default constructor; initializes empry data
	 */
	TSharedGPUData()
		:n_device(-1), t_device(0), t_context(0)
	{}

	/**
	 *	@brief destructor; frees GPU handles
	 */
	~TSharedGPUData()
	{
		__GPU_Function;

		if(t_context)
			cuCtxDestroy(t_context);
		for(size_t i = 0, n = omp_context_list.size(); i < n; ++ i) {
			if(omp_context_list[i] && omp_context_list[i] != t_context)
				cuCtxDestroy(omp_context_list[i]);
		}
		// delete cuda contexts
	}

	/**
	 *	@brief copy-constructor (not implemented)
	 *	@param[in] r_other is other instance to copy from
	 */
	TSharedGPUData(const TSharedGPUData &r_other); // no-copy

	/**
	 *	@brief copy operator (not implemented)
	 *	@param[in] r_other is other instance to copy from
	 *	@return Returns reference to this.
	 */
	TSharedGPUData &operator =(const TSharedGPUData &r_other); // no-copy
};

static TSharedGPUData &gpu = TSharedGPUData::r_GetInstance(); /**< @brief GPU data shared by dense and Schur GPU solvers */

/*
 *								=== ~TSharedGPUData ===
 */

/*
 *								=== COMPCUDAContextGuard ===
 */

COMPCUDAContextGuard::COMPCUDAContextGuard() // throw(std::runtime_error)
	:m_b_pushed(true)
{
#ifdef _OPENMP
	const int n_tid = omp_get_thread_num();
	int n_result;
	if(n_tid && (n_result = cuCtxPushCurrent(gpu.omp_context_list[n_tid])) != CUDA_SUCCESS) {
		m_b_pushed = false;
		char p_s_message[256];
		sprintf(p_s_message, "cuCtxPushCurrent() failed with %d", n_result);
		fprintf(stderr, "error: %s\n", p_s_message);
		throw std::runtime_error(p_s_message);
	}
#endif // _OPENMP
}

COMPCUDAContextGuard::COMPCUDAContextGuard(bool b_active) // throw(std::runtime_error)
	:m_b_pushed(b_active)
{
	if(!b_active)
		return; // do nothing
#ifdef _OPENMP
	const int n_tid = omp_get_thread_num();
	int n_result;
	if((n_result = cuCtxPushCurrent(gpu.omp_context_list[n_tid])) != CUDA_SUCCESS) {
		m_b_pushed = false;
		char p_s_message[256];
		sprintf(p_s_message, "cuCtxPushCurrent() failed with %d", n_result);
		fprintf(stderr, "error: %s\n", p_s_message);
		throw std::runtime_error(p_s_message);
	}
#endif // _OPENMP
}

COMPCUDAContextGuard::~COMPCUDAContextGuard() // throw(std::runtime_error)
{
#ifdef _OPENMP
	if(!m_b_pushed)
		return;
	CUcontext t_old_ctx;
	int n_result;
	const int n_tid = omp_get_thread_num();
	if(n_tid && (n_result = cuCtxPopCurrent(&t_old_ctx)) != CUDA_SUCCESS) {
		char p_s_message[256];
		sprintf(p_s_message, "cuCtxPopCurrent() failed with %d", n_result);
		fprintf(stderr, "error: %s\n", p_s_message);
		throw std::runtime_error(p_s_message);
	}
#endif // _OPENMP
}

/*
 *								=== ~COMPCUDAContextGuard ===
 */

/*
 *								=== CLinearSolver_DenseGPU ===
 */

bool CLinearSolver_DenseGPU::b_cula_initialized = false;
size_t CLinearSolver_DenseGPU::n_instance_num = 0;

CLinearSolver_DenseGPU::CLinearSolver_DenseGPU() // throw(std::runtime_error)
{
	__GPU_Function;

#ifdef _OPENMP
	//#pragma omp critical
#endif // _OPENMP
	{
#ifdef _OPENMP
		#pragma omp atomic
#endif // _OPENMP
		++ n_instance_num;
		if(!b_cula_initialized) {
			CGPUGuard::Register_SignalHandlers();
			// avoid letting the user quit the program in the middle
			// of some computation, as that sometimes crashes the GPU
			// now have to look at b_quit

			{
				gpu.CUInit();
				// initialize CUDA

				if(culaSelectDevice(gpu.n_device) != culaNoError)
					throw std::runtime_error("culaSelectDevice() failed");
				// "To bind without error, this function must be called before culaInitialize."
			}

			//printf("debug: culaInitialize()\n"); // seems to do that only once, when needed
			culaStatus s;
			switch(s = culaInitialize()) {
			case culaNoError:
				b_cula_initialized = true;
				break;
			case culaInsufficientRuntime:
				throw std::runtime_error("failed to initialize CULA: no compatible driver found");
			case culaInsufficientComputeCapability:
				throw std::runtime_error("failed to initialize CULA: no hardware with sufficient comp. cap. found");
			case culaNoHardware:
				throw std::runtime_error("failed to initialize CULA: no compatible hardware found");
			default:
				fprintf(stderr, "error: CULA error: \'%s\'\n", culaGetStatusString(s));
				{
					culaInfo i = culaGetErrorInfo();
					char p_s_error[1024];
					culaGetErrorInfoString(s, i, p_s_error, sizeof(p_s_error) / sizeof(char) - 1);
					fprintf(stderr, "error: CULA error: %d, \'%s\'\n", int(s), culaGetStatusString(s));
					fprintf(stderr, "error: CULA error info: %d, \'%s\'\n", int(i), p_s_error);
				}
				throw std::runtime_error("failed to initialize CULA: unspecified error");
			}
		}
		// the first one is in charge of initializing CULA
		// don't want to do lazy initialization here, just initialize it
		// when creating the solver so it is ready for when it is used
	}

	{
		_ASSERTE(sizeof(CUdeviceptr) == sizeof(m_gpu.p_lambda_storage));
		_ASSERTE(sizeof(CUdeviceptr) == sizeof(m_gpu.p_rhs_storage));
		//_ASSERTE(sizeof(CUdeviceptr) == sizeof(m_gpu.p_LU_perm));
		m_gpu.p_lambda_storage = 0;
		m_gpu.p_rhs_storage = 0;
		m_gpu.n_lambda_size = 0;
		//m_gpu.p_LU_perm = 0;
	}
	// initialize per-instance data (this is lazily allocated)
}

CLinearSolver_DenseGPU::CLinearSolver_DenseGPU(const CLinearSolver_DenseGPU &UNUSED(r_other))
{
	_ASSERTE(n_instance_num); // r_other exists, and already tried to initialize at this point
	++ n_instance_num;

	{
		_ASSERTE(sizeof(CUdeviceptr) == sizeof(m_gpu.p_lambda_storage));
		_ASSERTE(sizeof(CUdeviceptr) == sizeof(m_gpu.p_rhs_storage));
		//_ASSERTE(sizeof(CUdeviceptr) == sizeof(m_gpu.p_LU_perm));
		m_gpu.p_lambda_storage = 0;
		m_gpu.p_rhs_storage = 0;
		m_gpu.n_lambda_size = 0;
		//m_gpu.p_LU_perm = 0;
	}
	// initialize per-instance data (this is lazily allocated)
}

CLinearSolver_DenseGPU::~CLinearSolver_DenseGPU()
{
	__GPU_Function;

	/*{
		if(m_gpu.p_lambda_storage)
			cuMemFree((CUdeviceptr)m_gpu.p_lambda_storage);
		if(m_gpu.p_rhs_storage)
			cuMemFree((CUdeviceptr)m_gpu.p_rhs_storage);
		//if(m_gpu.p_LU_perm)
		//	cuMemFree((CUdeviceptr)m_gpu.p_LU_perm);
	}
	// delete per instance data*/

	Free_Memory(); // all in one place

#ifdef _OPENMP
	#pragma omp critical
#endif // _OPENMP
	{
		_ASSERTE(n_instance_num > 0); // this exists
		if(!(-- n_instance_num)) {
			if(b_cula_initialized) {
				b_cula_initialized = false;
				//printf("debug: culaShutdown()\n"); // seems to do that only once, when needed
				culaShutdown();
			}
		}
		// the last one takes care of the cleanup
	}
}

void CLinearSolver_DenseGPU::Free_Memory()
{
	__GPU_Function;

	{
		Eigen::MatrixXd empty;
		std::swap(empty, m_t_lambda);
	}
	// free host memory

	{
		int n_result = CUDA_SUCCESS;
		if(m_gpu.p_lambda_storage)
			n_result |= cuMemFree((CUdeviceptr)m_gpu.p_lambda_storage);
		if(m_gpu.p_rhs_storage)
			n_result |= cuMemFree((CUdeviceptr)m_gpu.p_rhs_storage);
		//if(m_gpu.p_LU_perm)
		//	n_result |= cuMemFree((CUdeviceptr)m_gpu.p_LU_perm);
		_ASSERTE(!n_result);
		m_gpu.p_lambda_storage = 0;
		m_gpu.p_rhs_storage = 0;
		m_gpu.n_lambda_size = 0;
		//m_gpu.p_LU_perm = 0;
	}
	// free GPU memory
}

bool CLinearSolver_DenseGPU::Solve_PosDef(const CUberBlockMatrix &r_lambda, Eigen::VectorXd &r_eta) // throw(std::bad_alloc, std::runtime_error)
{
	__GPU_Function;

	if(0) do {
		static bool b_first_time = true;
		if(!b_first_time)
			break;
		b_first_time = false;
		// once is enough

		cs *p_struct = cs_spalloc(r_lambda.n_BlockColumn_Num(),
			r_lambda.n_BlockColumn_Num(), 1, 1, 0);
		// alloc with values, so that it can be later converted back to a block
		// matrix (the conversion does not support binary matrices)

		p_struct = r_lambda.p_BlockStructure_to_Sparse(p_struct);
		// get block structure

		const size_t n = p_struct->n;
		std::vector<size_t> cumsums(n);
		for(size_t i = 0; i < n; ++ i)
			cumsums[i] = i + 1;
		// create cumsums of 1

		CUberBlockMatrix lam_sp(cumsums.begin(), cumsums.end(), cumsums.begin(), cumsums.end());
		_ASSERTE(lam_sp.n_Row_Num() == n && lam_sp.n_Column_Num() == n);
		// make a block matrix with 1x1 blocks and size matching that of the p_struct matrix

		cs *p_struct_tr = cs_transpose(p_struct, 1);
		cs_spfree(p_struct);
		p_struct = p_struct_tr;
		// transpose the structure, matrix market symmetric matrices are supposed to store the lower half

		std::vector<size_t> workspace;
		lam_sp.From_Sparse(0, 0, p_struct, false, workspace);
		cs_spfree(p_struct);
		// fill the block matrix with the sparse matrix

		lam_sp.Save_MatrixMarket("schur_sparsity.mtx", 0,
			"block matrix nonzero pattern", "matrix coordinate real symmetric");
		// todo - make this a CUberBlockMatrix member function (possibly in a more optimal manner)
	} while(0);
	// debug - save the schur complement sparsity pattern for further processing

	r_lambda.Convert_to_Dense(m_t_lambda);

#if 0 && defined(_DEBUG) // seems to work
	Eigen::VectorXd ref;
	{
		typedef Eigen::LLT<Eigen::MatrixXd, Eigen::Upper> _TyDecomposition; /**< @brief matrix factorization type (Cholesky) */
		_TyDecomposition m_t_factorization;

		m_t_factorization.compute(m_t_lambda);
		if(m_t_factorization.info() != Eigen::Success)
			return false;
		// Cholesky

		ref = m_t_factorization.solve(r_eta);
		// solve
	}
	// just a mockup
#endif // 0 && _DEBUG

	// on Venice:
	// Tesla K40 GPU:
	// device Cholesky		0.261068 sec / iteration, chi2 after 5 iters 234013913.66 (quite precise, I guess there is better IEEE-754)
	// Cholesky				0.297229 sec / iteration, chi2 after 5 iters 234013913.66

	// GTX 680 GPU:
	// device Cholesky		0.876127 sec / iteration, chi2 after 5 iters 234013920.81
	// Cholesky				1.018086 sec / iteration, chi2 after 5 iters 234013920.81
	// LU					2.977428 sec / iteration, chi2 after 5 iters 234084067.61 234013915.10 (for some reason very imprecise) | unable to reproduce
	// Chol + sw backsubst	1.389326 sec / iteration, chi2 after 5 iters 234013915.55
	// LU + sw backsubst	2.956637 sec / iteration, chi2 after 5 iters 234013920.97

	// CPU:
	// sparse				20 sec / iteration, chi2 after 5 iters 234013918.175338
	// LLT					9 sec / iteration, chi2 after 5 iters 234013924.73
	// LDLT					70 sec / iteration
	// PartialPivLU			19 sec / iteration // without parallelization
	// PartialPivLU			10 sec / iteration // with parallelization (8 cores), still slower than serial LLT, chi2 234013919.58
	// FullPivLU			486 sec / iteration // very slightly lower chi2
	// HouseholderQR		64 sec / iteration // pivotless

	size_t n = m_t_lambda.cols();
	_ASSERTE(m_t_lambda.rows() == n && r_eta.rows() == n);
	if(n > INT_MAX)
		throw std::runtime_error("matrix too large for CULA"); // likely wouldn't fit in the memory at this point
#if 1
	if(m_gpu.n_lambda_size < n) {
		if(m_gpu.p_lambda_storage)
			cuMemFree((CUdeviceptr)m_gpu.p_lambda_storage);
		if(m_gpu.p_rhs_storage)
			cuMemFree((CUdeviceptr)m_gpu.p_rhs_storage);
		if(cuMemAlloc((CUdeviceptr*)&m_gpu.p_lambda_storage, n * n * sizeof(double)) != CUDA_SUCCESS ||
		   cuMemAlloc((CUdeviceptr*)&m_gpu.p_rhs_storage, n * sizeof(double)) != CUDA_SUCCESS) {
			m_gpu.n_lambda_size = 0; // !!
			char p_s_error[256];
			sprintf(p_s_error, "CLinearSolver_DenseGPU::Solve_PosDef() failed to allocate"
				" memory (" PRIsize " x " PRIsize ", " PRIsize ")", n, n, r_lambda.n_NonZero_Num());
			throw std::runtime_error(p_s_error);
		}
		m_gpu.n_lambda_size = n;
	}
	// (re)allocate storage for lambda and the RHS

	if(cuMemcpyHtoDAsync((CUdeviceptr)m_gpu.p_lambda_storage, m_t_lambda.data(), n * n * sizeof(double), 0) != CUDA_SUCCESS ||
	   cuMemcpyHtoDAsync((CUdeviceptr)m_gpu.p_rhs_storage, r_eta.data(), n * sizeof(double), 0) != CUDA_SUCCESS ||
	   cuCtxSynchronize() != CUDA_SUCCESS)
		throw std::runtime_error("CLinearSolver_DenseGPU::Solve_PosDef() failed to upload data");
	// copy the data to the GPU

	culaStatus n_retval = culaDevicePosv('U', int(n), 1, (culaDeviceDouble*)m_gpu.p_lambda_storage,
		int(n), (culaDeviceDouble*)m_gpu.p_rhs_storage, int(n)); // Cholesky solve
	if(n_retval) {
		culaInfo n_info = culaGetErrorInfo();
		char p_s_error[1024];
		culaGetErrorInfoString(n_retval, n_info, p_s_error, sizeof(p_s_error) / sizeof(char) - 1);
		fprintf(stderr, "error: GPU solution returns %d (culaGetErrorInfo() returns %d, says \'%s\')\n",
			n_retval, n_info, p_s_error);
#ifdef __GPU_DENSE_FALLBACK_TO_LU
		fprintf(stderr, "error: fallback to LU\n");

		m_t_lambda.triangularView<Eigen::StrictlyLower>() =
			m_t_lambda.triangularView<Eigen::StrictlyUpper>().transpose();
		// make the matrix symmetric for GPU LU

		if(cuMemcpyHtoDAsync((CUdeviceptr)m_gpu.p_lambda_storage, m_t_lambda.data(), n * n * sizeof(double), 0) != CUDA_SUCCESS ||
		   cuMemcpyHtoDAsync((CUdeviceptr)m_gpu.p_rhs_storage, r_eta.data(), n * sizeof(double), 0) != CUDA_SUCCESS ||
		   cuCtxSynchronize() != CUDA_SUCCESS)
			throw std::runtime_error("CLinearSolver_DenseGPU::Solve_PosDef() failed to upload data");
		// copy the data to the GPU (reupload everything, just to be sure)

		std::vector<int> pivot(n);
		culaStatus n_retval = culaGesv(int(n), 1, m_t_lambda.data(), int(n), &pivot[0], r_eta.data(), int(n)); // LU solve
		if(n_retval) {
			culaInfo n_info = culaGetErrorInfo();
			culaGetErrorInfoString(n_retval, n_info, p_s_error, sizeof(p_s_error) / sizeof(char) - 1);
			fprintf(stderr, "error: GPU solution returns %d (culaGetErrorInfo() returns %d, says \'%s\')\n",
				n_retval, n_info, p_s_error);
			throw std::runtime_error("CLinearSolver_DenseGPU::Solve_PosDef() failed");
		}
		// factorize and solve on GPU using LU, may help in case the system is almost indefinite
#else // __GPU_DENSE_FALLBACK_TO_LU
		throw std::runtime_error("CLinearSolver_DenseGPU::Solve_PosDef() failed");
#endif // __GPU_DENSE_FALLBACK_TO_LU
	}
	// factorize and solve on GPU

	if(cuMemcpyDtoH(r_eta.data(), (CUdeviceptr)m_gpu.p_rhs_storage, n * sizeof(double)) != CUDA_SUCCESS)
		throw std::runtime_error("CLinearSolver_DenseGPU::Solve_PosDef() failed to download data");
	// copy only the RHS back, save time that would be spent transferring the factorization back - we don't need it
#elif 0
	culaStatus n_retval = culaPosv('U', int(n), 1, m_t_lambda.data(), int(n), r_eta.data(), int(n)); // Cholesky solve
	if(n_retval) {
		culaInfo n_info = culaGetErrorInfo();
		char p_s_error[1024];
		culaGetErrorInfoString(n_retval, n_info, p_s_error, sizeof(p_s_error) / sizeof(char) - 1);
		fprintf(stderr, "error: GPU solution returns %d (culaGetErrorInfo() returns %d, says \'%s\')\n",
			n_retval, n_info, p_s_error);
		throw std::runtime_error("CLinearSolver_DenseGPU::Solve_PosDef() failed");
	}
	// factorize and solve on GPU
#elif 1
	m_t_lambda.triangularView<Eigen::StrictlyLower>() =
		m_t_lambda.triangularView<Eigen::StrictlyUpper>().transpose();
	// make the matrix symmetric for GPU LU

	std::vector<int> pivot(n);
	culaStatus n_retval = culaGesv(int(n), 1, m_t_lambda.data(), int(n), &pivot[0], r_eta.data(), int(n)); // LU solve
	if(n_retval) {
		culaInfo n_info = culaGetErrorInfo();
		char p_s_error[1024];
		culaGetErrorInfoString(n_retval, n_info, p_s_error, sizeof(p_s_error) / sizeof(char) - 1);
		fprintf(stderr, "error: GPU solution returns %d (culaGetErrorInfo() returns %d, says \'%s\')\n",
			n_retval, n_info, p_s_error);
		throw std::runtime_error("CLinearSolver_DenseGPU::Solve_PosDef() failed");
	}
	// factorize and solve on GPU
#elif 0
	m_t_lambda.triangularView<Eigen::StrictlyLower>() =
		m_t_lambda.triangularView<Eigen::StrictlyUpper>().transpose();
	// make the matrix symmetric for GPU LU

	std::vector<int> pivot_order(n);
	culaStatus n_retval = culaGetrf(int(n), int(n), m_t_lambda.data(), int(n), &pivot_order[0]); // LU
	if(n_retval) {
		culaInfo n_info = culaGetErrorInfo();
		char p_s_error[1024];
		culaGetErrorInfoString(n_retval, n_info, p_s_error, sizeof(p_s_error) / sizeof(char) - 1);
		fprintf(stderr, "error: GPU solution returns %d (culaGetErrorInfo() returns %d, says \'%s\')\n",
			n_retval, n_info, p_s_error);
		throw std::runtime_error("CLinearSolver_DenseGPU::Solve_PosDef() failed");
	}
	// factorize on GPU

	for(size_t i = 0; i < n; ++ i)
		std::swap(r_eta(i), r_eta(pivot_order[i] - 1));
	// permute inplace using row swaps

	r_eta = m_t_lambda.triangularView<Eigen::UnitLower>().solve(r_eta); // backsubstitution
	r_eta = m_t_lambda.triangularView<Eigen::Upper>().solve(r_eta); // backsubstitution
	// solve

	// culaGetrf() performs only row pivoting, no unpermuting required at this point
#else // 1
	int n_retval = culaPotrf('U', int(n), m_t_lambda.data(), int(n)); // Cholesky
	if(n_retval == 8) {
		fprintf(stderr, "error: GPU solution returns %d (not pos def; culaGetErrorInfo() returns %d)\n", n_retval, culaGetErrorInfo());
		throw std::runtime_error("CLinearSolver_DenseGPU::Solve_PosDef(): not pos def");
	} else if(n_retval) {
		fprintf(stderr, "error: GPU solution returns %d (culaGetErrorInfo() returns %d)\n", n_retval, culaGetErrorInfo());
		throw std::runtime_error("CLinearSolver_DenseGPU::Solve_PosDef() failed");
	}
	r_eta = m_t_lambda.triangularView<Eigen::Upper>().transpose().solve(r_eta); // backsubstitution
	r_eta = m_t_lambda.triangularView<Eigen::Upper>().solve(r_eta); // backsubstitution
#endif // 1
	// do it on a GPU?

#if 0 && defined(_DEBUG) // seems to work
	printf("GPU solution error: %g\n", (ref - r_eta).norm());
#endif // 0 && _DEBUG

	return true;
}

bool CLinearSolver_DenseGPU::Dense_Inverse(double *p_dest, const double *p_src,
	const size_t n, bool b_upper_storage, bool b_try_Cholesky) // throw(std::bad_alloc, std::runtime_error)
{
	__GPU_Function;

#if 0 && defined(_DEBUG) // seems to work
	Eigen::MatrixXd ref;
	{
		typedef Eigen::PartialPivLU<Eigen::MatrixXd> _TyDecomposition; /**< @brief matrix factorization type (Cholesky) */
		_TyDecomposition t_factorization;

		Eigen::Map<const Eigen::MatrixXd> r_src(p_src, n, n);
		m_t_lambda = r_src;
		if(b_upper_storage) {
			m_t_lambda.triangularView<Eigen::StrictlyLower>() =
				r_src.transpose().triangularView<Eigen::StrictlyLower>();
		}

		t_factorization.compute(m_t_lambda);
		//if(t_factorization.info() != Eigen::Success) // always succeeds
		//	return false;
		// LU

		ref = t_factorization.inverse();
		// inverse
	}
	// just a mockup
#endif // 0 && _DEBUG

	//size_t n = r_src.cols();
	//_ASSERTE(r_src.rows() == n); // make sure it is square
	if(n > INT_MAX)
		throw std::runtime_error("matrix too large for CULA"); // likely wouldn't fit in the memory at this point
#if 1

	if(m_gpu.n_lambda_size < n) {
		if(m_gpu.p_lambda_storage)
			cuMemFree((CUdeviceptr)m_gpu.p_lambda_storage);
		if(m_gpu.p_rhs_storage)
			cuMemFree((CUdeviceptr)m_gpu.p_rhs_storage);
		//if(m_gpu.p_LU_perm)
		//	cuMemFree((CUdeviceptr)m_gpu.p_LU_perm);
		_ASSERTE(sizeof(double) >= sizeof(int)); // make sure p_LU_perm fits inside p_rhs_storage
		int n_result;
		if((n_result = cuMemAlloc((CUdeviceptr*)&m_gpu.p_lambda_storage, n * n * sizeof(double))) != CUDA_SUCCESS ||
		   (n_result = cuMemAlloc((CUdeviceptr*)&m_gpu.p_rhs_storage, n * sizeof(double))) != CUDA_SUCCESS /*||
		   cuMemAlloc((CUdeviceptr*)&m_gpu.p_LU_perm, n * sizeof(int)) != CUDA_SUCCESS*/) {
			m_gpu.n_lambda_size = 0; // !!
			char p_s_error[256];
			sprintf(p_s_error, "CLinearSolver_DenseGPU::Dense_Inverse() failed to allocate"
				" memory (" PRIsize " x " PRIsize ", " PRIsize "), result %d", n, n, n * n, n_result);
			throw std::runtime_error(p_s_error);
		}
		m_gpu.n_lambda_size = n;
	} /*else if(!m_gpu.p_LU_perm) {
		if(cuMemAlloc((CUdeviceptr*)&m_gpu.p_LU_perm, n * sizeof(int)) != CUDA_SUCCESS) {
			m_gpu.n_lambda_size = 0; // !!
			char p_s_error[256];
			sprintf(p_s_error, "CLinearSolver_DenseGPU::Solve_PosDef() failed to allocate"
				" memory (" PRIsize " x " PRIsize ", " PRIsize ")", n, n, n * n);
			throw std::runtime_error(p_s_error);
		}
		m_gpu.n_lambda_size = n;
	}*/
	// (re)allocate storage for lambda and the permutation (need to alloc the RHS as well - could just reuse it)

	if(cuMemcpyHtoDAsync((CUdeviceptr)m_gpu.p_lambda_storage, p_src, n * n * sizeof(double), 0) != CUDA_SUCCESS ||
	   cuCtxSynchronize() != CUDA_SUCCESS)
		throw std::runtime_error("CLinearSolver_DenseGPU::Dense_Inverse() failed to upload data");
	// copy the data to the GPU

	culaStatus n_retval;
	if(b_try_Cholesky) {
		n_retval = culaDevicePotrf('U', int(n), (culaDeviceDouble*)m_gpu.p_lambda_storage, int(n)); // Cholesky factorize
		if(!n_retval)
			n_retval = culaDevicePotri('U', int(n), (culaDeviceDouble*)m_gpu.p_lambda_storage, int(n)); // Cholesky invert
	}
	if(!b_try_Cholesky || n_retval) {
		if(b_try_Cholesky) {
			culaInfo n_info = culaGetErrorInfo();
			char p_s_error[1024];
			culaGetErrorInfoString(n_retval, n_info, p_s_error, sizeof(p_s_error) / sizeof(char) - 1);
			fprintf(stderr, "error: GPU inverse returns %d (culaGetErrorInfo() returns %d, says \'%s\')\n",
				n_retval, n_info, p_s_error);
#ifdef __GPU_DENSE_FALLBACK_TO_LU
			fprintf(stderr, "error: fallback to LU\n");
#else // __GPU_DENSE_FALLBACK_TO_LU
			throw std::runtime_error("CLinearSolver_DenseGPU::Solve_PosDef() failed");
#endif // __GPU_DENSE_FALLBACK_TO_LU
		}
		// did we get here by failing cholesky?

		if(b_upper_storage) {
			Eigen::Map<const Eigen::MatrixXd> r_src(p_src, n, n);
			Eigen::Map<Eigen::MatrixXd> r_dest(p_dest, n, n);
			if(p_src != p_dest) {
				//r_dest.triangularView<Eigen::Upper>() =
				//	r_src.triangularView<Eigen::Upper>();
				r_dest = r_src.selfadjointView<Eigen::Upper>();
			} else {
				r_dest.triangularView<Eigen::StrictlyLower>() =
					r_src.transpose().triangularView<Eigen::StrictlyLower>();
			}
		}
		// make the matrix full for GPU LU, without modifying src

		if(cuMemcpyHtoDAsync((CUdeviceptr)m_gpu.p_lambda_storage,
		   (b_upper_storage)? p_dest : p_src, n * n * sizeof(double), 0) != CUDA_SUCCESS ||
		   cuCtxSynchronize() != CUDA_SUCCESS)
			throw std::runtime_error("CLinearSolver_DenseGPU::Dense_Inverse() failed to upload data");
		// copy the data to the GPU (reupload everything, just to be sure)

		_ASSERTE(sizeof(double) >= sizeof(int)); // make sure p_LU_perm fits inside p_rhs_storage
		int *p_LU_perm = (int*)m_gpu.p_rhs_storage; // reuse
		culaStatus n_retval = culaDeviceGetrf(int(n), int(n), m_gpu.p_lambda_storage, int(n), /*m_gpu.*/p_LU_perm); // LU factorize
		if(!n_retval)
			n_retval = culaDeviceGetri(int(n), m_gpu.p_lambda_storage, int(n), /*m_gpu.*/p_LU_perm); // LU inverse
		if(n_retval) {
			culaInfo n_info = culaGetErrorInfo();
			char p_s_error[256];
			culaGetErrorInfoString(n_retval, n_info, p_s_error, sizeof(p_s_error) / sizeof(char) - 1);
			fprintf(stderr, "error: GPU inverse returns %d (culaGetErrorInfo() returns %d, says \'%s\')\n",
				n_retval, n_info, p_s_error);
			throw std::runtime_error("CLinearSolver_DenseGPU::Dense_Inverse() failed");
		}
		// factorize and solve on GPU using LU, may help in case the system is almost indefinite
	}
	// factorize and solve on GPU

	if(cuMemcpyDtoH(p_dest, (CUdeviceptr)m_gpu.p_lambda_storage, n * n * sizeof(double)) != CUDA_SUCCESS)
		throw std::runtime_error("CLinearSolver_DenseGPU::Dense_Inverse() failed to download data");
	// copy only the RHS back, save time that would be spent transferring the factorization back - we don't need it
#endif // 1
	// do it on a GPU?

#if 0 && defined(_DEBUG) // seems to work
	{
		Eigen::Map<Eigen::MatrixXd> r_dest(p_dest, n, n);
		printf("debug: GPU inverse error: %g\n", (ref - r_dest).norm());
	}
#endif // 0 && _DEBUG

	return true;
}

/*
 *								=== ~CLinearSolver_DenseGPU ===
 */

/*
 *								=== block matrix on GPU ===
 */

#include <map>

class CGPU_BlockMatrix : public CUberBlockMatrix {
public:
	inline size_t n_Data_Size() const
	{
		return m_data_pool.size() * sizeof(double);
	}

	// for every operation, need to create a buffer with matrix layout and strategy
	// (that can be done while the matrix data is copying)

	// ops will be probably implemented differently if only a single block size is present
	// in contrast to a mixture of block sizes

	void Get_BlockColumn_Indices(std::vector<int> &r_block_column) const
	{
		const size_t n = m_block_cols_list.size();

		r_block_column.clear();
		r_block_column.reserve(n);

		for(size_t i = 0; i < n; ++ i) {
			r_block_column.insert(r_block_column.end(),
				m_block_cols_list[i].block_list.size(), i);
		}
		// gets block column indices
	}

	// returns n + 1 numbers!
	void Get_BlockColumn_BlockNumCumsum(std::vector<int> &r_block_column) const
	{
		const size_t n = m_block_cols_list.size();

		r_block_column.clear();
		r_block_column.resize(n + 1);

		size_t n_cumsum = 0;
		for(size_t i = 0; i < n; ++ i) {
			_ASSERTE(n_cumsum <= INT_MAX);
			r_block_column[i] = (int)n_cumsum;
			n_cumsum += m_block_cols_list[i].block_list.size();
		}
		_ASSERTE(n_cumsum <= INT_MAX);
		r_block_column.back() = (int)n_cumsum; // !!
		// gets block column indices
	}

	void Get_BlockRow_Indices(std::vector<int> &r_block_row) const
	{
		const size_t n = m_block_cols_list.size();

		r_block_row.clear();
		r_block_row.reserve(n);

		for(size_t i = 0, n_dest = 0; i < n; ++ i) {
			const std::vector<TColumn::TBlockEntry> &r_col = m_block_cols_list[i].block_list;
			size_t m = r_col.size();
			_ASSERTE(n_dest == r_block_row.size());
			r_block_row.resize(n_dest + m);
			for(size_t j = 0; j < m; ++ j, ++ n_dest)
				r_block_row[n_dest] = int(r_col[j].first);
		}
		// gets block row indices
	}

	// returns block size if all blocks have same size, otherwise returns -1
	int n_Get_BlockBuffers_SquareBlocks(std::vector<int> &r_block_offset,
		std::vector<int> &r_block_size, CGPU_BlockMatrix &r_data_matrix) const
	{
		const size_t n = m_block_cols_list.size();

		r_block_size.clear();
		r_block_offset.clear();
		//r_block_size.reserve(n); // do not, will be empty for matrices with blocks with the same size
		r_block_offset.reserve(n);

		int n_block_size = (m_block_cols_list.empty())? -1 :
			int(m_block_cols_list.front().n_width); // note that this assumes that there is a block in the first block row / block column; if the matrix is sparser than that, it might be overly pedantic
		{
			size_t i = 0;
			for(; i < n; ++ i) {
				size_t n_size = m_block_cols_list[i].n_width;
				if(n_block_size != n_size) { // note that this assumes that the block column is not empty
					r_block_size.reserve(std::max(r_block_offset.size(), n)); // do it now
					r_block_size.insert(r_block_size.begin(), r_block_offset.size(), n_block_size); // insert prev size, r_block_offset.size() times
					n_block_size = -1;
					break;
				}
				//n_block_size |= -(n_block_size != n_size);

				const std::vector<TColumn::TBlockEntry> &r_block_list = m_block_cols_list[i].block_list;
				for(size_t j = 0, m = r_block_list.size(); j < m; ++ j) {
					const TColumn::TBlockEntry &r_block = r_block_list[j];
					_ASSERTE(m_block_rows_list[r_block.first].n_height == n_size); // make sure it is a square block
					size_t n_index = r_data_matrix.m_data_pool.index_of(r_block.second);
					r_block_offset.push_back(int(n_index)); // needs to have access to the pool
					_ASSERTE(n_index != size_t(-1));
				}
			}
			// loop over same size blocks

			for(; i < n; ++ i) {
				size_t n_size = m_block_cols_list[i].n_width;
				const std::vector<TColumn::TBlockEntry> &r_block_list = m_block_cols_list[i].block_list;
				for(size_t j = 0, m = r_block_list.size(); j < m; ++ j) {
					const TColumn::TBlockEntry &r_block = r_block_list[j];
					_ASSERTE(m_block_rows_list[r_block.first].n_height == n_size); // make sure it is a square block
					r_block_size.push_back(int(n_size));
					size_t n_index = r_data_matrix.m_data_pool.index_of(r_block.second);
					r_block_offset.push_back(int(n_index)); // needs to have access to the pool
					_ASSERTE(n_index != size_t(-1));
				}
			}
			// loop for blocks of varying sizes (continues since the first size
			// difference is encountered by the first loop)
		}
		// generate arrays of block offsets and sizes

		return n_block_size;
	}

	std::pair<int, int> t_Get_BlockBuffers(std::vector<int> &r_block_offset,
		std::vector<int> &r_block_rows, std::vector<int> &r_block_cols, CGPU_BlockMatrix &r_data_matrix) const
	{
		const size_t n = m_block_cols_list.size();

		r_block_rows.clear();
		r_block_cols.clear();
		r_block_offset.clear();
		/*r_block_rows.reserve(n);
		r_block_cols.reserve(n);*/ // do not, will be empty for matrices with blocks with the same size
		r_block_offset.reserve(n);

		int n_block_rows = (m_block_rows_list.empty())? -1 :
			int(m_block_rows_list.front().n_height);
		int n_block_cols = (m_block_cols_list.empty())? -1 :
			int(m_block_cols_list.front().n_width); // note that this assumes that there is a block in the first block row / block column; if the matrix is sparser than that, it might be overly pedantic
		{
			size_t i = 0;
			for(; i < n; ++ i) {
				size_t n_cols = m_block_cols_list[i].n_width;
				if(n_block_cols != n_cols) { // note that this assumes that the block column is not empty
					r_block_cols.reserve(std::max(r_block_offset.size(), n)); // do it now
					r_block_cols.insert(r_block_cols.begin(), r_block_offset.size(), n_block_cols); // insert prev size, r_block_offset.size() times
					n_block_cols = -1;
					break;
				}
				//n_block_size |= -(n_block_size != n_size);

				const std::vector<TColumn::TBlockEntry> &r_block_list = m_block_cols_list[i].block_list;
				for(size_t j = 0, m = r_block_list.size(); j < m; ++ j) {
					const TColumn::TBlockEntry &r_block = r_block_list[j];
					size_t n_rows = m_block_rows_list[r_block.first].n_height;
					if(n_block_rows != n_rows) {
						r_block_cols.reserve(std::max(r_block_offset.size(), n)); // do it now
						r_block_rows.reserve(std::max(r_block_offset.size(), n)); // do it now
						r_block_cols.insert(r_block_cols.begin(), r_block_offset.size(), n_block_cols);
						r_block_rows.insert(r_block_rows.begin(), r_block_offset.size(), n_block_rows); // insert prev size, r_block_offset.size() times
						n_block_rows = -1;
						break;
					}
					//_ASSERTE(m_block_rows_list[r_block.first].n_height == n_size); // make sure it is a square block
					size_t n_index = r_data_matrix.m_data_pool.index_of(r_block.second);
					r_block_offset.push_back(int(n_index)); // needs to have access to the pool
					_ASSERTE(n_index != size_t(-1));
				}
				if(n_block_rows == -1)
					break;
			}
			// loop over same size blocks

			for(; i < n; ++ i) {
				size_t n_cols = m_block_cols_list[i].n_width;
				const std::vector<TColumn::TBlockEntry> &r_block_list = m_block_cols_list[i].block_list;
				for(size_t j = 0, m = r_block_list.size(); j < m; ++ j) {
					const TColumn::TBlockEntry &r_block = r_block_list[j];
					size_t n_rows = m_block_rows_list[r_block.first].n_height;
					r_block_cols.push_back(int(n_cols));
					r_block_rows.push_back(int(n_rows));
					size_t n_index = r_data_matrix.m_data_pool.index_of(r_block.second);
					r_block_offset.push_back(int(n_index)); // needs to have access to the pool
					_ASSERTE(n_index != size_t(-1));
				}
			}
			// loop for blocks of varying sizes (continues since the first size
			// difference is encountered by the first loop)
		}
		// generate arrays of block offsets and sizes

		return std::make_pair(n_block_rows, n_block_cols);
		// return a pair (rows, cols)
	}

#if 0
	inline void Inverse_ThinBlockDiagonal(cl_command_queue h_cmd_queue,
		cl_mem t_data_buffer)
	{
		Inverse_ThinBlockDiagonal(h_cmd_queue, t_data_buffer, *this);
		// in case the data is in this matrix
	}

	void Inverse_ThinBlockDiagonal(cl_command_queue h_cmd_queue,
		cl_mem t_data_buffer, CGPU_BlockMatrix &r_data_matrix)
	{
		//_ASSERTE(b_SymmetricLayout()); // todo - reenable

		// note that the block_size could be optimized away if all blocks have the same size
		// (maybe even using a runtime check, but maybe need to hint to aid the compilation? need to see if loop unrolling is a way to go)

		// note that in case when the matrix is not permuted (and there is no debug bounds checking and the matrix fits in a single page / all pages are completely full), the block offset could also be optimized away
		// note that this can probably only happen in a very limited number of scenarios, and might be not so important

		// note that the page translation could be done on GPU by also passing first page elem pointers, not sure if that saves something

		// note that the addresses should be sorted for optimum performance

		// to read a matrix block, one needs to engage the whole warp in reading
		// in case the next matrix block is not within range, the reading will fail

		// each thread has an id of a block that it wants to read in the local memory
		// probably only a single block is read at a time, the logic to decide how many blocks to read would be very complicated
		// also, all the blocks still need to fit in the local memory (not a problem for robotic problems)

		// each thread copies *dest_ptr = *src_ptr
		// where dest_ptr = dest_base + block_size_cumsum[i] + interblock_off
		// and src_ptr = src_base + block_offset[i] + interblock_off
		// and basically for each thread, we need to calculate dest_ptr (src_ptr just increases)
		// the new memory controllers on fermi / kepler maybe wouldn't need strict coalescing, but just permuted one

		// need to calculate block_size_cumsum, which is just a simple scan op (and the longer the scan is, the better)
		// maybe could get the scan from the CPU, that would make it easier, and on CPU, scan is no problem

		// still need to decide which thread should read which element of which block

		// each thread has its copy sequence number (local_therad_id + local_group_size * n)
		// each thread needs to calculate both pointers; probably dest_ptr = dest_base + seq_no
		// then block_size_cumsum[i] + interblock_off = seq_no
		// then i = lower_bound(block_size_cumsum, seq_no, last_i) // divergent +/- 1 cycle, at each read
		// then interblock_off = seq_no - block_size_cumsum[i]
		// then the src_ptr would be approximately coalesced, given that the block_offset is monotonically increasing
		// this is quite bad, as it requires binary search per every read element

		// could also read every block separately, in srictly coalesced fashion (probably also the copy_? function does that)
		// then the problem is that block size % warp size != 0
		// the idle threads should ideally read the next block
		// could preprocess the block offsets on CPU to have "superblocks" which would signify contiguous
		// block for transfer by thread group. the superblocks would end at group's work-unit boundaries
		// and could also load >1 block per thread at a time, as it is simpler to decide on the CPU
		// the problem comes when the blocks are not sorted at all

		// ideally, the threads after the first block would continue with the next block (that would save
		// bandwidth, at least if the blocks are not over 32 elements apart in memory, which is often a realistic
		// assumption)

		// to do that, one needs a translation table. assume blocks 3x3, 3x3, 2x2 and 3x3
		// the block sizes are 9, 9, 4, 9 (31 elements)
		// threads read the data as follows:
		// t0 - t8 : b1
		// t9 - t17: b2
		// t18 - t21: b3
		// t22 - t30: b4
		// each thread needs block index and intra-block offset

		// what if the thread divergence in lower_bound hides in memory latency?
		// what if there is a way to avoid that divergence for a cost of some threads copying
		// elements that are already copied by different threads. is there a lower bound on that?

		// does it matter at all? will we want to do stuff with extremely large data and will anyway
		// need a streaming strategy that could take care of realigning the data?

		// note that the data is aligned for SSE processing, it could be also somehow aligned for GPU

		// also, keep in mind that those are doubles, that makes each block 18 elements,
		// and only less than two such blocks fit in a single warp read (four 2x2, less than a half of 6x6)
		// is it better to use type puning and read the doubles as pairs of uint32_t? probably.
	}
#endif // 0

	bool ProductOf_GPU(const CGPU_BlockMatrix &r_a, const CGPU_BlockMatrix &r_b,
		bool b_upper_diag_only, void *h_cublas)
	{
		__GPU_Function;

		double *dp_device_buffer_A = 0;
		double *dp_device_buffer_B;
		double *dp_device_buffer_C = 0;
		size_t n_buff_A, n_buff_B, n_buff_C = 0;
		if(cuMemAlloc((CUdeviceptr*)&dp_device_buffer_A, n_buff_A = r_a.m_data_pool.size() * sizeof(double)) ||
		   cuMemAlloc((CUdeviceptr*)&dp_device_buffer_B, n_buff_B = r_b.m_data_pool.size() * sizeof(double))) {
			if(dp_device_buffer_A)
				cuMemFree((CUdeviceptr)dp_device_buffer_A);
			return false;
		}
		// alloc memory for A and B

		if(!r_a.UploadData_to_GPU(dp_device_buffer_A, n_buff_A) ||
		   !r_b.UploadData_to_GPU(dp_device_buffer_B, n_buff_B)) {
			cuMemFree((CUdeviceptr)dp_device_buffer_A);
			cuMemFree((CUdeviceptr)dp_device_buffer_B);
			return false;
		}
		// copy the data to the GPU (asynchronous; will compute the structure of the product on the CPU in the meanwhile)

		bool b_result;
		try {
			b_result = ProductOf_GPU(r_a, r_b, b_upper_diag_only, dp_device_buffer_A,
				dp_device_buffer_B, dp_device_buffer_C, n_buff_C, h_cublas);
		} catch(std::exception &r_exc) {
			cuMemFree((CUdeviceptr)dp_device_buffer_A);
			cuMemFree((CUdeviceptr)dp_device_buffer_B);
			if(dp_device_buffer_C)
				cuMemFree((CUdeviceptr)dp_device_buffer_C);
			throw r_exc;
		}
		// multiply

		cuMemFree((CUdeviceptr)dp_device_buffer_A);
		cuMemFree((CUdeviceptr)dp_device_buffer_B);
		if(dp_device_buffer_C)
			cuMemFree((CUdeviceptr)dp_device_buffer_C);
		// delete all

		return b_result;
	}

		typedef std::pair<std::pair<int, int>, int> TBSKey;

		typedef std::map<double*, std::vector<std::pair<const double*, const double*> > > TProdMap;

		struct TMultInfo {
			int ra, ca, /*rb,*/ cb/*, rp, cp*/; // rb = ca, rp = ca, cp = cb // dimensions of blocks
			//int lda, ldb, ldc; // leading dimensions (= nums of rows), lda = ra, ldb = ca, ldc = ra

			inline int n_RowsA() const { return ra; } // number of rows
			inline int n_RowsB() const { return ca/*rb*/; }
			inline int n_RowsP() const { return ca/*rp*/; }
			inline int n_ColsA() const { return ca; } // number of columns
			inline int n_ColsB() const { return cb; }
			inline int n_ColsP() const { return cb/*cp*/; }
			inline int n_LdA() const { return n_RowsA(); } // leading dimension (column major storage)
			inline int n_LdB() const { return n_RowsB(); }
			inline int n_LdP() const { return n_RowsP(); }

			TProdMap prod_map; // the pointers to the individual arrays

			size_t n_longest_prod;

			TMultInfo(int _ra = 0, int _ca = 0, int _rb = 0, int _cb = 0)
				:ra(_ra), ca(_ca), cb(_cb), n_longest_prod(0)
			{
				_ASSERTE(_rb == _ca); // so the product is doable?
			}
			
			static TBSKey t_Key(int _ra, int _ca, int _rb, int _cb)
			{
				_ASSERTE(_rb == _ca); // so the product is doable?
				return TBSKey(std::make_pair(_ra, _ca), _cb);
			}

			TBSKey t_Key() const
			{
				return TBSKey(std::make_pair(ra, ca), cb);
			}
		};

			struct TLaunchParams {
				size_t n_first, n_count; // the first prod and the number of prods; if !n_count then this is a barrier
				TBSKey t_block_size;
			};

	bool ProductOf_GPU(const CGPU_BlockMatrix &r_a, const CGPU_BlockMatrix &r_b, bool b_upper_diag_only,
		const double *dp_device_buffer_A, const double *dp_device_buffer_B,
		double *&r_dp_device_buffer_C, size_t &r_n_device_buffer_C_size,
		void *h_cublas) // throw(std::bad_alloc)
	{
		__GPU_Function;

		r_a.CheckIntegrity(true);
		r_b.CheckIntegrity(true);

		if(r_a.m_n_col_num != r_b.m_n_row_num)
			return false;
		// check common dimension

		CDeltaTimer dt;

		CGPU_BlockMatrix &r_dest = *this;
		const CUberBlockMatrix &A = r_a;
		const CUberBlockMatrix &B = r_b;
		// name the matrices; the operation is r_dest = A * B

		r_dest.Clear();

		double *dp_C_data_base_ptr = (r_n_device_buffer_C_size)? r_dp_device_buffer_C : 0; // worry a bit about integer overflows in pointer arithmetics
		std::map<TBSKey, TMultInfo> cublas_data;
		// data for batched GEMM

		const std::vector<TRow> &r_row_list_A = A.m_block_rows_list;
		const std::vector<TRow> &r_row_list_B = B.m_block_rows_list;
		std::vector<TRow> &r_row_list_dest = r_dest.m_block_rows_list;
		const std::vector<TColumn> &r_col_list_A = A.m_block_cols_list;
		const std::vector<TColumn> &r_col_list_B = B.m_block_cols_list;
		std::vector<TColumn> &r_col_list_dest = r_dest.m_block_cols_list;
		// name the cumsums for easier access

		size_t n_dest_col_num = B.m_n_col_num, n_dest_row_num = A.m_n_row_num;
		// these are the dimensions of the new matrix

		{
			r_dest.Clear();
			r_row_list_dest = r_row_list_A; // copy row layout from this
			r_col_list_dest.resize(r_col_list_B.size());
			std::transform(r_col_list_B.begin(), r_col_list_B.end(),
				r_col_list_dest.begin(), t_ColumnCumsumCopy);
			r_dest.m_n_col_num = n_dest_col_num;
			r_dest.m_n_row_num = n_dest_row_num;
			// create layout for the destination matrix (linear time)
		}

		std::vector<size_t> reindex_rows_B_to_cols_A;
		{
			{
				std::vector<size_t> reindex_cols_A;
				size_t n_common_size = n_MergeLayout(r_col_list_A,
					r_row_list_B, reindex_cols_A, reindex_rows_B_to_cols_A);
				// merge matrix layouts (linear time in max(A column blocks, B row blocks) or something like that)

				std::vector<size_t> common(n_common_size, size_t(-2)); // helper array (note the use of -2 !!)
				for(size_t i = 0, n = reindex_cols_A.size(); i < n; ++ i) {
					size_t n_common_A;
					if((n_common_A = reindex_cols_A[i]) == size_t(-1))
						continue;
					_ASSERTE(n_common_A < common.size());
					common[n_common_A] = i;
				}
				// create inverse mapping for columns of A (linear time in number of column blocks of A)

				if(reindex_cols_A.capacity() < reindex_rows_B_to_cols_A.size())
					reindex_cols_A.clear(); // to prevent resize() on the next line from copying the data (that are going to be overwritten anyway)
				reindex_cols_A.resize(reindex_rows_B_to_cols_A.size()); // reuse this array as output (may not actually resize in most cases)
				for(size_t i = 0, n = reindex_rows_B_to_cols_A.size(); i < n; ++ i) {
					size_t n_common_B;
					if((n_common_B = reindex_rows_B_to_cols_A[i]) == size_t(-1)) {
						reindex_cols_A[i] = -1; // !!
						continue;
					}
					_ASSERTE(n_common_B < common.size());
					reindex_cols_A[i] = common[n_common_B];
				}
				reindex_cols_A.swap(reindex_rows_B_to_cols_A); // swap with the array we want output in
				// map inverse mapping of A to B (linear time in number of row blocks of B)
			}
		}
		// merge the common dimension layout (linear time)
		// -1 means the row did not map/exist in B, -2 meants it did not map/exist in A

		{
			std::vector<size_t> cols_load_list(r_col_list_dest.size(), 0);
			std::vector<std::vector<TColumn::TBlockEntry> >
				transpose_cols_list(r_row_list_dest.size());
			// list for storing transpose columns (note that the sizes and cumsums are not initialized and are invalid)

			size_t n_column_id_B = 0;
			for(_TyColumnConstIter p_col_B_it = r_col_list_B.begin(),
			   p_col_B_end_it = r_col_list_B.end(); p_col_B_it != p_col_B_end_it;
			   ++ p_col_B_it, ++ n_column_id_B) {
				const TColumn &r_t_column_B = *p_col_B_it;
				// for each column in B (index is n_column_id_B)

				for(_TyBlockConstIter p_block_B_it =
				   r_t_column_B.block_list.begin(), p_block_B_end_it = r_t_column_B.block_list.end();
				   p_block_B_it != p_block_B_end_it; ++ p_block_B_it) {
					const TColumn::TBlockEntry &r_t_block_B = *p_block_B_it;
					size_t n_row_id_B = r_t_block_B.first; // get row of the block
					// for each block in the current column in B

					const size_t n_bmB_rows = r_row_list_B[n_row_id_B].n_height, n_bmB_cols = r_t_column_B.n_width;
					//const double *p_blockB = r_t_block_B.second;
					const double *dp_blockB = dp_device_buffer_B + B.m_data_pool.index_of(r_t_block_B.second);
					// want *offsets*, to be usable on the GPU

					size_t n_column_id_A = reindex_rows_B_to_cols_A[n_row_id_B];
					_ASSERTE(size_t(-1) > size_t(-2)); // just to make sure the next line is correct
					if(n_column_id_A >= size_t(-2)) {
						if(n_column_id_A == size_t(-1))
							return false; // didn't map from B to common and we know it was not empty (we have a block here)
						continue; // do not fail, it might also mean that there are no blocks in that column in A and hence the result of multiplication is zero
					}
					_ASSERTE(n_column_id_A < r_col_list_A.size());
					const TColumn &r_t_column_A = r_col_list_A[n_column_id_A];
					_ASSERTE(r_t_column_A.n_width == r_row_list_B[n_row_id_B].n_height);
					// lookup which column in A corresponds with current block row in B

					for(_TyBlockConstIter p_block_A_it =
					   r_t_column_A.block_list.begin(), p_block_A_end_it =
					   r_t_column_A.block_list.end(); p_block_A_it != p_block_A_end_it;
					   ++ p_block_A_it) {
						const TColumn::TBlockEntry &r_t_block_A = *p_block_A_it;
						size_t n_row_id_A = r_t_block_A.first; // get row of the block
						// for each block in the current column in A

						if(b_upper_diag_only && n_row_id_A > n_column_id_B)
							break; // these only get higher, might as well break instead of continue
						// this block would've ended up at lower diagonal, don't want that

						const size_t n_bmA_rows = r_row_list_A[n_row_id_A].n_height, n_bmA_cols = r_t_column_A.n_width;
						//const double *p_blockA = r_t_block_A.second;
						const double *dp_blockA = dp_device_buffer_A + A.m_data_pool.index_of(r_t_block_A.second);
						// want *offsets*, to be usable on the GPU

						// multiplication of blockA * blockB yields block at (n_row_id_A, n_column_id_B)

						_ASSERTE(n_row_id_A < r_row_list_dest.size());
						_ASSERTE(n_column_id_B < r_col_list_dest.size());
						_ASSERTE(r_row_list_dest[n_row_id_A].n_height == n_bmA_rows);
						_ASSERTE(r_col_list_dest[n_column_id_B].n_width == n_bmB_cols);
						if(n_bmA_cols != n_bmB_rows)
							return false; // make sure the blocks are multiplicable (not able to verify by just merging the layout)
						// basic checks about matrix dimensions

						TBSKey k = TMultInfo::t_Key((int)n_bmA_rows, (int)n_bmA_cols, (int)n_bmB_rows, (int)n_bmB_cols);
						// key to the block size map (possibly there's only one, but this way we can handle VBR matrices)

						_ASSERTE(n_row_id_A < transpose_cols_list.size());
						std::vector<TColumn::TBlockEntry> &r_transpose_column =
							transpose_cols_list[n_row_id_A];
						double *dp_block_data;
						if(r_transpose_column.empty() ||
						   r_transpose_column.back().first < n_column_id_B) {
							double *p_new_block_data =
								r_dest.p_Get_DenseStorage(n_bmA_rows * n_bmB_cols);
							// get storage

							r_transpose_column.push_back(
								TColumn::TBlockEntry(n_column_id_B, p_new_block_data));
							// add it to the list

							dp_block_data = dp_C_data_base_ptr + r_dest.m_data_pool.index_of(p_new_block_data);
							// want *offsets*, to be usable on the GPU

							_ASSERTE(n_column_id_B < cols_load_list.size());
							++ cols_load_list[n_column_id_B];
							// we have a list of numbers of entries per row, that can be done in linear time and it might help speeding up the final matrix transpose later
						} else {
							_ASSERTE(r_transpose_column.back().first == n_column_id_B); // n_column_id_B is monotonically increasing, no matter what, don't have to do log(N) lookup here ;)
							double *p_block_data = r_transpose_column.back().second;

							dp_block_data = dp_C_data_base_ptr + r_dest.m_data_pool.index_of(p_block_data);
							// want *offsets*, to be usable on the GPU
						}

						////_ASSERTE(!cublas_data.count(k)); // this is a new prod, should not exist yet // can exist; this is just a new column but a prod of this size could have been there before // from top branch
						//_ASSERTE(cublas_data.count(k)); // should exist now // from bottom branch
						std::map<TBSKey, TMultInfo>::iterator p_bs_it = cublas_data.find(k);
						if(p_bs_it == cublas_data.end()) {
							p_bs_it = cublas_data.insert(std::make_pair(k,
								TMultInfo((int)n_bmA_rows, (int)n_bmA_cols, (int)n_bmB_rows, (int)n_bmB_cols))).first; // only once for each block size
						}
						TMultInfo &r_t_info = (*p_bs_it).second;
						r_t_info.prod_map[dp_block_data].push_back(std::make_pair(dp_blockA, dp_blockB));
						// add to the existing block product sum
					}
				}
			}
			// performs sparse matrix multiplication (linear time in number of blocks * constant time to insert the blocks)

			printf("debug: SpDGEMM structure: %.3f msec\n", dt.f_Time() * 1000);

			size_t n_C_buffer_size = r_dest.m_data_pool.size() * sizeof(double);
			if(!r_dp_device_buffer_C || r_n_device_buffer_C_size < n_C_buffer_size) {
				if(r_dp_device_buffer_C)
					cuMemFree((CUdeviceptr)r_dp_device_buffer_C);
				if(cuMemAlloc((CUdeviceptr*)&r_dp_device_buffer_C, n_C_buffer_size)) {
					r_n_device_buffer_C_size = 0;
					r_dp_device_buffer_C = 0;
					fprintf(stderr, "error: failed to allocate enough device memory for the product result\n");
					return false;
				}
				r_n_device_buffer_C_size = n_C_buffer_size;
			}
			// allocate buffer for the data in C

			size_t bnnz = std::accumulate(cols_load_list.begin(), cols_load_list.end(), size_t(0));
			std::vector<const double*> pA, pB, pC;
			pA.reserve(bnnz);
			pB.reserve(bnnz);
			pC.reserve(bnnz);
			// global arrays with the ptrs

			std::vector<TLaunchParams> launches;

			size_t n_wave_num = SIZE_MAX, n_longest = 0, n_first_wave_size = 0;
			for(size_t n_wave = 0; n_wave < n_wave_num; ++ n_wave) {
				for(std::map<TBSKey, TMultInfo>::iterator p_mult_it = cublas_data.begin(),
				   p_end_it = cublas_data.end(); p_mult_it != p_end_it; ++ p_mult_it) {

					size_t n_first = pC.size();

					TMultInfo &r_t_prod = (*p_mult_it).second;
					for(TProdMap::iterator p_prod_it = r_t_prod.prod_map.begin(),
					   p_prod_end_it = r_t_prod.prod_map.end(); p_prod_it != p_prod_end_it;) { // increment inside
						double *dp_dest = (*p_prod_it).first;
						std::vector<std::pair<const double*, const double*> > &r_src_list = (*p_prod_it).second;
						_ASSERTE(!r_src_list.empty());
						// get destintation and the product list

						if(r_dp_device_buffer_C != dp_C_data_base_ptr)
							dp_dest = r_dp_device_buffer_C + (dp_dest - dp_C_data_base_ptr);
						// in case the base pointer was different or unknown, need to shift it

						n_longest = std::max(n_longest, r_src_list.size());

						if(n_wave < r_src_list.size()) {
							pA.push_back(r_src_list[n_wave].first);
							pB.push_back(r_src_list[n_wave].second);
							pC.push_back(dp_dest);
						}
						// emit a product

						if(n_wave + 1 == r_src_list.size()) {
							TProdMap::iterator p_delete_it = p_prod_it;
							++ p_prod_it; // increment
							r_t_prod.prod_map.erase(p_delete_it); // delete; invalidates p_delete_it but not p_prod_it
						} else
							++ p_prod_it; // increment
						// delete this entry so that we don't spend time iterating over it the next time around
					}
					// go over all products involving blocks of one size, emit products

					size_t n_count = pC.size() - n_first;
					if(n_count) {
						TLaunchParams t_launch;
						t_launch.n_first = n_first;
						t_launch.n_count = n_count;
						t_launch.t_block_size = (*p_mult_it).first;
						launches.push_back(t_launch);
					}
					// schedule this launch to go
				}
				// peel off one product from each of the dest elements

				if(!n_wave) {
					n_first_wave_size = launches.size(); // the ones that do c = a * b rather than c += a * b
					n_wave_num = n_longest;
				}

				if(n_wave + 1 < n_longest) {
					TLaunchParams t_barrier = {0};
					launches.push_back(t_barrier);
				}
				// if there will be another wave, insert a barrier so that
				// the threads do not overwrite each other's results
			}
			// generate a list of launches which are chopped into "waves"

			printf("debug: SpDGEMM preparation: %.3f msec\n", dt.f_Time() * 1000);

			_ASSERTE(pA.size() == pB.size() && pA.size() == pC.size()); // those are triplets
			_ASSERTE(sizeof(CUdeviceptr) == sizeof(double*)); // otherwise won't work
			const size_t n_launch_data_size = pA.size() * sizeof(double*);

			double **dpA = 0, **dpB = 0, **dpC = 0;
			if(cuMemAlloc((CUdeviceptr*)&dpA, n_launch_data_size) ||
			   cuMemAlloc((CUdeviceptr*)&dpB, n_launch_data_size) ||
			   cuMemAlloc((CUdeviceptr*)&dpC, n_launch_data_size)) {
				if(dpA)
					cuMemFree((CUdeviceptr)dpA);
				if(dpB)
					cuMemFree((CUdeviceptr)dpB);
				_ASSERTE(!dpC); // what failed otherwise?
				fprintf(stderr, "error: failed to allocate enough device memory for the command buffers\n");
				return false;
			}
			if(cuMemcpyHtoDAsync((CUdeviceptr)dpA, &pA[0], n_launch_data_size, 0) ||
			   cuMemcpyHtoDAsync((CUdeviceptr)dpB, &pB[0], n_launch_data_size, 0) ||
			   cuMemcpyHtoDAsync((CUdeviceptr)dpC, &pC[0], n_launch_data_size, 0))
				fprintf(stderr, "warning: errors while cuMemcpyHtoDAsync()\n");
			// alloc and copy the launch pointers

#if defined(_DEBUG) && defined(__UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK)
			if(!r_dest.UploadData_to_GPU(r_dp_device_buffer_C, r_n_device_buffer_C_size))
				throw std::runtime_error("UploadData_to_GPU() failed " FILE_LINE);
#endif // _DEBUG && __UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK
			// in case the block bounds markers are going to be checked, need to upload
			// the empty blocks to the GPU, along with the bounds markers); this checks
			// the correctness of the CUBLAS code as a side product

#ifdef _DEBUG // otherwise get unreferenced local variable warning about n_pmode
			cublasPointerMode_t n_pmode;
			_ASSERTE(cublasGetPointerMode((cublasHandle_t)h_cublas, &n_pmode) ==
				CUBLAS_STATUS_SUCCESS && n_pmode == CUBLAS_POINTER_MODE_HOST);
#endif // _DEBUG
			// for this to work, the pointers for alpha and beta need to be read from the host

			if(cuCtxSynchronize() != CUDA_SUCCESS)
				throw std::runtime_error("cuCtxSynchronize() failed " FILE_LINE);

			for(size_t i = 0, n = launches.size(); i < n; ++ i) {
				const TLaunchParams &t_launch = launches[i];
				if(t_launch.n_count) {
					const int ra = t_launch.t_block_size.first.first; // std::make_pair(_ra, _ca), _cb
					const int ca = t_launch.t_block_size.first.second; // std::make_pair(_ra, _ca), _cb
					const int rb = ca; // otherwise wouldn't multiply
					const int cb = t_launch.t_block_size.second; // std::make_pair(_ra, _ca), _cb
					const int rp = ra;
					const int cp = cb;
					// dims of the block

					_ASSERTE(t_launch.n_count <= INT_MAX);
					const double f_alpha = 1.0, f_beta = (i < n_first_wave_size)? 0.0 : 1.0;
					cublasStatus_t n_result = cublasDgemmBatched((cublasHandle_t)h_cublas,
						CUBLAS_OP_N, CUBLAS_OP_N, ra, cb, ca, // not transpose, not transpose
						&f_alpha, const_cast<const double**>(dpA + t_launch.n_first), ra,
						const_cast<const double**>(dpB + t_launch.n_first), rb,
						&f_beta, dpC + t_launch.n_first, rp,
						(int)t_launch.n_count);
					// a launch (f_beta controls whether it is doing c = a * b (0.0) or c += a * b (1.0))
				} else {
					// a barrier; not needed here as cublas routines are blocking (but is needed for the OpenCL version)
				}
			}

			cuMemFree((CUdeviceptr)dpA);
			cuMemFree((CUdeviceptr)dpB);
			cuMemFree((CUdeviceptr)dpC);
			// delete the data

			if(!r_dest.DownloadData_from_GPU(r_dp_device_buffer_C, r_n_device_buffer_C_size))
				throw std::runtime_error("DownloadData_from_GPU() failed " FILE_LINE);
			// start downloading the dest matrix

			{
				_ASSERTE(cols_load_list.size() == r_col_list_dest.size());
				for(size_t i = 0, n = cols_load_list.size(); i < n; ++ i)
					r_col_list_dest[i].block_list.reserve(cols_load_list[i]);
				// allocate block lists

				for(size_t i = 0, n = transpose_cols_list.size(); i < n; ++ i) {
					const std::vector<TColumn::TBlockEntry> &r_col = transpose_cols_list[i];
					for(size_t j = 0, m = r_col.size(); j < m; ++ j) {
						const TColumn::TBlockEntry &r_block = r_col[j];
						_ASSERTE(r_block.first < r_col_list_dest.size());
						_ASSERTE(r_col_list_dest[r_block.first].block_list.capacity() >
							r_col_list_dest[r_block.first].block_list.size()); // since it was preallocated
						r_col_list_dest[r_block.first].block_list.push_back(
							TColumn::TBlockEntry(i, r_block.second));
					}
				}
				// performs the final transpose (linear time in number of blocks)
			}
			// this part is independent from the GPU computation, can run in the background
			// (in the hindsight, it is a waste of time to prealloc the columns then, could
			// have started the kernels slightly earlier if we didnt do it)

			if(cuCtxSynchronize() != CUDA_SUCCESS)
				throw std::runtime_error("cuCtxSynchronize() failed " FILE_LINE);

			//r_dest.CheckIntegrity();
			// makes sure the dest matrix is ok

			printf("debug: SpDGEMM execution: %.3f msec\n", dt.f_Time() * 1000);
		}

		return true;
	}

// some cuda code that we use
/*
		int *csrRowPtrA, *csrColIndA;
		double *csrValA;
		{
			int result = cuMemAlloc((CUdeviceptr*)&csrRowPtrA, sizeof(int) * (p_A->n + 1));
			result |= cuMemAlloc((CUdeviceptr*)&csrColIndA, sizeof(int) * nnzA);
			result |= cuMemAlloc((CUdeviceptr*)&csrValA, sizeof(double) * nnzA);
			if(result)
				fprintf(stderr, "error: cuMemAlloc() failed (%d, " PRIsize ")\n", result, nnzA);
			result |= cuMemcpyHtoDAsync((CUdeviceptr)csrRowPtrA, p_A->p, sizeof(int) * (p_A->n + 1), 0);
			result |= cuMemcpyHtoDAsync((CUdeviceptr)csrColIndA, p_A->i, sizeof(int) * nnzA, 0);
			result |= cuMemcpyHtoDAsync((CUdeviceptr)csrValA, p_A->x, sizeof(double) * nnzA, 0);
			if(result != CUDA_SUCCESS)
				throw std::runtime_error("failed to send the A matrix to GPU");
		}
		int *csrRowPtrB, *csrColIndB;
		double *csrValB;
		{
			int result = cuMemAlloc((CUdeviceptr*)&csrRowPtrB, sizeof(int) * (p_B->n + 1));
			result |= cuMemAlloc((CUdeviceptr*)&csrColIndB, sizeof(int) * nnzB);
			result |= cuMemAlloc((CUdeviceptr*)&csrValB, sizeof(double) * nnzB);
			if(result)
				fprintf(stderr, "error: cuMemAlloc() failed (%d, " PRIsize ")\n", result, nnzB);
			result |= cuMemcpyHtoDAsync((CUdeviceptr)csrRowPtrB, p_B->p, sizeof(int) * (p_B->n + 1), 0);
			result |= cuMemcpyHtoDAsync((CUdeviceptr)csrColIndB, p_B->i, sizeof(int) * nnzB, 0);
			result |= cuMemcpyHtoDAsync((CUdeviceptr)csrValB, p_B->x, sizeof(double) * nnzB, 0);
			if(result != CUDA_SUCCESS)
				throw std::runtime_error("failed to send the B matrix to GPU");
		}
		// B is symmetric - the same in SCR and CSC
		// todo - see which combination of transpose flags yields faster results

		if(cuCtxSynchronize() != CUDA_SUCCESS)
			throw std::runtime_error("cuCtxSynchronize() failed " FILE_LINE);

		//printf("debug: have matrices in GPU\n"); // debug

		// p_A, p_B are not needed anymore // todo - reuse for the second product

		{
			const double f_x = -1;
			_ASSERTE(nnzA < INT_MAX);
			cublasDscal((cublasHandle_t)m_gpu.t_cublas, int(nnzA), &f_x, csrValA, 1);
		}
		// use cublasDscal() to flip signs on A (A = C^-1)
*/

	bool UploadData_to_GPU(double *dp_buffer, size_t n_buffer_size, bool b_blocking = false) const
	{
		_ASSERTE(!m_n_ref_elem_num); // if this triggers, make a deep copy of the matrix
		// must upload data of the original matrix, *will not* work on the ref matrices (would lose performance)

		const size_t n_data_size = m_data_pool.size() * sizeof(double);
		if(n_buffer_size < n_data_size)
			return false;

		int n_result = CUDA_SUCCESS;
		_ASSERTE(!n_result); // if not 0, then |= won't work

		if(n_data_size) {
			const size_t n_page_size = m_data_pool.page_size(),
				n_page_num_1 = m_data_pool.page_num() - 1;
			for(size_t i = 0; i < n_page_num_1; ++ i, dp_buffer += n_page_size) {
				const double *p_data = &m_data_pool[i * n_page_size];

				if(b_blocking)
					n_result |= cuMemcpyHtoD((CUdeviceptr)dp_buffer, p_data, sizeof(double) * n_page_size);
				else
					n_result |= cuMemcpyHtoDAsync((CUdeviceptr)dp_buffer, p_data, sizeof(double) * n_page_size, 0); // the last arg is stream handle
			}
			// upload full pages

			{
				const double *p_last_data = &m_data_pool[n_page_num_1 * n_page_size];
				size_t n_last_page_size = m_data_pool.last_page_usage();

				if(b_blocking)
					n_result |= cuMemcpyHtoD((CUdeviceptr)dp_buffer, p_last_data, sizeof(double) * n_last_page_size);
				else
					n_result |= cuMemcpyHtoDAsync((CUdeviceptr)dp_buffer, p_last_data, sizeof(double) * n_last_page_size, 0); // the last arg is stream handle
			}
			// upload the last page
		}
		// schedule upload of every page

		return n_result == CUDA_SUCCESS;
	}

	// n_buffer_offset is in bytes
	bool DownloadData_from_GPU(double *dp_buffer, size_t n_buffer_size, bool b_blocking = false)
	{
		_ASSERTE(!m_n_ref_elem_num); // if this triggers, make a deep copy of the matrix
		// must upload data of the original matrix, *will not* work on the ref matrices (would lose performance)

		const size_t n_data_size = m_data_pool.size() * sizeof(double);
		if(n_buffer_size < n_data_size)
			return false;

		int n_result = CUDA_SUCCESS;
		_ASSERTE(!n_result); // if not 0, then |= won't work

		if(n_data_size) {
			const size_t n_page_size = m_data_pool.page_size(),
				n_page_num_1 = m_data_pool.page_num() - 1;
			for(size_t i = 0; i < n_page_num_1; ++ i, dp_buffer += n_page_size) {
				double *p_data = &m_data_pool[i * n_page_size];

				if(b_blocking)
					n_result |= cuMemcpyDtoH(p_data, (CUdeviceptr)dp_buffer, sizeof(double) * n_page_size);
				else
					n_result |= cuMemcpyDtoHAsync(p_data, (CUdeviceptr)dp_buffer, sizeof(double) * n_page_size, 0); // the last arg is stream handle
			}
			// download full pages

			{
				double *p_last_data = &m_data_pool[n_page_num_1 * n_page_size];
				size_t n_last_page_size = m_data_pool.last_page_usage();

				if(b_blocking)
					n_result |= cuMemcpyDtoH(p_last_data, (CUdeviceptr)dp_buffer, sizeof(double) * n_last_page_size);
				else
					n_result |= cuMemcpyDtoHAsync(p_last_data, (CUdeviceptr)dp_buffer, sizeof(double) * n_last_page_size, 0); // the last arg is stream handle
			}
			// download the last page
		}
		// schedule upload of every page

		return n_result == CUDA_SUCCESS;
	}

#if 0
	// n_buffer_offset is in bytes
	bool UploadData_to_GPU(cl_command_queue h_cmd_queue, cl_mem t_buffer, size_t n_buffer_size,
		size_t n_buffer_offset = 0, bool b_blocking = false, cl_event *p_event = 0) const
	{
		_ASSERTE(!m_n_ref_elem_num);
		// must upload data of the original matrix, does not work on the ref matrices (so far)

		size_t n_data_size = m_data_pool.size() * sizeof(double);

		_ASSERTE(n_buffer_offset <= n_buffer_size);
		if(n_buffer_size - n_buffer_offset < n_data_size)
			return false;

		if(!m_data_pool.empty()) {
			size_t n_page_size = m_data_pool.page_size(),
				n_page_num_1 = m_data_pool.page_num() - 1;
			for(size_t i = 0; i < n_page_num_1; ++ i) {
				const double *p_data = &m_data_pool[i * n_page_size];

				clEnqueueWriteBuffer(h_cmd_queue, t_buffer, b_blocking,
					n_buffer_offset, n_page_size * sizeof(double), p_data, 0, 0, 0);

				n_buffer_offset += n_page_size * sizeof(double);
			}
			// upload full pages

			{
				const double *p_last_data = &m_data_pool[n_page_num_1 * n_page_size];
				size_t n_last_page_size = m_data_pool.last_page_usage();

				clEnqueueWriteBuffer(h_cmd_queue, t_buffer, b_blocking,
					n_buffer_offset, n_last_page_size * sizeof(double),
					p_last_data, 0, 0, p_event);
			}
			// upload the last page
		}
		// schedule upload of every page

		return true;
	}

	// n_buffer_offset is in bytes
	bool DownloadData_from_GPU(cl_command_queue h_cmd_queue, cl_mem t_buffer, size_t n_buffer_size,
		size_t n_buffer_offset = 0, bool b_blocking = false, cl_event *p_event = 0)
	{
		_ASSERTE(!m_n_ref_elem_num);
		// must upload data of the original matrix, does not work on the ref matrices (so far)

		size_t n_data_size = m_data_pool.size() * sizeof(double);

		_ASSERTE(n_buffer_offset <= n_buffer_size);
		if(n_buffer_size - n_buffer_offset < n_data_size)
			return false;

		if(!m_data_pool.empty()) {
			size_t n_page_size = m_data_pool.page_size(),
				n_page_num_1 = m_data_pool.page_num() - 1;
			for(size_t i = 0; i < n_page_num_1; ++ i) {
				double *p_data = &m_data_pool[i * n_page_size];

				clEnqueueReadBuffer(h_cmd_queue, t_buffer, b_blocking,
					n_buffer_offset, n_page_size * sizeof(double), p_data, 0, 0, 0);

				n_buffer_offset += n_page_size * sizeof(double);
			}
			// download full pages

			{
				double *p_last_data = &m_data_pool[n_page_num_1 * n_page_size];
				size_t n_last_page_size = m_data_pool.last_page_usage();

				clEnqueueReadBuffer(h_cmd_queue, t_buffer, b_blocking,
					n_buffer_offset, n_last_page_size * sizeof(double),
					p_last_data, 0, 0, p_event);
			}
			// download the last page
		}
		// schedule upload of every page

		return true;
	}

	// debug function for reference CPU implementations
	// n_buffer_offset is in bytes
	bool UploadData_to_Buffer(std::vector<double> &r_t_buffer, size_t n_buffer_offset = 0) const
	{
		_ASSERTE(!(n_buffer_offset % sizeof(double))); // we copy doubles, offset must be in doubles
		_ASSERTE(!m_n_ref_elem_num);
		// must upload data of the original matrix, does not work on the ref matrices (so far)

		size_t n_data_size = m_data_pool.size() * sizeof(double);

		size_t n_buffer_size = r_t_buffer.size() * sizeof(double);
		_ASSERTE(n_buffer_offset <= n_buffer_size);
		if(n_buffer_size - n_buffer_offset < n_data_size)
			return false;

		n_buffer_offset /= sizeof(double);
		// !!

		if(!m_data_pool.empty()) {
			size_t n_page_size = m_data_pool.page_size(),
				n_page_num_1 = m_data_pool.page_num() - 1;
			for(size_t i = 0; i < n_page_num_1; ++ i) {
				const double *p_data = &m_data_pool[i * n_page_size];

				memcpy(&r_t_buffer[n_buffer_offset], p_data, n_page_size * sizeof(double));

				n_buffer_offset += n_page_size;
			}
			// upload full pages

			{
				const double *p_last_data = &m_data_pool[n_page_num_1 * n_page_size];
				size_t n_last_page_size = m_data_pool.last_page_usage();

				memcpy(&r_t_buffer[n_buffer_offset], p_last_data, n_last_page_size * sizeof(double));
			}
			// upload the last page
		}
		// schedule upload of every page

		return true;
	}

	// debug function for reference CPU implementations
	// n_buffer_offset is in bytes
	bool DownloadData_from_Buffer(const std::vector<double> &r_t_buffer, size_t n_buffer_offset = 0)
	{
		_ASSERTE(!(n_buffer_offset % sizeof(double))); // we copy doubles, offset must be in doubles
		_ASSERTE(!m_n_ref_elem_num);
		// must upload data of the original matrix, does not work on the ref matrices (so far)

		size_t n_data_size = m_data_pool.size() * sizeof(double);

		size_t n_buffer_size = r_t_buffer.size() * sizeof(double);
		_ASSERTE(n_buffer_offset <= n_buffer_size);
		if(n_buffer_size - n_buffer_offset < n_data_size)
			return false;

		n_buffer_offset /= sizeof(double);
		// !!

		if(!m_data_pool.empty()) {
			size_t n_page_size = m_data_pool.page_size(),
				n_page_num_1 = m_data_pool.page_num() - 1;
			for(size_t i = 0; i < n_page_num_1; ++ i) {
				double *p_data = &m_data_pool[i * n_page_size];

				memcpy(p_data, &r_t_buffer[n_buffer_offset], n_page_size * sizeof(double));

				n_buffer_offset += n_page_size;
			}
			// download full pages

			{
				double *p_last_data = &m_data_pool[n_page_num_1 * n_page_size];
				size_t n_last_page_size = m_data_pool.last_page_usage();

				memcpy(p_last_data, &r_t_buffer[n_buffer_offset], n_last_page_size * sizeof(double));
			}
			// download the last page
		}
		// schedule upload of every page

		return true;
	}
#endif // 0
};

/*
 *								=== ~block matrix on GPU ===
 */

/*
 *								=== CLinearSolver_Schur_GPUBase ===
 */

bool CLinearSolver_Schur_GPUBase::b_cuda_initialized = false;
size_t CLinearSolver_Schur_GPUBase::n_instance_num = 0;

CLinearSolver_Schur_GPUBase::CLinearSolver_Schur_GPUBase()
{
	__GPU_Function;

	++ n_instance_num;
	if(!b_cuda_initialized) {
		CGPUGuard::Register_SignalHandlers();
		// avoid letting the user quit the program in the middle
		// of some computation, as that sometimes crashes the GPU
		// now have to look at b_quit

		{
			gpu.CUInit();
			// initialize CUDA
		}
	}

	{
		if(cusparseCreate((cusparseHandle_t*)&m_gpu.t_cusparse) != CUSPARSE_STATUS_SUCCESS)
			throw std::runtime_error("cusparseCreate() failed");
		// create a cusparse context

		if(cusparseCreateMatDescr((cusparseMatDescr_t*)&m_gpu.t_matrix_descr) != CUSPARSE_STATUS_SUCCESS ||
		   cusparseSetMatType((cusparseMatDescr_t)m_gpu.t_matrix_descr, CUSPARSE_MATRIX_TYPE_GENERAL) != CUSPARSE_STATUS_SUCCESS ||
		   cusparseSetMatIndexBase((cusparseMatDescr_t)m_gpu.t_matrix_descr, CUSPARSE_INDEX_BASE_ZERO) != CUSPARSE_STATUS_SUCCESS)
			throw std::runtime_error("cusparseCreateMatDescr() failed");
		if(cusparseCreateMatDescr((cusparseMatDescr_t*)&m_gpu.t_sym_matrix_descr) != CUSPARSE_STATUS_SUCCESS ||
		   cusparseSetMatType((cusparseMatDescr_t)m_gpu.t_sym_matrix_descr, CUSPARSE_MATRIX_TYPE_SYMMETRIC) != CUSPARSE_STATUS_SUCCESS ||
		   cusparseSetMatIndexBase((cusparseMatDescr_t)m_gpu.t_sym_matrix_descr, CUSPARSE_INDEX_BASE_ZERO) != CUSPARSE_STATUS_SUCCESS)
			throw std::runtime_error("cusparseCreateMatDescr() failed");
		// create matrix descriptors (will recycle that)

		if(cusparseSetPointerMode((cusparseHandle_t)m_gpu.t_cusparse, CUSPARSE_POINTER_MODE_HOST) != CUSPARSE_STATUS_SUCCESS)
			throw std::runtime_error("cusparseSetPointerMode() failed");
		// cusparse misc settings

		if(cublasCreate((cublasHandle_t*)&m_gpu.t_cublas) != CUBLAS_STATUS_SUCCESS)
			throw std::runtime_error("cublasCreate() failed");
		// initialize cublas

		if(cublasSetPointerMode((cublasHandle_t)m_gpu.t_cublas, CUBLAS_POINTER_MODE_HOST) != CUBLAS_STATUS_SUCCESS)
			throw std::runtime_error("cublasSetPointerMode() failed");
		// cublas misc settings
	}
	// the rest of the state is per instance
}

CLinearSolver_Schur_GPUBase::~CLinearSolver_Schur_GPUBase()
{
	__GPU_Function;

	{
		if(m_gpu.p_A)
			cs_spfree(m_gpu.p_A);
		if(m_gpu.p_B)
			cs_spfree(m_gpu.p_B);
		if(m_gpu.csrRowPtrD)
			cuMemFree((CUdeviceptr)m_gpu.csrRowPtrD);
		if(m_gpu.t_matrix_descr)
			cusparseDestroyMatDescr((cusparseMatDescr_t)m_gpu.t_matrix_descr);
		if(m_gpu.t_sym_matrix_descr)
			cusparseDestroyMatDescr((cusparseMatDescr_t)m_gpu.t_sym_matrix_descr);
		// delete GPU resources

		if(m_gpu.t_cublas)
			cublasDestroy((cublasHandle_t)m_gpu.t_cublas);
		if(m_gpu.t_cusparse)
			cusparseDestroy((cusparseHandle_t)m_gpu.t_cusparse);
		// delete context
	}
	// delete per instance data

	_ASSERTE(n_instance_num > 0); // this exists
	if(!(-- n_instance_num)) {
		// nothing really happens, CUDA remains initialized
	}
	// the last one takes care of the cleanup
}

#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
cs *cs_spalloc32(csi m, csi n, csi nzmax, csi values, csi triplet);
// in BlockMatrix.cpp, needed for x64 builds as cusparse is always using 32-bit indices
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64

bool CLinearSolver_Schur_GPUBase::SpDGEMM(CUberBlockMatrix &r_dest, const CUberBlockMatrix &r_a,
	const CUberBlockMatrix &r_b, bool b_upper_diag_only /*= false*/)
{
	CGPU_BlockMatrix &r_dest_GPU = reinterpret_cast<CGPU_BlockMatrix&>(r_dest);
	const CGPU_BlockMatrix &r_a_GPU = reinterpret_cast<const CGPU_BlockMatrix&>(r_a);
	const CGPU_BlockMatrix &r_b_GPU = reinterpret_cast<const CGPU_BlockMatrix&>(r_b);
	return r_dest_GPU.ProductOf_GPU(r_a_GPU, r_b_GPU, b_upper_diag_only, m_gpu.t_cublas);
}

#if 0

bool CLinearSolver_Schur_GPUBase::SpDGEMM(cs *&r_p_dest,
	const cs *p_A, const cs *p_B, size_t nnzC = size_t(-1), // if nnzC is known, can save computation
	void *h_cusparse, cusparseMatDescr_t t_descr_a,
	cusparseMatDescr_t t_descr_b, cusparseMatDescr_t t_descr_prod,
	const int *&csrRowPtrC, const int *&csrColIndC, const double *&csrValC,
	size_t &r_n_max_nnz_C, size_t &r_n_max_row_C)
{
	int *csrRowPtrA, *csrColIndA;
	double *csrValA;
	{
		int result = cuMemAlloc((CUdeviceptr*)&csrRowPtrA, sizeof(int) * (p_A->n + 1));
		result |= cuMemAlloc((CUdeviceptr*)&csrColIndA, sizeof(int) * nnzA);
		result |= cuMemAlloc((CUdeviceptr*)&csrValA, sizeof(double) * nnzA);
		if(result)
			fprintf(stderr, "error: cuMemAlloc() failed (%d, " PRIsize ")\n", result, nnzA);
		result |= cuMemcpyHtoDAsync((CUdeviceptr)csrRowPtrA, p_A->p, sizeof(int) * (p_A->n + 1), 0);
		result |= cuMemcpyHtoDAsync((CUdeviceptr)csrColIndA, p_A->i, sizeof(int) * nnzA, 0);
		result |= cuMemcpyHtoDAsync((CUdeviceptr)csrValA, p_A->x, sizeof(double) * nnzA, 0);
		if(result != CUDA_SUCCESS)
			throw std::runtime_error("failed to send the A matrix to GPU");
	}
	int *csrRowPtrB, *csrColIndB;
	double *csrValB;
	{
		int result = cuMemAlloc((CUdeviceptr*)&csrRowPtrB, sizeof(int) * (p_B->n + 1));
		result |= cuMemAlloc((CUdeviceptr*)&csrColIndB, sizeof(int) * nnzB);
		result |= cuMemAlloc((CUdeviceptr*)&csrValB, sizeof(double) * nnzB);
		if(result)
			fprintf(stderr, "error: cuMemAlloc() failed (%d, " PRIsize ")\n", result, nnzB);
		result |= cuMemcpyHtoDAsync((CUdeviceptr)csrRowPtrB, p_B->p, sizeof(int) * (p_B->n + 1), 0);
		result |= cuMemcpyHtoDAsync((CUdeviceptr)csrColIndB, p_B->i, sizeof(int) * nnzB, 0);
		result |= cuMemcpyHtoDAsync((CUdeviceptr)csrValB, p_B->x, sizeof(double) * nnzB, 0);
		if(result != CUDA_SUCCESS)
			throw std::runtime_error("failed to send the B matrix to GPU");
	}
	// B is symmetric - the same in CSR and CSC
	// todo - see which combination of transpose flags yields faster results

	if(cuCtxSynchronize() != CUDA_SUCCESS)
		throw std::runtime_error("cuCtxSynchronize() failed " FILE_LINE);

	size_t n_max_nnz_C = 0, n_max_rows_C = 0;
	int *csrRowPtrC = 0, *csrColIndC = 0;
	double *csrValC = 0;
	bool b_result = SpDGEMM(r_p_dest, p_A, p_B, nnzC, h_cusparse, t_descr_a,
		csrRowPtrA, csrColIndA, csrValA, csrRowPtrB, csrColIndB, csrValB,
		csrRowPtrC, csrColIndC, csrValC, n_max_nnz_C, n_max_rows_C);
	// multiply

	cuMemFree((CUdeviceptr)csrRowPtrA);
	cuMemFree((CUdeviceptr)csrColIndA);
	cuMemFree((CUdeviceptr)csrValA);
	cuMemFree((CUdeviceptr)csrRowPtrB);
	cuMemFree((CUdeviceptr)csrColIndB);
	cuMemFree((CUdeviceptr)csrValB);
	cuMemFree((CUdeviceptr)csrRowPtrC);
	cuMemFree((CUdeviceptr)csrColIndC);
	cuMemFree((CUdeviceptr)csrValC);
	// cleanup

	return b_result;
}

bool CLinearSolver_Schur_GPUBase::SpDGEMM(cs *&r_p_dest,
	const cs *p_A, const cs *p_B, size_t nnzC /*= size_t(-1)*/, // if nnzC is known, can save computation
	void *h_cusparse, cusparseMatDescr_t t_descr_a,
	cusparseMatDescr_t t_descr_b, cusparseMatDescr_t t_descr_prod,
	const int *csrRowPtrA, const int *csrColIndA, const double *csrValA,
	const int *csrRowPtrB, const int *csrColIndB, const double *csrValB,
	int *&csrRowPtrC, int *&csrColIndC, double *&csrValC,
	size_t &r_n_max_nnz_C, size_t &r_n_max_row_C)
{
	__GPU_Function;

	const size_t nnzA = ((uint32_t*)p_A->p)[p_A->n], nnzB = ((uint32_t*)p_B->p)[p_B->n];
	const size_t m = p_A->n; // A is CUSPARSE_OPERATION_NON_TRANSPOSE
	const size_t n = p_B->m; // B is CUSPARSE_OPERATION_NON_TRANSPOSE
	const size_t k = p_A->m; // A is CUSPARSE_OPERATION_NON_TRANSPOSE
	// m, n are also dimensions of C
	
	int *csrRowPtrC;
	/*{
		int result = cuMemAlloc((CUdeviceptr*)&csrRowPtrC, sizeof(int) * (m + 1));
		if(result != CUDA_SUCCESS)
			throw std::runtime_error("failed to allocate product matrix rowptr on GPU");
	}*/
	// the same as csrRowPtrB

	if(nnzC != size_t(-1)) {
		{
			_ASSERTE(m <= INT_MAX && n <= INT_MAX && k <= INT_MAX);
			_ASSERTE(nnzA <= INT_MAX && nnzB <= INT_MAX);
			int nnzCi;
			int status = cusparseXcsrgemmNnz((cusparseHandle_t)h_cusparse, CUSPARSE_OPERATION_NON_TRANSPOSE,
				CUSPARSE_OPERATION_NON_TRANSPOSE, int(m), int(n), int(k), t_descr_a,
				int(nnzA), csrRowPtrA, csrColIndA, t_descr_b, int(nnzB), csrRowPtrB,
				csrColIndB, t_descr_prod, csrRowPtrC, &nnzCi);
			nnzC = nnzCi; // !!
			// calculate the number of nnz in the product

			if(status != CUSPARSE_STATUS_SUCCESS)
				throw std::runtime_error("cusparseXcsrgemmNnz() failed");
			//nnzCstat = status; // debug
		}
		// t_odo - recaculate this only if !b_keep_ordering, otherwise reuse
		// t_odo - here we are multiplying a block matrix by a block diagonal matrix
		//		  the sparsity is therefore always the same, as is the nnz

		if(cuCtxSynchronize() != CUDA_SUCCESS)
			throw std::runtime_error("cuCtxSynchronize() failed " FILE_LINE);
		// about to read the NNZs back for cuMemAlloc()
	}
	// calculate the amount of NNZ in C, unless known

	//printf("debug: the product has %d nnz (%d)\n", nnzC, nnzCstat); // debug

	int *csrColIndC;
	double *csrValC;
	{
		int result = CUDA_SUCCESS;
#ifdef _DEBUG
		int baseC, _nnzC;
		result |= cuMemcpyDtoH(&_nnzC, (CUdeviceptr)(csrRowPtrC + m), sizeof(int)); // read the nnz
		_ASSERTE(_nnzC == nnzC);
		result |= cuMemcpyDtoH(&baseC, (CUdeviceptr)csrRowPtrC, sizeof(int)); // read the base
		_ASSERTE(!baseC); // debug
#endif // _DEBUG
		result |= cuMemAlloc((CUdeviceptr*)&csrColIndC, sizeof(int) * nnzC);
		result |= cuMemAlloc((CUdeviceptr*)&csrValC, sizeof(double) * nnzC);
		if(result != CUDA_SUCCESS)
			throw std::runtime_error("failed to allocate product matrix on GPU");
	}
	// alloc C

	//printf("debug: allocated product storage\n"); // debug

	{
		_ASSERTE(m <= INT_MAX && n <= INT_MAX && k <= INT_MAX);
		_ASSERTE(nnzA <= INT_MAX && nnzB <= INT_MAX);
		int status = cusparseDcsrgemm((cusparseHandle_t)m_gpu.t_cusparse, CUSPARSE_OPERATION_NON_TRANSPOSE,
			CUSPARSE_OPERATION_NON_TRANSPOSE, int(m), int(n), int(k), (cusparseMatDescr_t)m_gpu.t_matrix_descr,
			int(nnzA), csrValA, csrRowPtrA, csrColIndA, (cusparseMatDescr_t)m_gpu.t_matrix_descr, int(nnzB), csrValB,
			csrRowPtrB, csrColIndB, (cusparseMatDescr_t)m_gpu.t_matrix_descr, csrValC, csrRowPtrC, csrColIndC);
		// A is symmetric (A is C^-1)

		if(status != CUSPARSE_STATUS_SUCCESS)
			throw std::runtime_error("cusparseDcsrgemm() failed");
	}
	// gemm

	//printf("debug: GEMM finished\n"); // debug

	if(cuCtxSynchronize() != CUDA_SUCCESS)
		throw std::runtime_error("cuCtxSynchronize() failed " FILE_LINE);

	return true;
}

#endif // 0

bool CLinearSolver_Schur_GPUBase::GPUSolve(const CUberBlockMatrix &r_lambda,
	Eigen::VectorXd &r_v_eta, bool b_keep_ordering, size_t n_landmark_dimension,
	std::vector<double> &m_double_workspace, const std::vector<size_t> &m_order,
	const size_t m_n_matrix_cut, _TyBaseSolver &m_linear_solver)
{
	__GPU_Function;

	_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
	_ASSERTE(r_v_eta.rows() == r_lambda.n_Row_Num()); // make sure the vector has correct dimension

	_ASSERTE(r_lambda.b_SymmetricLayout());
	_ASSERTE(r_lambda.n_BlockColumn_Num() == m_order.size());
	_ASSERTE((m_order.empty() && !m_n_matrix_cut) || (!m_order.empty() &&
		m_n_matrix_cut > 0 && m_n_matrix_cut < SIZE_MAX && m_n_matrix_cut + 1 < m_order.size()));
	_ASSERTE(r_v_eta.rows() == r_lambda.n_Column_Num());

#ifdef __SCHUR_PROFILING
	CDeltaTimer dt;
#endif // __SCHUR_PROFILING

	CUberBlockMatrix lambda_perm;
	r_lambda.Permute_UpperTriangular_To(lambda_perm, &m_order[0], m_order.size(), true);
	// reorder the matrix

#ifdef __SCHUR_PROFILING
	double f_reperm_time = dt.f_Time();
#endif // __SCHUR_PROFILING

	const size_t n = lambda_perm.n_BlockColumn_Num();

	CUberBlockMatrix A, U, C;
	const size_t n_matrix_cut = m_n_matrix_cut; // antialiass
	lambda_perm.SliceTo(A, 0, n_matrix_cut, 0, n_matrix_cut, true);
	lambda_perm.SliceTo(U, 0, n_matrix_cut, n_matrix_cut, n, true);
	lambda_perm.SliceTo(C, n_matrix_cut, n, n_matrix_cut, n, true);
	// cut Lambda matrix into pieces
	// \lambda = | A U |
	//           | V C |

	const size_t n_rhs_vector_size = r_lambda.n_Column_Num();
	const size_t n_pose_vector_size = A.n_Column_Num(); // 6 * m_n_matrix_cut;
	const size_t n_landmark_vector_size = U.n_Column_Num(); // 3 * (n - m_n_matrix_cut);
	// not block columns! element ones

#ifdef __SCHUR_PROFILING
	double f_slice_time = dt.f_Time();
	double f_transpose_time = 0;//dt.f_Time();
#endif // __SCHUR_PROFILING

#if 0
	CUberBlockMatrix C_inv;
	if(n_landmark_dimension == 3) // this can be anticipated
		C_inv.InverseOf_Symmteric_FBS<MakeTypelist_Safe((fbs_ut::CCTSize2D<3, 3>))>(C, true); // C is block diagonal (should also be symmetric)
	else
		C_inv.InverseOf_Symmteric(C, true);
#else // 0
	_ASSERTE(C.b_BlockDiagonal()); // it is, unless the ordering is bad
	CUberBlockMatrix /*&*/C_inv/* = C*/; // can do it inplace // do *not* reuse storage, will modify lambda!
	if(n_landmark_dimension == 3) // this can be anticipated in BA
		C_inv.InverseOf_BlockDiag_FBS_Parallel<MakeTypelist_Safe((fbs_ut::CCTSize2D<3, 3>))>(C); // faster version, slightly less general
	else
		C_inv.InverseOf_Symmteric(C, true); // there is no non-FBS InverseOf_BlockDiag(), just use this one instead 
#endif // 0
	// inverse of C

#ifdef __SCHUR_PROFILING
	double f_inverse_time = dt.f_Time();
#endif // __SCHUR_PROFILING

	/*CUberBlockMatrix unity, u_ref;
	unity.ProductOf(C, C_inv);
	unity.CopyLayoutTo(u_ref);
	u_ref.SetIdentity();
	unity.AddTo(u_ref, -1);
	double f_error = u_ref.f_Norm();
	fprintf(stderr, "error of matrix inverse is: %g\n", f_error);*/
	// check inverse

#ifdef __GPU_SCHUR_VERIFY_RESULT
	CUberBlockMatrix U_Cinv_ref, minus_schur_ref;
	{
		U.MultiplyToWith(U_Cinv_ref, C_inv);
#ifndef __GPU_SCHUR_EASY_PROD_ONLY
		CUberBlockMatrix V;
		U.TransposeTo(V); // because lower-triangular of lambda is not calculated
		U_Cinv_ref.MultiplyToWith(minus_schur_ref, V);
#endif // !__GPU_SCHUR_EASY_PROD_ONLY
	}
	// debug - calculate product on the CPU
#endif // __GPU_SCHUR_VERIFY_RESULT

#ifdef __SCHUR_PROFILING
	double f_mul0_time; // inside
#endif // __SCHUR_PROFILING
	CUberBlockMatrix minus_U_Cinv, schur_compl;

#if 0 // all prods on CPU (overrides __GPU_SCHUR_NO_PRODS in LinearSolver_Schur.h without having to rebuild all; note that it is non-FBS and therefore slower)
	C_inv.Scale(-1);
	//U.MultiplyToWith(minus_U_Cinv, C_inv);
	U.MultiplyToWith_FBS<MakeTypelist_Safe((Eigen::Matrix<double, 6, 3>)),
		MakeTypelist_Safe((Eigen::Matrix<double, 3, 3>))>(minus_U_Cinv, C_inv); // use FBS here (guess the block sizes)
#ifdef __SCHUR_PROFILING
	f_mul0_time = dt.f_Time();
#endif // __SCHUR_PROFILING
	printf("UBlock GEMM1 time: %f\n", f_mul0_time);
	{
		CUberBlockMatrix V;
		U.TransposeTo(V); // because lower-triangular of lambda is not calculated
		//minus_U_Cinv.MultiplyToWith_FBS<>(schur_compl, V);
		minus_U_Cinv.MultiplyToWith_FBS<MakeTypelist_Safe((Eigen::Matrix<double, 6, 3>)),
			MakeTypelist_Safe((Eigen::Matrix<double, 3, 6>))>(schur_compl, V);
#ifdef __SCHUR_PROFILING
		printf("UBlock GEMM2 time: %f (full matrix, not only upper half, " PRIsize " NNZ blocks)\n", dt.f_Time(),
			schur_compl.n_Block_Num());
#endif // __SCHUR_PROFILING
		// time with perfect ordering

		/*{
			cs *p_blayout = V.p_BlockStructure_to_Sparse(cs_spalloc(V.n_BlockRow_Num(),
				V.n_BlockColumn_Num(), V.n_Block_Num(), 1, 0));
			if(!p_blayout)
				throw std::runtime_error("V.p_BlockStructure_to_Sparse() failed");
			std::vector<size_t> row_cumsums(p_blayout->m), col_cumsums(p_blayout->n), workspace;
			for(size_t i = 0, n = row_cumsums.size(); i < n; ++ i)
				row_cumsums[i] = i + 1;
			for(size_t i = 0, n = col_cumsums.size(); i < n; ++ i)
				col_cumsums[i] = i + 1;
			CUberBlockMatrix V_bs(row_cumsums.begin(), row_cumsums.end(),
				col_cumsums.begin(), col_cumsums.end());
			if(!V_bs.From_Sparse(0, 0, p_blayout, false, workspace))
				throw std::runtime_error("V_bs.From_Sparse() failed");
			cs_spfree(p_blayout);
			V_bs.Save_MatrixMarket("G:\\uflsmc\\sparse\\mat\\Venice\\V_struct.mtx");
		}
		// save V's structure as .mtx

		V.Save_MatrixMarket("G:\\uflsmc\\sparse\\mat\\Venice\\V_full.mtx");
		// save full V as .mtx

		{
			cs *p_blayout = minus_U_Cinv.p_BlockStructure_to_Sparse(cs_spalloc(minus_U_Cinv.n_BlockRow_Num(),
				minus_U_Cinv.n_BlockColumn_Num(), minus_U_Cinv.n_Block_Num(), 1, 0));
			if(!p_blayout)
				throw std::runtime_error("UCinv.p_BlockStructure_to_Sparse() failed");
			std::vector<size_t> row_cumsums(p_blayout->m), col_cumsums(p_blayout->n), workspace;
			for(size_t i = 0, n = row_cumsums.size(); i < n; ++ i)
				row_cumsums[i] = i + 1;
			for(size_t i = 0, n = col_cumsums.size(); i < n; ++ i)
				col_cumsums[i] = i + 1;
			CUberBlockMatrix V_bs(row_cumsums.begin(), row_cumsums.end(),
				col_cumsums.begin(), col_cumsums.end());
			if(!V_bs.From_Sparse(0, 0, p_blayout, false, workspace))
				throw std::runtime_error("UCinv_bs.From_Sparse() failed");
			cs_spfree(p_blayout);
			V_bs.Save_MatrixMarket("G:\\uflsmc\\sparse\\mat\\Venice\\UCinv_struct.mtx");
		}
		// save minus_U_Cinv's structure as .mtx

		minus_U_Cinv.Save_MatrixMarket("G:\\uflsmc\\sparse\\mat\\Venice\\UCinv_full.mtx");
		// save full minus_U_Cinv as .mtx

		for(size_t n_slice_num = 4; n_slice_num < 64; n_slice_num *= 2) {
			for(int i = 0; i < n_slice_num; ++ i) {
				const size_t n = minus_U_Cinv.n_BlockColumn_Num();
				const size_t b = (n * i) / n_slice_num;
				const size_t e = (i + 1 < n_slice_num)? (n * (i + 1)) / n_slice_num : n;

				CUberBlockMatrix slice;
				minus_U_Cinv.SliceTo(slice, 0, minus_U_Cinv.n_BlockRow_Num(), b, e, true);
				char p_s_name[1024];
				sprintf(p_s_name, "G:\\uflsmc\\sparse\\mat\\Venice\\UCinv_slice_%dof%d.mtx",
					int(i + 1), int(n_slice_num));
				slice.Save_MatrixMarket(p_s_name); 
			}
		}*/
		// save different granularity slices of minus_U_Cinv as .mtx

		/*
#ifdef __SCHUR_PROFILING
		dt.f_Time(); // don't count the stuff before
#endif // __SCHUR_PROFILING

		cs *p_U = U.p_Convert_to_Sparse();
		cs *p_C_inv = C_inv.p_Convert_to_Sparse();
		cs *p_U_Cinv = cs_multiply(p_U, p_C_inv);

#ifdef __SCHUR_PROFILING
		printf("CSparse GEMM1 takes %f sec\n", dt.f_Time());
#endif // __SCHUR_PROFILING

		cs_spfree(p_U);
		cs_spfree(p_C_inv);
		cs_spfree(p_U_Cinv);

#ifdef __SCHUR_PROFILING
		dt.f_Time(); // don't count spfree()
#endif // __SCHUR_PROFILING

		cs *p_minus_U_Cinv = minus_U_Cinv.p_Convert_to_Sparse();
		cs *p_V = V.p_Convert_to_Sparse();
		cs *p_SC = cs_multiply(p_minus_U_Cinv, p_V);

#ifdef __SCHUR_PROFILING
		printf("CSparse GEMM2 takes %f sec\n", dt.f_Time());
#endif // __SCHUR_PROFILING

		cs_spfree(p_minus_U_Cinv);
		cs_spfree(p_V);
		cs_spfree(p_SC);

#ifdef __SCHUR_PROFILING
		dt.f_Time(); // don't count spfree()
#endif // __SCHUR_PROFILING
		*/
		// time CSparse
	}
	C_inv.Scale(-1);
#else // 0
	// GPU gemm
	{
		cs *&p_A = m_gpu.p_A, *&p_B = m_gpu.p_B;
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
		p_A = C_inv.p_Convert_to_Sparse32(p_A);
		p_B = U.p_Convert_to_Sparse32(p_B);
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
		p_A = C_inv.p_Convert_to_Sparse(p_A);
		p_B = U.p_Convert_to_Sparse(p_B);
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
		// swap order! it does it in transpose and transposes the result

		const size_t nnzA = ((uint32_t*)p_A->p)[p_A->n], nnzB = ((uint32_t*)p_B->p)[p_B->n];
		const size_t m = p_A->n; // A is CUSPARSE_OPERATION_NON_TRANSPOSE
		const size_t n = p_B->m; // B is CUSPARSE_OPERATION_NON_TRANSPOSE
		const size_t k = p_A->m; // A is CUSPARSE_OPERATION_NON_TRANSPOSE
		// m, n are also dimensions of C

		/*printf("A is " PRIsize " x " PRIsize "\n", p_A->m, p_A->n);
		printf("B is " PRIsize " x " PRIsize "\n", p_B->m, p_B->n);
		printf("entered m, n, k: " PRIsize ", " PRIsize ", " PRIsize "\n", m, n, k);
		printf("A has " PRIsize " nnz, B has " PRIsize " nnz\n", U.n_NonZero_Num(), C_inv.n_NonZero_Num());
		U.MultiplyToWith(minus_U_Cinv, C_inv);
		printf("A * B has " PRIsize " nnz (int max is %d; %d)\n", minus_U_Cinv.n_NonZero_Num(),
			INT_MAX, minus_U_Cinv.n_NonZero_Num() < INT_MAX);
		printf("A * B is " PRIsize " x " PRIsize "\n", minus_U_Cinv.n_Row_Num(), minus_U_Cinv.n_Column_Num());
		printf("A has " PRIsize " nnz < INT_MAX, B has " PRIsize " nnz < INT_MAX\n",
			U.n_NonZero_Num() < INT_MAX, C_inv.n_NonZero_Num() < INT_MAX);
		CUberBlockMatrix minus_U_Cinv_ref;
		minus_U_Cinv_ref.Swap(minus_U_Cinv);
		minus_U_Cinv.Clear();
		// debug - matrix sizes
		printf("A has " PRIsize " nnz, B has " PRIsize " nnz\n", nnzA, nnzB);*/
		// debug

		int *csrRowPtrC;
		/*{
			int result = cuMemAlloc((CUdeviceptr*)&csrRowPtrC, sizeof(int) * (m + 1));
			if(result != CUDA_SUCCESS)
				throw std::runtime_error("failed to allocate product matrix rowptr on GPU");
		}*/
		// the same as csrRowPtrB

		int *csrRowPtrA, *csrColIndA;
		double *csrValA;
		{
			int result = cuMemAlloc((CUdeviceptr*)&csrRowPtrA, sizeof(int) * (p_A->n + 1));
			result |= cuMemAlloc((CUdeviceptr*)&csrColIndA, sizeof(int) * nnzA);
			result |= cuMemAlloc((CUdeviceptr*)&csrValA, sizeof(double) * nnzA);
			if(result)
				fprintf(stderr, "error: cuMemAlloc() failed (%d, " PRIsize ")\n", result, nnzA);
			result |= cuMemcpyHtoDAsync((CUdeviceptr)csrRowPtrA, p_A->p, sizeof(int) * (p_A->n + 1), 0);
			result |= cuMemcpyHtoDAsync((CUdeviceptr)csrColIndA, p_A->i, sizeof(int) * nnzA, 0);
			result |= cuMemcpyHtoDAsync((CUdeviceptr)csrValA, p_A->x, sizeof(double) * nnzA, 0);
			if(result != CUDA_SUCCESS)
				throw std::runtime_error("failed to send the A matrix to GPU");
		}
		int *csrRowPtrB, *csrColIndB;
		double *csrValB;
		{
			int result = cuMemAlloc((CUdeviceptr*)&csrRowPtrB, sizeof(int) * (p_B->n + 1));
			result |= cuMemAlloc((CUdeviceptr*)&csrColIndB, sizeof(int) * nnzB);
			result |= cuMemAlloc((CUdeviceptr*)&csrValB, sizeof(double) * nnzB);
			if(result)
				fprintf(stderr, "error: cuMemAlloc() failed (%d, " PRIsize ")\n", result, nnzB);
			result |= cuMemcpyHtoDAsync((CUdeviceptr)csrRowPtrB, p_B->p, sizeof(int) * (p_B->n + 1), 0);
			result |= cuMemcpyHtoDAsync((CUdeviceptr)csrColIndB, p_B->i, sizeof(int) * nnzB, 0);
			result |= cuMemcpyHtoDAsync((CUdeviceptr)csrValB, p_B->x, sizeof(double) * nnzB, 0);
			if(result != CUDA_SUCCESS)
				throw std::runtime_error("failed to send the B matrix to GPU");
		}
		// B is symmetric - the same in SCR and CSC
		// todo - see which combination of transpose flags yields faster results

		if(cuCtxSynchronize() != CUDA_SUCCESS)
			throw std::runtime_error("cuCtxSynchronize() failed " FILE_LINE);

		//printf("debug: have matrices in GPU\n"); // debug

		// p_A, p_B are not needed anymore // todo - reuse for the second product

		{
			const double f_x = -1;
			_ASSERTE(nnzA < INT_MAX);
			cublasDscal((cublasHandle_t)m_gpu.t_cublas, int(nnzA), &f_x, csrValA, 1);
		}
		// use cublasDscal() to flip signs on A (A = C^-1)

		size_t nnzC/*, nnzCstat*/;
#if 1
		nnzC = nnzB;
		/*{
			int result = cuMemcpyDtoD((CUdeviceptr)csrRowPtrC, (CUdeviceptr)csrRowPtrB, (p_B->n + 1) * sizeof(int));
			if(result != CUDA_SUCCESS)
				throw std::runtime_error("cuMemcpyDtoD() failed");
			// this is the same thing, the result has the same structure as B
		}*/
		csrRowPtrC = csrRowPtrB; // in fact, just use the same array, it will not get overwritten
#else // 1
		{
			_ASSERTE(m <= INT_MAX && n <= INT_MAX && k <= INT_MAX);
			_ASSERTE(nnzA <= INT_MAX && nnzB <= INT_MAX);
			int nnzCi;
			int status = cusparseXcsrgemmNnz((cusparseHandle_t)m_gpu.t_cusparse, CUSPARSE_OPERATION_NON_TRANSPOSE,
				CUSPARSE_OPERATION_NON_TRANSPOSE, int(m), int(n), int(k), (cusparseMatDescr_t)m_gpu.t_matrix_descr,
				int(nnzA), csrRowPtrA, csrColIndA, (cusparseMatDescr_t)m_gpu.t_matrix_descr, int(nnzB), csrRowPtrB,
				csrColIndB, (cusparseMatDescr_t)m_gpu.t_matrix_descr, csrRowPtrC, &nnzCi);
			nnzC = nnzCi; // !!
			// A is symmetric (A is C^-1)

			if(status != CUSPARSE_STATUS_SUCCESS)
				throw std::runtime_error("cusparseXcsrgemmNnz() failed");
			//nnzCstat = status; // debug
		}
		// calculate the amount of NNZ in C
		// t_odo - recaculate this only if !b_keep_ordering, otherwise reuse
		// t_odo - here we are multiplying a block matrix by a block diagonal matrix
		//		  the sparsity is therefore always the same, as is the nnz

		if(cuCtxSynchronize() != CUDA_SUCCESS)
			throw std::runtime_error("cuCtxSynchronize() failed " FILE_LINE);
		// about to read the NNZs back for cuMemAlloc()
#endif // 1

		//printf("debug: the product has %d nnz (%d)\n", nnzC, nnzCstat); // debug

		int *csrColIndC;
		double *csrValC;
		{
			int result = CUDA_SUCCESS;
#ifdef _DEBUG
			int baseC, _nnzC;
			result |= cuMemcpyDtoH(&_nnzC, (CUdeviceptr)(csrRowPtrC + m), sizeof(int)); // read the nnz
			_ASSERTE(_nnzC == nnzC);
			result |= cuMemcpyDtoH(&baseC, (CUdeviceptr)csrRowPtrC, sizeof(int)); // read the base
			_ASSERTE(!baseC); // debug
#endif // _DEBUG
			result |= cuMemAlloc((CUdeviceptr*)&csrColIndC, sizeof(int) * nnzC);
			result |= cuMemAlloc((CUdeviceptr*)&csrValC, sizeof(double) * nnzC);
			if(result != CUDA_SUCCESS)
				throw std::runtime_error("failed to allocate product matrix on GPU");
		}
		// alloc C

		//printf("debug: allocated product storage\n"); // debug

		{
			_ASSERTE(m <= INT_MAX && n <= INT_MAX && k <= INT_MAX);
			_ASSERTE(nnzA <= INT_MAX && nnzB <= INT_MAX);
			int status = cusparseDcsrgemm((cusparseHandle_t)m_gpu.t_cusparse, CUSPARSE_OPERATION_NON_TRANSPOSE,
				CUSPARSE_OPERATION_NON_TRANSPOSE, int(m), int(n), int(k), (cusparseMatDescr_t)m_gpu.t_matrix_descr,
				int(nnzA), csrValA, csrRowPtrA, csrColIndA, (cusparseMatDescr_t)m_gpu.t_matrix_descr, int(nnzB), csrValB,
				csrRowPtrB, csrColIndB, (cusparseMatDescr_t)m_gpu.t_matrix_descr, csrValC, csrRowPtrC, csrColIndC);
			// A is symmetric (A is C^-1)

			if(status != CUSPARSE_STATUS_SUCCESS)
				throw std::runtime_error("cusparseDcsrgemm() failed");
		}
		// gemm

		//printf("debug: GEMM finished\n"); // debug

		if(cuCtxSynchronize() != CUDA_SUCCESS)
			throw std::runtime_error("cuCtxSynchronize() failed " FILE_LINE);

		{
			int result = cuMemFree((CUdeviceptr)csrRowPtrA);
			result |= cuMemFree((CUdeviceptr)csrColIndA);
			result |= cuMemFree((CUdeviceptr)csrValA);
			if(result != CUDA_SUCCESS)
				throw std::runtime_error("failed to free matrix storage on GPU");
		}
		// can delete A at this point

#ifndef __GPU_SCHUR_EASY_PROD_ONLY

		const size_t m1 = /*p_B->m*/U.n_Row_Num(); // B is CUSPARSE_OPERATION_TRANSPOSE
		const size_t n1 = /*p_C->*/n; // C is CUSPARSE_OPERATION_NON_TRANSPOSE
		const size_t k1 = /*p_B->n*/U.n_Column_Num(); // B is CUSPARSE_OPERATION_TRANSPOSE

		int *&csrRowPtrD = m_gpu.csrRowPtrD, &nnzD = m_gpu.nnzD;
		if(!csrRowPtrD || !b_keep_ordering) {
			if(!csrRowPtrD || m_gpu.csrRowPtrD_size != (n + 1)) {
				if(csrRowPtrD)
					cuMemFree((CUdeviceptr)csrRowPtrD);
				m_gpu.csrRowPtrD_size = 0;
				int result = cuMemAlloc((CUdeviceptr*)&csrRowPtrD, sizeof(int) * (n + 1));
				if(result != CUDA_SUCCESS)
					throw std::runtime_error("failed to allocate product matrix rowptr on GPU");
				m_gpu.csrRowPtrD_size = n + 1;
			}
			// alloc

			_ASSERTE(m1 <= INT_MAX && n1 <= INT_MAX && k1 <= INT_MAX);
			_ASSERTE(nnzB <= INT_MAX && nnzC <= INT_MAX);
			int status = cusparseXcsrgemmNnz((cusparseHandle_t)m_gpu.t_cusparse, CUSPARSE_OPERATION_TRANSPOSE,
				CUSPARSE_OPERATION_NON_TRANSPOSE, int(m1), int(n1), int(k1), (cusparseMatDescr_t)m_gpu.t_matrix_descr,
				int(nnzB), csrRowPtrB, csrColIndB, (cusparseMatDescr_t)m_gpu.t_matrix_descr, int(nnzC), csrRowPtrC, csrColIndC,
				(cusparseMatDescr_t)m_gpu.t_matrix_descr, csrRowPtrD, &nnzD); // t_odo - D = U(C^-1)V is symmetric, try what it does // fails
			if(status != CUSPARSE_STATUS_SUCCESS)
				throw std::runtime_error("cusparseXcsrgemmNnz() failed");
			// fill
		}
		// recalculate this only if ordering changes

		if(cuCtxSynchronize() != CUDA_SUCCESS)
			throw std::runtime_error("cuCtxSynchronize() failed " FILE_LINE);
		// about to read the NNZs back for cuMemAlloc()

		/*printf("the product of all 3 matrices will be " PRIsize " x " PRIsize " and will have " PRIsize " nnz\n",
			m1, n1, nnzD);*/

		int *csrColIndD; // todo - could cache this nut it probably chanes often
		double *csrValD;
		{
			int result = CUDA_SUCCESS;
#ifdef _DEBUG
			int baseD, _nnzD;
			result |= cuMemcpyDtoH(&_nnzD, (CUdeviceptr)(csrRowPtrD + m1), sizeof(int)); // read the nnz
			_ASSERTE(_nnzD == nnzD);
			result |= cuMemcpyDtoH(&baseD, (CUdeviceptr)csrRowPtrD, sizeof(int)); // read the base
			_ASSERTE(!baseD); // debug
#endif // _DEBUG
			result |= cuMemAlloc((CUdeviceptr*)&csrColIndD, sizeof(int) * nnzD);
			result |= cuMemAlloc((CUdeviceptr*)&csrValD, sizeof(double) * nnzD);
			if(result != CUDA_SUCCESS)
				throw std::runtime_error("failed to allocate product matrix on GPU");
		}
		// alloc D

		//printf("debug: allocated product storage\n"); // debug

		{
			int status = cusparseDcsrgemm((cusparseHandle_t)m_gpu.t_cusparse, CUSPARSE_OPERATION_TRANSPOSE,
				CUSPARSE_OPERATION_NON_TRANSPOSE, int(m1), int(n1), int(k1), (cusparseMatDescr_t)m_gpu.t_matrix_descr,
				int(nnzB), csrValB, csrRowPtrB, csrColIndB, (cusparseMatDescr_t)m_gpu.t_matrix_descr, int(nnzC), csrValC,
				csrRowPtrC, csrColIndC, (cusparseMatDescr_t)m_gpu.t_matrix_descr, csrValD, csrRowPtrD, csrColIndD);
			// todo - D = U(C^-1)V is symmetric, try what it does

			if(status != CUSPARSE_STATUS_SUCCESS)
				throw std::runtime_error("cusparseDcsrgemm() failed");
		}
		// gemm

#endif // !__GPU_SCHUR_EASY_PROD_ONLY

		//printf("debug: GEMM finished\n"); // debug

		{
			//U.CopyLayoutTo(minus_U_Cinv);
			minus_U_Cinv = U; // also preallocates the required blocks
			//U.Swap(minus_U_Cinv); // U is needed below
			// prepare block layout

			if(p_B->n != m || p_B->m != n || p_B->nzmax < 0 || size_t(p_B->nzmax) < nnzC)
				throw std::runtime_error("can't reuse p_B");
			cs *p_C = p_B;//cs_spalloc32(n, m, nnzC, 1, 0); // can reuse the storage
			{
				int result = cuMemcpyDtoHAsync(p_C->p, (CUdeviceptr)csrRowPtrC, sizeof(int) * (m + 1), 0);
				result |= cuMemcpyDtoHAsync(p_C->i, (CUdeviceptr)csrColIndC, sizeof(int) * nnzC, 0);
				result |= cuMemcpyDtoHAsync(p_C->x, (CUdeviceptr)csrValC, sizeof(double) * nnzC, 0);
				if(result != CUDA_SUCCESS)
					throw std::runtime_error("failed to download GEMM result from the GPU");
			}
			// copy data but don't free the product yet, the last GEMM is still referencing it

			if(cuCtxSynchronize() != CUDA_SUCCESS)
				throw std::runtime_error("cuCtxSynchronize() failed " FILE_LINE);

			std::vector<size_t> workspace0;
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
			if(!minus_U_Cinv.From_Sparse32_Parallel(0, 0, p_C, false, workspace0)) {
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
			if(!minus_U_Cinv.From_Sparse_Parallel(0, 0, p_C, false, workspace0)) {
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
				fprintf(stderr, "error: C is " PRIsize " x " PRIsize ", UC--1 expects " PRIsize " x " PRIsize "\n",
					p_C->m, p_C->n, minus_U_Cinv.n_Row_Num(), minus_U_Cinv.n_Column_Num());
				throw std::runtime_error("From_Sparse_Parallel() failed");
			}
			// make it back into a block matrix

			//cs_spfree(p_C);
			// not needed anymore
		}
		// overlap this with the GPU computation
		// contains synchronization

		{
			int result = cuMemFree((CUdeviceptr)csrRowPtrB);
			result |= cuMemFree((CUdeviceptr)csrColIndB);
			result |= cuMemFree((CUdeviceptr)csrValB);
			//result |= cuMemFree((CUdeviceptr)csrRowPtrC); // reused as csrRowPtrC
			result |= cuMemFree((CUdeviceptr)csrColIndC);
			result |= cuMemFree((CUdeviceptr)csrValC);
			if(result != CUDA_SUCCESS)
				throw std::runtime_error("failed to free matrix storage on GPU");
		}
		// free the operands

		//printf("debug: operands deleted\n"); // debug

#ifdef __SCHUR_PROFILING
		f_mul0_time = dt.f_Time();
#endif // __SCHUR_PROFILING

#ifndef __GPU_SCHUR_EASY_PROD_ONLY

#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
		cs *p_D = cs_spalloc32(n1, m1, nnzD, 1, 0);
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
		cs *p_D = cs_spalloc(n1, m1, nnzD, 1, 0);
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
		{
			int result = cuMemcpyDtoHAsync(p_D->p, (CUdeviceptr)csrRowPtrD, sizeof(int) * (m1 + 1), 0);
			result |= cuMemcpyDtoHAsync(p_D->i, (CUdeviceptr)csrColIndD, sizeof(int) * nnzD, 0);
			result |= cuMemcpyDtoHAsync(p_D->x, (CUdeviceptr)csrValD, sizeof(double) * nnzD, 0);
			//result |= cuMemFree((CUdeviceptr)csrRowPtrD); // don't, it is cached
			result |= cuMemFree((CUdeviceptr)csrColIndD);
			result |= cuMemFree((CUdeviceptr)csrValD);
			if(result != CUDA_SUCCESS)
				throw std::runtime_error("failed to download GEMM result from the GPU");
		}
		// copy, free the product

		//printf("debug: product deleted\n"); // debug

		A.CopyLayoutTo(schur_compl);
		// prepare block layout

		if(cuCtxSynchronize() != CUDA_SUCCESS)
			throw std::runtime_error("cuCtxSynchronize() failed " FILE_LINE);
		// wait for the copy to finish

		//printf("debug: calling From_Sparse32_Parallel()\n"); // debug

		std::vector<size_t> workspace;
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
		if(!schur_compl.From_Sparse32_Parallel(0, 0, p_D, false, workspace)) {
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
		if(!schur_compl.From_Sparse_Parallel(0, 0, p_D, false, workspace)) {
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
			fprintf(stderr, "error: D is " PRIsize " x " PRIsize ", UC--1 expects " PRIsize " x " PRIsize "\n",
				p_D->m, p_D->n, schur_compl.n_Row_Num(), schur_compl.n_Column_Num());
			throw std::runtime_error("From_Sparse_Parallel() failed");
		}
		// make it back into a block matrix

		cs_spfree(p_D);
		// not needed anymore

#endif // !__GPU_SCHUR_EASY_PROD_ONLY

		/*printf("debug: done\n");*/

#ifdef __GPU_SCHUR_VERIFY_RESULT
		{
			double f_correct_norm = U_Cinv_ref.f_Norm();
			minus_U_Cinv.AddTo(U_Cinv_ref);
			printf("GPU UC^-1 precise to: %g (%g rel)\n", U_Cinv_ref.f_Norm(),
				U_Cinv_ref.f_Norm() / f_correct_norm);
		}
#ifndef __GPU_SCHUR_EASY_PROD_ONLY
		{
			double f_correct_norm = minus_schur_ref.f_Norm();
			schur_compl.AddTo(minus_schur_ref);
			printf("GPU UC^-1V precise to: %g (%g rel)\n", minus_schur_ref.f_Norm(),
				minus_schur_ref.f_Norm() / f_correct_norm);
		}
#endif // !__GPU_SCHUR_EASY_PROD_ONLY
		// debug - verify the GPU results

		/*U_Cinv_ref.Scale(-1);
		minus_U_Cinv.Swap(U_Cinv_ref);
#ifndef __GPU_SCHUR_EASY_PROD_ONLY
		minus_schur_ref.Scale(-1);
		schur_compl.Swap(minus_schur_ref);
#endif // !__GPU_SCHUR_EASY_PROD_ONLY*/
		// debug - replace GPU results by CPU
#endif // __GPU_SCHUR_VERIFY_RESULT
	}

#ifdef __GPU_SCHUR_EASY_PROD_ONLY
	{
		CUberBlockMatrix V;
		U.TransposeTo(V); // because lower-triangular of lambda is not calculated
		// if the other half of the product runs on CPU, need V; U gets destroyed in the process
		minus_U_Cinv.MultiplyToWith/*_FBS<_TyLambdaMatrixBlockSizes,
			_TyLambdaMatrixBlockSizes>*/(schur_compl, V, true); // -U*(C^-1)V // UV is symmetric, the whole product should be symmetric, calculate only the upper triangular part
	}
#endif // __GPU_SCHUR_EASY_PROD_ONLY
#endif // 0

#ifdef __SCHUR_PROFILING
	double f_mul1_time = dt.f_Time();
#endif // __SCHUR_PROFILING

	/*minus_U_Cinv.Scale(-1.0);	// -U*(C^-1)

	double f_scale_time = dt.f_Time();*/

	//C_inv.Scale(-1.0);	// -(C^-1)

#ifdef __SCHUR_PROFILING
	double f_scale_time = 0;//dt.f_Time();
#endif // __SCHUR_PROFILING

	//CUberBlockMatrix schur_compl; // not needed afterwards
	//minus_U_Cinv.MultiplyToWith/*_FBS<_TyLambdaMatrixBlockSizes,
	//	_TyLambdaMatrixBlockSizes>*/(schur_compl, V, true); // -U*(C^-1)V // UV is symmetric, the whole product should be symmetric, calculate only the upper triangular part

	//printf("the product of all 3 matrices is " PRIsize " x " PRIsize " and will have " PRIsize " nnz\n",
	//	schur_compl.n_Row_Num(), schur_compl.n_Column_Num(), schur_compl.n_NonZero_Num());
	// debug

#ifdef __SCHUR_PROFILING
	//double f_mul1_time = dt.f_Time();
#endif // __SCHUR_PROFILING

	A.AddTo/*_FBS<_TyLambdaMatrixBlockSizes>*/(schur_compl); // -U*(C^-1)V + A // A is symmetric, if schur_compl is symmetric, the difference also is
	// compute left-hand side A - U(C^-1)V
	// todo - need multiplication with transpose matrices (in this case schur * U^T)

#ifdef __SCHUR_PROFILING
	double f_add_time = dt.f_Time();
#endif // __SCHUR_PROFILING

	// note that the sum and difference of two symmetric matrices is again symmetric,
	// but this is not always true for the product

	/*lambda_perm.Save_MatrixMarket("lambda_perm.mtx", "lambda_perm.bla");
	A.Save_MatrixMarket("lambda_perm00.mtx", "lambda_perm00.bla");
	U.Save_MatrixMarket("lambda_perm01.mtx", "lambda_perm01.bla");
	V.Save_MatrixMarket("lambda_perm10.mtx", "lambda_perm10.bla");
	C.Save_MatrixMarket("lambda_perm11.mtx", "lambda_perm11.bla");
	C_inv.Save_MatrixMarket("lambda_perm11_inv.mtx", "lambda_perm11_inv.bla");
	schur_compl.Save_MatrixMarket("schur.mtx", "schur.bla");*/
	/*lambda_perm.Rasterize("schur0_lambda_perm.tga", 3);
	A.Rasterize("schur1_lambda_perm00.tga", 3);
	U.Rasterize("schur2_lambda_perm01.tga", 3);
	V.Rasterize("schur3_lambda_perm10.tga", 3);
	C.Rasterize("schur4_lambda_perm11.tga", 3);
	schur_compl.Rasterize("schur5_A-(UC-1V).tga", 3);
	C_inv.Rasterize("schur6_lambda_perm11_inv.tga", 3);*/
	// debug

	if(m_double_workspace.capacity() < n_rhs_vector_size) {
		m_double_workspace.clear(); // avoid data copying
		m_double_workspace.reserve(std::max(2 * m_double_workspace.capacity(), n_rhs_vector_size));
	}
	m_double_workspace.resize(n_rhs_vector_size);
	double *p_double_workspace = &m_double_workspace[0];
	// alloc workspace

	lambda_perm.InversePermute_RightHandSide_Vector(p_double_workspace,
		&r_v_eta(0), n_rhs_vector_size, &m_order[0], m_order.size());
	// need to permute the vector !!

	Eigen::VectorXd v_x = Eigen::Map<Eigen::VectorXd>(p_double_workspace, n_pose_vector_size); // don't really need a copy, but need Eigen::VectorXd for _TyLinearSolverWrapper::Solve()
	Eigen::VectorXd v_l = Eigen::Map<Eigen::VectorXd>(p_double_workspace +
		n_pose_vector_size, n_landmark_vector_size); // need a copy, need one vector of workspace
	// get eta and cut it into pieces
	// \eta = | x |
	//        | l |

	// we are now solving:
	// \lambda          \eta
	// | A U | | dx | = | x |
	// | V C | | dl |   | l |

	minus_U_Cinv.PreMultiply_Add/*_FBS<_TyLambdaMatrixBlockSizes>*/(&v_x(0),
		n_pose_vector_size, &v_l(0), n_landmark_vector_size); // x - U(C^-1)l
	// compute right-hand side x - U(C^-1)l

#ifdef __SCHUR_PROFILING
	double f_RHS_time = dt.f_Time();
#endif // __SCHUR_PROFILING

	if(!b_keep_ordering)
		_TyLinearSolverWrapper::FinalBlockStructure(m_linear_solver, schur_compl); // the ordering on schur_compl will not change, can calculate it only in the first pass and then reuse
	bool b_result = _TyLinearSolverWrapper::Solve(m_linear_solver, schur_compl, v_x);
	Eigen::VectorXd &v_dx = v_x; // rename, solves inplace
	// solve for dx = A - U(C^-1)V / x

#ifdef __SCHUR_PROFILING
	double f_linsolve0_time = dt.f_Time();
#endif // __SCHUR_PROFILING

	// note that schur_compl only contains pose-sized blocks when guided ordering is used! could optimize for that
	// also note that schur_compl is not completely dense if it is not many times smaller than C

	Eigen::Map<Eigen::VectorXd>(p_double_workspace, n_pose_vector_size) = v_dx; // an unnecessary copy, maybe could work around
	// obtained the first part of the solution

#if 1
	Eigen::Map<Eigen::VectorXd> v_dl(p_double_workspace +
		n_pose_vector_size, n_landmark_vector_size); // calculated inplace
	v_dl.setZero();
	//V.PreMultiply_Add/*_FBS<_TyLambdaMatrixBlockSizes>*/(&v_dl(0),
	//	n_landmark_vector_size, &v_dx(0), n_pose_vector_size); // V * dx
	U.PostMultiply_Add/*_FBS<_TyLambdaMatrixBlockSizes>*/(&v_dl(0),
		n_landmark_vector_size, &v_dx(0), n_pose_vector_size); // dx^T * U^T = (V * dx)^T and the vector transposes are ignored
	v_l -= v_dl; // (l - dl)
	v_dl.setZero();
	C_inv.PreMultiply_Add/*_FBS<_TyLambdaMatrixBlockSizes>*/(&v_dl(0),
		n_landmark_vector_size, &v_l(0), n_landmark_vector_size); // (C^-1)(l - V * dx)
	// solve for dl = (C^-1)(l - V * dx)
	// the second part of the solution is calculated inplace in the dest vector
#else // 1
	Eigen::Map<Eigen::VectorXd> v_dl(p_double_workspace +
		n_pose_vector_size, n_landmark_vector_size); // calculated inplace
	v_l = -v_l; // trades setZero() for sign negation and a smaller sign negation above
	V.PreMultiply_Add/*_FBS<_TyLambdaMatrixBlockSizes>*/(&v_l(0),
		n_landmark_vector_size, &v_dx(0), n_pose_vector_size); // V * dx
	// l = V * dx - l
	v_dl.setZero();
	C_inv.PreMultiply_Add/*_FBS<_TyLambdaMatrixBlockSizes>*/(&v_dl(0),
		n_landmark_vector_size, &v_l(0), n_landmark_vector_size); // (C^-1)(l - V * dx)
	// solve for dl = (C^-1)(l - V * dx) = -(C^-1)(V * dx - l)
	// the second part of the solution is calculated inplace in the dest vector
	// t_odo - carry this modification to the original schur as well
	// todo - free U from memory after calculating U(C^-1) in the original schur as well - or do it inplace using a specialized diagonal multiply kernel
#endif // 1

	lambda_perm.Permute_RightHandSide_Vector(&r_v_eta(0), p_double_workspace,
		n_rhs_vector_size, &m_order[0], m_order.size());
	// permute back!

#ifdef __SCHUR_PROFILING
	double f_linsolve1_time = dt.f_Time();
	double f_totel_time = f_reperm_time + f_slice_time + f_transpose_time +
		f_inverse_time + f_mul0_time + f_scale_time + f_mul1_time +
		f_add_time + f_RHS_time + f_linsolve0_time + f_linsolve1_time;

	printf("Schur took %f sec, out of which:\n", f_totel_time);
	printf("   reperm: %f\n", f_reperm_time);
	printf("    slice: %f\n", f_slice_time);
	printf("transpose: %f\n", f_transpose_time);
	printf("  inverse: %f\n", f_inverse_time);
	printf(" multiply: %f, out of which:\n", f_mul0_time + f_scale_time + f_mul1_time);
	printf("\tdiag gemm: %f\n", f_mul0_time);
	printf("\t    scale: %f\n", f_scale_time);
	printf("\t     gemm: %f\n", f_mul1_time);
	printf("      add: %f\n", f_add_time);
	printf(" RHS prep: %f\n", f_RHS_time);
	printf("  cholsol: %f (" PRIsize " x " PRIsize ", " PRIsize " nnz (%.2f %%))\n",
		f_linsolve0_time, schur_compl.n_Row_Num(), schur_compl.n_Column_Num(),
		schur_compl.n_NonZero_Num(), 100 * float(schur_compl.n_NonZero_Num()) /
		(schur_compl.n_Row_Num() * schur_compl.n_Column_Num()));
	printf(" dy solve: %f\n", f_linsolve1_time);
	// debug - do some profiling
#endif // __SCHUR_PROFILING

	/*static size_t n_iter = 0;
	if(!n_iter) {
		std::string s_name;
		{
			char p_s_it_nr[256];
			sprintf(p_s_it_nr, "schur/lambda_perm_" PRIsize "_", n_iter);
			s_name = p_s_it_nr;
			++ n_iter;
		}
		A.Save_MatrixMarket((s_name + "00.mtx").c_str(), (s_name + "00.bla").c_str());
		U.Save_MatrixMarket((s_name + "01.mtx").c_str(), (s_name + "01.bla").c_str());
		V.Save_MatrixMarket((s_name + "10.mtx").c_str(), (s_name + "10.bla").c_str());
		C.Save_MatrixMarket((s_name + "11.mtx").c_str(), (s_name + "11.bla").c_str());
		C_inv.Save_MatrixMarket((s_name + "11_inv.mtx").c_str(), (s_name + "11_inv.bla").c_str());
		schur_compl.Save_MatrixMarket((s_name + "schur.mtx").c_str(), (s_name + "schur.bla").c_str());
	}*/
	// debug - dump the matrices

	return b_result;
}

/*
 *								=== ~CLinearSolver_Schur_GPUBase ===
 */

#else // /*(_WIN32 || _WIN64) &&*/ !__DISABLE_GPU

// GPU disabled, fallback to reference CPU implementation

/*
 *								=== CLinearSolver_DenseGPU ===
 */

bool CLinearSolver_DenseGPU::b_cula_initialized = false;
size_t CLinearSolver_DenseGPU::n_instance_num = 0;

CLinearSolver_DenseGPU::CLinearSolver_DenseGPU() // throw(std::runtime_error)
{
	if(!n_instance_num) {
		fprintf(stderr, "warning: built without GPU support: fallback to software\n");
		//fprintf(stderr, "warning: this software implementation is slower than SLAM++ default\n"); // not true, dense Eigen would be used anyway and the non-fbs path won't be called
		++ n_instance_num;
	}
}

CLinearSolver_DenseGPU::~CLinearSolver_DenseGPU()
{}

void CLinearSolver_DenseGPU::Free_Memory()
{
	{
		Eigen::MatrixXd empty;
		std::swap(empty, m_t_lambda);
	}
}

bool CLinearSolver_DenseGPU::Solve_PosDef(const CUberBlockMatrix &r_lambda, Eigen::VectorXd &r_eta) // throw(std::bad_alloc)
{
	r_lambda.Convert_to_Dense(m_t_lambda);

	typedef Eigen::LLT<Eigen::MatrixXd, Eigen::Upper> _TyDecomposition; /**< @brief matrix factorization type (Cholesky) */
	_TyDecomposition m_t_factorization;

	m_t_factorization.compute(m_t_lambda);
	if(m_t_factorization.info() != Eigen::Success)
		return false;
	// Cholesky

	r_eta = m_t_factorization.solve(r_eta);
	// solve

	return true;
}

/*
 *								=== ~CLinearSolver_DenseGPU ===
 */

/*
 *								=== CLinearSolver_Schur_GPUBase ===
 */

CLinearSolver_Schur_GPUBase::CLinearSolver_Schur_GPUBase()
{}

CLinearSolver_Schur_GPUBase::~CLinearSolver_Schur_GPUBase()
{}

bool CLinearSolver_Schur_GPUBase::SpDGEMM(CUberBlockMatrix &r_dest, const CUberBlockMatrix &r_a,
	const CUberBlockMatrix &r_b, bool b_upper_diag_only /*= false*/)
{
	return r_dest.ProductOf(r_a, r_b, b_upper_diag_only);
}

bool CLinearSolver_Schur_GPUBase::GPUSolve(const CUberBlockMatrix &r_lambda,
	Eigen::VectorXd &r_v_eta, bool b_keep_ordering, size_t n_landmark_dimension,
	std::vector<double> &m_double_workspace, const std::vector<size_t> &m_order,
	const size_t m_n_matrix_cut, _TyBaseSolver &m_linear_solver)
{
#if 1 // don't repeat code (though the code below works just fine)
	throw std::runtime_error("no GPU");
	return false;
#else // 1
	_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
	_ASSERTE(r_v_eta.rows() == r_lambda.n_Row_Num()); // make sure the vector has correct dimension

	_ASSERTE(r_lambda.b_SymmetricLayout());
	_ASSERTE(r_lambda.n_BlockColumn_Num() == m_order.size());
	_ASSERTE((m_order.empty() && !m_n_matrix_cut) || (!m_order.empty() &&
		m_n_matrix_cut > 0 && m_n_matrix_cut < SIZE_MAX && m_n_matrix_cut + 1 < m_order.size()));
	_ASSERTE(r_v_eta.rows() == r_lambda.n_Column_Num());

	CDeltaTimer dt;

	CUberBlockMatrix lambda_perm;
	r_lambda.Permute_UpperTriangular_To(lambda_perm, &m_order[0], m_order.size(), true);
	// reorder the matrix

	double f_reperm_time = dt.f_Time();

	const size_t n = lambda_perm.n_BlockColumn_Num();

	CUberBlockMatrix A, U, C, V;
	const size_t n_matrix_cut = m_n_matrix_cut; // antialiass
	lambda_perm.SliceTo(A, 0, n_matrix_cut, 0, n_matrix_cut, true);
	lambda_perm.SliceTo(U, 0, n_matrix_cut, n_matrix_cut, n, true);
	lambda_perm.SliceTo(C, n_matrix_cut, n, n_matrix_cut, n, true);
	// cut Lambda matrix into pieces
	// \lambda = | A U |
	//           | V C |

	const size_t n_rhs_vector_size = r_lambda.n_Column_Num();
	const size_t n_pose_vector_size = A.n_Column_Num(); // 6 * m_n_matrix_cut;
	const size_t n_landmark_vector_size = U.n_Column_Num(); // 3 * (n - m_n_matrix_cut);
	// not block columns! element ones

	double f_slice_time = dt.f_Time();

	U.TransposeTo(V);

	double f_transpose_time = dt.f_Time();

	_ASSERTE(C.b_BlockDiagonal()); // it is, unless the ordering is bad
	CUberBlockMatrix &C_inv = C; // can do it inplace
	if(n_landmark_dimension == 3) // this can be anticipated in BA
		C_inv.InverseOf_BlockDiag_FBS_Parallel<MakeTypelist_Safe((fbs_ut::CCTSize2D<3, 3>))>(C); // faster version, slightly less general
	else
		C_inv.InverseOf_Symmteric(C, true); // there is no non-FBS InverseOf_BlockDiag(), just use this one instead 
	// inverse of C

	double f_inverse_time = dt.f_Time();

	C_inv.Scale(-1.0);	// -U*(C^-1)

	double f_scale_time = dt.f_Time();

	CUberBlockMatrix minus_U_Cinv;
	U.MultiplyToWith/*_FBS<_TyLambdaMatrixBlockSizes,
		_TyLambdaMatrixBlockSizes>*/(minus_U_Cinv, C_inv);

	double f_mul0_time = dt.f_Time();

	CUberBlockMatrix schur_compl; // not needed afterwards
	minus_U_Cinv.MultiplyToWith/*_FBS<_TyLambdaMatrixBlockSizes,
		_TyLambdaMatrixBlockSizes>*/(schur_compl, V, true); // -U*(C^-1)V // UV is symmetric, the whole product should be symmetric, calculate only the upper triangular part

	double f_mul1_time = dt.f_Time();

	A.AddTo/*_FBS<_TyLambdaMatrixBlockSizes>*/(schur_compl); // -U*(C^-1)V + A // A is symmetric, if schur_compl is symmetric, the difference also is
	// compute left-hand side A - U(C^-1)V

	double f_add_time = dt.f_Time();

	// note that the sum and difference of two symmetric matrices is again symmetric,
	// but this is not always true for the product

	if(m_double_workspace.capacity() < n_rhs_vector_size) {
		m_double_workspace.clear(); // avoid data copying
		m_double_workspace.reserve(std::max(2 * m_double_workspace.capacity(), n_rhs_vector_size));
	}
	m_double_workspace.resize(n_rhs_vector_size);
	double *p_double_workspace = &m_double_workspace[0];
	// alloc workspace

	lambda_perm.InversePermute_RightHandSide_Vector(p_double_workspace,
		&r_v_eta(0), n_rhs_vector_size, &m_order[0], m_order.size());
	// need to permute the vector !!

	Eigen::VectorXd v_x = Eigen::Map<Eigen::VectorXd>(p_double_workspace, n_pose_vector_size); // don't really need a copy, but need Eigen::VectorXd for _TyLinearSolverWrapper::Solve()
	Eigen::VectorXd v_l = Eigen::Map<Eigen::VectorXd>(p_double_workspace +
		n_pose_vector_size, n_landmark_vector_size); // need a copy, need one vector of workspace
	// get eta and cut it into pieces
	// \eta = | x |
	//        | l |

	// we are now solving:
	// \lambda          \eta
	// | A U | | dx | = | x |
	// | V C | | dl |   | l |

	minus_U_Cinv.PreMultiply_Add/*_FBS<_TyLambdaMatrixBlockSizes>*/(&v_x(0),
		n_pose_vector_size, &v_l(0), n_landmark_vector_size); // x - U(C^-1)l
	// compute right-hand side x - U(C^-1)l

	double f_RHS_time = dt.f_Time();

	if(!b_keep_ordering)
		_TyLinearSolverWrapper::FinalBlockStructure(m_linear_solver, schur_compl); // the ordering on schur_compl will not change, can calculate it only in the first pass and then reuse
	bool b_result = _TyLinearSolverWrapper::Solve(m_linear_solver, schur_compl, v_x);
	Eigen::VectorXd &v_dx = v_x; // rename, solves inplace
	// solve for dx = A - U(C^-1)V / x

	double f_linsolve0_time = dt.f_Time();

	// note that schur_compl only contains pose-sized blocks when guided ordering is used! could optimize for that
	// also note that schur_compl is not completely dense if it is not many times smaller than C

	Eigen::Map<Eigen::VectorXd>(p_double_workspace, n_pose_vector_size) = v_dx; // an unnecessary copy, maybe could work around
	// obtained the first part of the solution

	Eigen::Map<Eigen::VectorXd> v_dl(p_double_workspace +
		n_pose_vector_size, n_landmark_vector_size); // calculated inplace
	v_l = -v_l; // trades setZero() for sign negation and a smaller sign negation above
	Eigen::VectorXd v_l_copy = v_l;

	//V.PreMultiply_Add/*_FBS<_TyLambdaMatrixBlockSizes>*/(&v_l(0),
	//	n_landmark_vector_size, &v_dx(0), n_pose_vector_size); // V * dx
	U.PostMultiply_Add_Parallel/*_FBS<_TyLambdaMatrixBlockSizes>*/(&v_l_copy(0),
		n_landmark_vector_size, &v_dx(0), n_pose_vector_size); // dx^T * U^T = (V * dx)^T and the vector transposes are ignored
	//printf("post-mad error %g (rel %g)\n", (v_l_copy - v_l).norm(), (v_l_copy - v_l).norm() / v_l.norm());

	// l = V * dx - l
	v_dl.setZero();
	C_inv.PreMultiply_Add/*_FBS<_TyLambdaMatrixBlockSizes>*/(&v_dl(0),
		n_landmark_vector_size, &v_l(0), n_landmark_vector_size); // (C^-1)(l - V * dx)
	// solve for dl = (C^-1)(l - V * dx) = -(C^-1)(V * dx - l)
	// the second part of the solution is calculated inplace in the dest vector
	// t_odo - carry this modification to the original schur as well
	// todo - free U from memory after calculating U(C^-1) in the original schur as well - or do it inplace using a specialized diagonal multiply kernel

	lambda_perm.Permute_RightHandSide_Vector(&r_v_eta(0), p_double_workspace,
		n_rhs_vector_size, &m_order[0], m_order.size());
	// permute back!

#if defined(_DEBUG) || defined(__SCHUR_PROFILING)
	double f_linsolve1_time = dt.f_Time();
	double f_totel_time = f_reperm_time + f_slice_time + f_transpose_time +
		f_inverse_time + f_mul0_time + f_scale_time + f_mul1_time +
		f_add_time + f_RHS_time + f_linsolve0_time + f_linsolve1_time;

	printf("Schur took %f sec, out of which:\n", f_totel_time);
	printf("   reperm: %f\n", f_reperm_time);
	printf("    slice: %f\n", f_slice_time);
	printf("transpose: %f\n", f_transpose_time);
	printf("  inverse: %f\n", f_inverse_time);
	printf(" multiply: %f, out of which:\n", f_mul0_time + f_scale_time + f_mul1_time);
	printf("\tdiag gemm: %f\n", f_mul0_time);
	printf("\t    scale: %f\n", f_scale_time);
	printf("\t     gemm: %f\n", f_mul1_time);
	printf("      add: %f\n", f_add_time);
	printf(" RHS prep: %f\n", f_RHS_time);
	printf("  cholsol: %f (" PRIsize " x " PRIsize ", " PRIsize " nnz (%.2f %%))\n",
		f_linsolve0_time, schur_compl.n_Row_Num(), schur_compl.n_Column_Num(),
		schur_compl.n_NonZero_Num(), 100 * float(schur_compl.n_NonZero_Num()) /
		(schur_compl.n_Row_Num() * schur_compl.n_Column_Num()));
	printf(" dy solve: %f\n", f_linsolve1_time);
	// debug - do some profiling
#endif // _DEBUG || __SCHUR_PROFILING

	return b_result;
#endif // 1
}

/*
 *								=== ~CLinearSolver_Schur_GPUBase ===
 */

#endif // !__DISABLE_GPU
