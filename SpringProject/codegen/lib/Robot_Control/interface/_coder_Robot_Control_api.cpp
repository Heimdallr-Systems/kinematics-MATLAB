//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_Robot_Control_api.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "_coder_Robot_Control_api.h"
#include "_coder_Robot_Control_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131610U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "Robot_Control",                                      // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[3];

static real_T (*b_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *r_II_B_dead,
                                   const char_T *identifier))[3];

static const mxArray *b_emlrt_marshallOut(const real_T u[4]);

static void b_emlrt_marshallOut(const real_T u[9], const mxArray *y);

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *T_I_B_dead,
                                   const char_T *identifier))[9];

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[9];

static void c_emlrt_marshallOut(const boolean_T u[4], const mxArray *y);

static boolean_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static boolean_T d_emlrt_marshallIn(const emlrtStack *sp,
                                    const mxArray *firstCall,
                                    const char_T *identifier);

static void d_emlrt_marshallOut(const real_T u[12], const mxArray *y);

static const mxArray *d_emlrt_marshallOut(const real_T u[3]);

static boolean_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                      const emlrtMsgIdentifier *parentId))[4];

static boolean_T (*e_emlrt_marshallIn(const emlrtStack *sp,
                                      const mxArray *legs_valid,
                                      const char_T *identifier))[4];

static const mxArray *e_emlrt_marshallOut(const boolean_T u[4]);

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *Theta,
                                 const char_T *identifier))[12];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[12];

static const mxArray *emlrt_marshallOut(const real_T u[12]);

static const mxArray *emlrt_marshallOut(const uint8_T u);

static void emlrt_marshallOut(const real_T u[3], const mxArray *y);

static const mxArray *emlrt_marshallOut(const real_T u);

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[12];

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *r_II_c_dead,
                                   const char_T *identifier))[12];

static uint8_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId);

static uint8_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *stage,
                                  const char_T *identifier);

static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *Euler_d,
                                   const char_T *identifier))[3];

static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[3];

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *gamma_m,
                                   const char_T *identifier))[36];

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[36];

static real_T (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[12];

static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3];

static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[9];

static boolean_T m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

static boolean_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                      const emlrtMsgIdentifier *msgId))[4];

static real_T (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[12];

static uint8_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId);

static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3];

static real_T (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[36];

// Function Definitions
//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[3]
//
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[3]
{
  real_T(*y)[3];
  y = k_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *r_II_B_dead
//                const char_T *identifier
// Return Type  : real_T (*)[3]
//
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *r_II_B_dead,
                                   const char_T *identifier))[3]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[3];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(r_II_B_dead), &thisId);
  emlrtDestroyArray(&r_II_B_dead);
  return y;
}

//
// Arguments    : const real_T u[9]
//                const mxArray *y
// Return Type  : void
//
static void b_emlrt_marshallOut(const real_T u[9], const mxArray *y)
{
  static const int32_T iv[2]{3, 3};
  emlrtMxSetData((mxArray *)y, (void *)(&u[0]));
  (void)emlrtSetDimensions((mxArray *)y, &iv[0], 2);
}

//
// Arguments    : const real_T u[4]
// Return Type  : const mxArray *
//
static const mxArray *b_emlrt_marshallOut(const real_T u[4])
{
  static const int32_T i{0};
  static const int32_T i1{4};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)(&i),
                              static_cast<int32_T>(mxDOUBLE_CLASS),
                              static_cast<int32_T>(mxREAL));
  emlrtMxSetData((mxArray *)m, (void *)(&u[0]));
  (void)emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *T_I_B_dead
//                const char_T *identifier
// Return Type  : real_T (*)[9]
//
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *T_I_B_dead,
                                   const char_T *identifier))[9]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[9];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = c_emlrt_marshallIn(sp, emlrtAlias(T_I_B_dead), &thisId);
  emlrtDestroyArray(&T_I_B_dead);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[9]
//
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[9]
{
  real_T(*y)[9];
  y = l_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const boolean_T u[4]
//                const mxArray *y
// Return Type  : void
//
static void c_emlrt_marshallOut(const boolean_T u[4], const mxArray *y)
{
  static const int32_T iv[2]{1, 4};
  emlrtMxSetData((mxArray *)y, (void *)(&u[0]));
  (void)emlrtSetDimensions((mxArray *)y, &iv[0], 2);
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : boolean_T
//
static boolean_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = m_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *firstCall
//                const char_T *identifier
// Return Type  : boolean_T
//
static boolean_T d_emlrt_marshallIn(const emlrtStack *sp,
                                    const mxArray *firstCall,
                                    const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  boolean_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(firstCall), &thisId);
  emlrtDestroyArray(&firstCall);
  return y;
}

//
// Arguments    : const real_T u[12]
//                const mxArray *y
// Return Type  : void
//
static void d_emlrt_marshallOut(const real_T u[12], const mxArray *y)
{
  static const int32_T iv[2]{3, 4};
  emlrtMxSetData((mxArray *)y, (void *)(&u[0]));
  (void)emlrtSetDimensions((mxArray *)y, &iv[0], 2);
}

//
// Arguments    : const real_T u[3]
// Return Type  : const mxArray *
//
static const mxArray *d_emlrt_marshallOut(const real_T u[3])
{
  static const int32_T i{0};
  static const int32_T i1{3};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)(&i),
                              static_cast<int32_T>(mxDOUBLE_CLASS),
                              static_cast<int32_T>(mxREAL));
  emlrtMxSetData((mxArray *)m, (void *)(&u[0]));
  (void)emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : boolean_T (*)[4]
//
static boolean_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                      const emlrtMsgIdentifier *parentId))[4]
{
  boolean_T(*y)[4];
  y = n_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *legs_valid
//                const char_T *identifier
// Return Type  : boolean_T (*)[4]
//
static boolean_T (*e_emlrt_marshallIn(const emlrtStack *sp,
                                      const mxArray *legs_valid,
                                      const char_T *identifier))[4]
{
  emlrtMsgIdentifier thisId;
  boolean_T(*y)[4];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = e_emlrt_marshallIn(sp, emlrtAlias(legs_valid), &thisId);
  emlrtDestroyArray(&legs_valid);
  return y;
}

//
// Arguments    : const boolean_T u[4]
// Return Type  : const mxArray *
//
static const mxArray *e_emlrt_marshallOut(const boolean_T u[4])
{
  static const int32_T iv[2]{0, 0};
  static const int32_T iv1[2]{1, 4};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateLogicalArray(2, &iv[0]);
  emlrtMxSetData((mxArray *)m, (void *)(&u[0]));
  (void)emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *Theta
//                const char_T *identifier
// Return Type  : real_T (*)[12]
//
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *Theta,
                                 const char_T *identifier))[12]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[12];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(sp, emlrtAlias(Theta), &thisId);
  emlrtDestroyArray(&Theta);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[12]
//
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[12]
{
  real_T(*y)[12];
  y = j_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const real_T u[3]
//                const mxArray *y
// Return Type  : void
//
static void emlrt_marshallOut(const real_T u[3], const mxArray *y)
{
  static const int32_T i{3};
  emlrtMxSetData((mxArray *)y, (void *)(&u[0]));
  (void)emlrtSetDimensions((mxArray *)y, &i, 1);
}

//
// Arguments    : const real_T u[12]
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const real_T u[12])
{
  static const int32_T i{0};
  static const int32_T i1{12};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)(&i),
                              static_cast<int32_T>(mxDOUBLE_CLASS),
                              static_cast<int32_T>(mxREAL));
  emlrtMxSetData((mxArray *)m, (void *)(&u[0]));
  (void)emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const real_T u
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const uint8_T u
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const uint8_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericMatrix(1, 1, static_cast<int32_T>(mxUINT8_CLASS),
                               static_cast<int32_T>(mxREAL));
  *((uint8_T *)emlrtMxGetData(m)) = u;
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *r_II_c_dead
//                const char_T *identifier
// Return Type  : real_T (*)[12]
//
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *r_II_c_dead,
                                   const char_T *identifier))[12]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[12];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(r_II_c_dead), &thisId);
  emlrtDestroyArray(&r_II_c_dead);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[12]
//
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[12]
{
  real_T(*y)[12];
  y = o_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : uint8_T
//
static uint8_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId)
{
  uint8_T y;
  y = p_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *stage
//                const char_T *identifier
// Return Type  : uint8_T
//
static uint8_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *stage,
                                  const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  uint8_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = g_emlrt_marshallIn(sp, emlrtAlias(stage), &thisId);
  emlrtDestroyArray(&stage);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *Euler_d
//                const char_T *identifier
// Return Type  : real_T (*)[3]
//
static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *Euler_d,
                                   const char_T *identifier))[3]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[3];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(sp, emlrtAlias(Euler_d), &thisId);
  emlrtDestroyArray(&Euler_d);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[3]
//
static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[3]
{
  real_T(*y)[3];
  y = q_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *gamma_m
//                const char_T *identifier
// Return Type  : real_T (*)[36]
//
static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *gamma_m,
                                   const char_T *identifier))[36]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[36];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = i_emlrt_marshallIn(sp, emlrtAlias(gamma_m), &thisId);
  emlrtDestroyArray(&gamma_m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[36]
//
static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[36]
{
  real_T(*y)[36];
  y = r_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[12]
//
static real_T (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[12]
{
  static const int32_T dims{12};
  real_T(*ret)[12];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)(&dims));
  ret = (real_T(*)[12])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[3]
//
static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3]
{
  static const int32_T dims{3};
  real_T(*ret)[3];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)(&dims));
  ret = (real_T(*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[9]
//
static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[9]
{
  static const int32_T dims[2]{3, 3};
  real_T(*ret)[9];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)(&dims[0]));
  ret = (real_T(*)[9])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : boolean_T
//
static boolean_T m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  boolean_T ret;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"logical",
                          false, 0U, (void *)(&dims));
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : boolean_T (*)[4]
//
static boolean_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                      const emlrtMsgIdentifier *msgId))[4]
{
  static const int32_T dims[2]{1, 4};
  boolean_T(*ret)[4];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"logical",
                          false, 2U, (void *)(&dims[0]));
  ret = (boolean_T(*)[4])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[12]
//
static real_T (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[12]
{
  static const int32_T dims[2]{3, 4};
  real_T(*ret)[12];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)(&dims[0]));
  ret = (real_T(*)[12])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : uint8_T
//
static uint8_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  uint8_T ret;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"uint8",
                          false, 0U, (void *)(&dims));
  ret = *((uint8_T *)emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[3]
//
static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3]
{
  static const int32_T dims[2]{1, 3};
  real_T(*ret)[3];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)(&dims[0]));
  ret = (real_T(*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[36]
//
static real_T (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[36]
{
  static const int32_T dims{36};
  real_T(*ret)[36];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)(&dims));
  ret = (real_T(*)[36])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const mxArray * const prhs[7]
//                int32_T nlhs
//                const mxArray *plhs[4]
// Return Type  : void
//
void CallTheDead_api(const mxArray *const prhs[7], int32_T nlhs,
                     const mxArray *plhs[4])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *prhs_copy_idx_0;
  const mxArray *prhs_copy_idx_1;
  const mxArray *prhs_copy_idx_2;
  const mxArray *prhs_copy_idx_3;
  const mxArray *prhs_copy_idx_4;
  const mxArray *prhs_copy_idx_5;
  const mxArray *prhs_copy_idx_6;
  real_T(*Theta)[12];
  real_T(*r_II_c_dead)[12];
  real_T(*T_I_B_dead)[9];
  real_T(*r_II_B_dead)[3];
  boolean_T(*legs_valid)[4];
  boolean_T(*prev_legs_valid)[4];
  boolean_T firstCall;
  st.tls = emlrtRootTLSGlobal;
  prhs_copy_idx_0 = prhs[0];
  prhs_copy_idx_1 = emlrtProtectR2012b(prhs[1], 1, true, -1);
  prhs_copy_idx_2 = emlrtProtectR2012b(prhs[2], 2, true, -1);
  prhs_copy_idx_3 = prhs[3];
  prhs_copy_idx_4 = prhs[4];
  prhs_copy_idx_5 = emlrtProtectR2012b(prhs[5], 5, true, -1);
  prhs_copy_idx_6 = emlrtProtectR2012b(prhs[6], 6, true, -1);
  // Marshall function inputs
  Theta = emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_0), "Theta");
  r_II_B_dead =
      b_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_1), "r_II_B_dead");
  T_I_B_dead =
      c_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_2), "T_I_B_dead");
  firstCall =
      d_emlrt_marshallIn(&st, emlrtAliasP(prhs_copy_idx_3), "firstCall");
  legs_valid =
      e_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_4), "legs_valid");
  prev_legs_valid =
      e_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_5), "prev_legs_valid");
  r_II_c_dead =
      f_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_6), "r_II_c_dead");
  // Invoke the target function
  CallTheDead(*Theta, *r_II_B_dead, *T_I_B_dead, firstCall, *legs_valid,
              *prev_legs_valid, *r_II_c_dead);
  // Marshall function outputs
  emlrt_marshallOut(*r_II_B_dead, prhs_copy_idx_1);
  plhs[0] = prhs_copy_idx_1;
  if (nlhs > 1) {
    b_emlrt_marshallOut(*T_I_B_dead, prhs_copy_idx_2);
    plhs[1] = prhs_copy_idx_2;
  }
  if (nlhs > 2) {
    c_emlrt_marshallOut(*prev_legs_valid, prhs_copy_idx_5);
    plhs[2] = prhs_copy_idx_5;
  }
  if (nlhs > 3) {
    d_emlrt_marshallOut(*r_II_c_dead, prhs_copy_idx_6);
    plhs[3] = prhs_copy_idx_6;
  }
}

//
// Arguments    : const mxArray * const prhs[5]
//                int32_T nlhs
//                const mxArray *plhs[7]
// Return Type  : void
//
void Robot_Control_api(const mxArray *const prhs[5], int32_T nlhs,
                       const mxArray *plhs[7])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  real_T(*gamma_m)[36];
  real_T(*Theta1_d_out)[4];
  real_T(*Theta2_d_out)[4];
  real_T(*Theta3_d_out)[4];
  real_T(*Euler_d)[3];
  real_T(*r_II_B_d)[3];
  real_T(*r_II_B_d_temp_out)[3];
  real_T phi_d_temp_out;
  boolean_T(*floor_toggle_out)[4];
  boolean_T(*legs_on_gnd)[4];
  boolean_T(*legs_valid_out)[4];
  boolean_T init_toggle;
  st.tls = emlrtRootTLSGlobal;
  Theta1_d_out = (real_T(*)[4])mxMalloc(sizeof(real_T[4]));
  Theta2_d_out = (real_T(*)[4])mxMalloc(sizeof(real_T[4]));
  Theta3_d_out = (real_T(*)[4])mxMalloc(sizeof(real_T[4]));
  r_II_B_d_temp_out = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  floor_toggle_out = (boolean_T(*)[4])mxMalloc(sizeof(boolean_T[4]));
  legs_valid_out = (boolean_T(*)[4])mxMalloc(sizeof(boolean_T[4]));
  // Marshall function inputs
  r_II_B_d = b_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "r_II_B_d");
  Euler_d = h_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "Euler_d");
  gamma_m = i_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "gamma_m");
  init_toggle = d_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "init_toggle");
  legs_on_gnd = e_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "legs_on_gnd");
  // Invoke the target function
  Robot_Control(*r_II_B_d, *Euler_d, *gamma_m, init_toggle, *legs_on_gnd,
                *Theta1_d_out, *Theta2_d_out, *Theta3_d_out, &phi_d_temp_out,
                *r_II_B_d_temp_out, *floor_toggle_out, *legs_valid_out);
  // Marshall function outputs
  plhs[0] = b_emlrt_marshallOut(*Theta1_d_out);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(*Theta2_d_out);
  }
  if (nlhs > 2) {
    plhs[2] = b_emlrt_marshallOut(*Theta3_d_out);
  }
  if (nlhs > 3) {
    plhs[3] = emlrt_marshallOut(phi_d_temp_out);
  }
  if (nlhs > 4) {
    plhs[4] = d_emlrt_marshallOut(*r_II_B_d_temp_out);
  }
  if (nlhs > 5) {
    plhs[5] = e_emlrt_marshallOut(*floor_toggle_out);
  }
  if (nlhs > 6) {
    plhs[6] = e_emlrt_marshallOut(*legs_valid_out);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void Robot_Control_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  (void)mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  Robot_Control_xil_terminate();
  Robot_Control_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void Robot_Control_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  (void)mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  (void)emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void Robot_Control_terminate()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

//
// Arguments    : const mxArray * const prhs[2]
//                int32_T nlhs
//                const mxArray *plhs[2]
// Return Type  : void
//
void getUp_api(const mxArray *const prhs[2], int32_T nlhs,
               const mxArray *plhs[2])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  real_T(*Theta)[12];
  real_T(*Theta_d)[12];
  uint8_T stage;
  st.tls = emlrtRootTLSGlobal;
  Theta_d = (real_T(*)[12])mxMalloc(sizeof(real_T[12]));
  // Marshall function inputs
  Theta = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "Theta");
  stage = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "stage");
  // Invoke the target function
  getUp(*Theta, &stage, *Theta_d);
  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(*Theta_d);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(stage);
  }
}

//
// File trailer for _coder_Robot_Control_api.cpp
//
// [EOF]
//
