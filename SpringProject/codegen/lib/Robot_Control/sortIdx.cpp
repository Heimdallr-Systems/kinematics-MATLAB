//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sortIdx.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "sortIdx.h"
#include "Robot_Control_rtwutil.h"
#include "Robot_Control_types.h"
#include "eml_int_forloop_overflow_check.h"

// Function Declarations
namespace Codegen {
static int asr_s32(int u, unsigned int n);

namespace coder {
namespace internal {
static void merge(int idx[4], double x[4], int offset, int np, int nq,
                  int iwork[4], double xwork[4]);

static void merge_block(int idx[4], double x[4], int offset, int n,
                        int preSortLevel, int iwork[4], double xwork[4]);

} // namespace internal
} // namespace coder
} // namespace Codegen

// Function Definitions
//
// Arguments    : int u
//                unsigned int n
// Return Type  : int
//
namespace Codegen {
static int asr_s32(int u, unsigned int n)
{
  int y;
  if (u >= 0) {
    y = static_cast<int>(static_cast<unsigned int>(
        (static_cast<unsigned int>(u)) >> (static_cast<unsigned long>(n))));
  } else {
    y = (-(static_cast<int>(static_cast<unsigned int>(
            (static_cast<unsigned int>(static_cast<int>(-1 - u))) >>
            (static_cast<unsigned long>(n)))))) -
        1;
  }
  return y;
}

//
// Arguments    : int idx[4]
//                double x[4]
//                int offset
//                int np
//                int nq
//                int iwork[4]
//                double xwork[4]
// Return Type  : void
//
namespace coder {
namespace internal {
static void merge(int idx[4], double x[4], int offset, int np, int nq,
                  int iwork[4], double xwork[4])
{
  if ((np != 0) && (nq != 0)) {
    int iout;
    int n_tmp;
    int p;
    int q;
    n_tmp = np + nq;
    if ((1 <= n_tmp) && (n_tmp > 2147483646)) {
      check_forloop_overflow_error(true);
    }
    for (int j{0}; j < n_tmp; j++) {
      int i;
      i = offset + j;
      iwork[j] = idx[i];
      xwork[j] = x[i];
    }
    p = 0;
    q = np;
    iout = offset - 1;
    int exitg1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork[p] <= xwork[q]) {
        idx[iout] = iwork[p];
        x[iout] = xwork[p];
        if ((p + 1) < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx[iout] = iwork[q];
        x[iout] = xwork[q];
        if ((q + 1) < n_tmp) {
          q++;
        } else {
          int offset1;
          offset1 = iout - p;
          if (((p + 1) <= np) && (np > 2147483646)) {
            check_forloop_overflow_error(true);
          }
          for (int b_j{p + 1}; b_j <= np; b_j++) {
            int i1;
            i1 = offset1 + b_j;
            idx[i1] = iwork[b_j - 1];
            x[i1] = xwork[b_j - 1];
          }
          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

//
// Arguments    : int idx[4]
//                double x[4]
//                int offset
//                int n
//                int preSortLevel
//                int iwork[4]
//                double xwork[4]
// Return Type  : void
//
static void merge_block(int idx[4], double x[4], int offset, int n,
                        int preSortLevel, int iwork[4], double xwork[4])
{
  static rtBoundsCheckInfo
      emlrtBCI{
          0,             // iFirst
          31,            // iLast
          490,           // lineNo
          24,            // colNo
          "",            // aName
          "merge_block", // fName
          "D:\\Program "
          "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
          "internal\\sortIdx.m", // pName
          1                      // checkKind
      };
  int bLen;
  int nBlocks;
  if ((preSortLevel < 0) || (preSortLevel > 31)) {
    rtDynamicBoundsError(preSortLevel, 0, 31, &emlrtBCI);
  }
  nBlocks = asr_s32(n, static_cast<unsigned int>(preSortLevel));
  bLen = ((static_cast<int>(1)) << (static_cast<unsigned long>(preSortLevel)));
  while (nBlocks > 1) {
    int bLen2;
    int nPairs;
    if ((nBlocks & 1) != 0) {
      int nTail;
      int tailOffset;
      nBlocks--;
      tailOffset = bLen * nBlocks;
      nTail = n - tailOffset;
      if (nTail > bLen) {
        merge(idx, x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
      }
    }
    bLen2 = bLen * 2;
    nPairs = asr_s32(nBlocks, 1U);
    for (int k{0}; k < nPairs; k++) {
      merge(idx, x, offset + (k * bLen2), bLen, bLen, iwork, xwork);
    }
    bLen = bLen2;
    nBlocks = nPairs;
  }
  if (n > bLen) {
    merge(idx, x, offset, bLen, n - bLen, iwork, xwork);
  }
}

//
// Arguments    : double x[4]
//                int idx[4]
// Return Type  : void
//
void sortIdx(double x[4], int idx[4])
{
  double x4[4];
  double xwork[4];
  double d;
  double d1;
  int iwork[4];
  int i1;
  int i2;
  int i3;
  int i4;
  signed char idx4[4];
  signed char perm_idx_0;
  signed char perm_idx_1;
  signed char perm_idx_2;
  signed char perm_idx_3;
  idx4[0] = 1;
  x4[0] = x[0];
  idx4[1] = 2;
  x4[1] = x[1];
  idx4[2] = 3;
  x4[2] = x[2];
  idx4[3] = 4;
  x4[3] = x[3];
  if (x4[0] <= x4[1]) {
    i1 = 1;
    i2 = 2;
  } else {
    i1 = 2;
    i2 = 1;
  }
  if (x4[2] <= x4[3]) {
    i3 = 3;
    i4 = 4;
  } else {
    i3 = 4;
    i4 = 3;
  }
  d = x4[i1 - 1];
  d1 = x4[i3 - 1];
  if (d <= d1) {
    double d3;
    d3 = x4[i2 - 1];
    if (d3 <= d1) {
      perm_idx_0 = static_cast<signed char>(i1);
      perm_idx_1 = static_cast<signed char>(i2);
      perm_idx_2 = static_cast<signed char>(i3);
      perm_idx_3 = static_cast<signed char>(i4);
    } else if (d3 <= x4[i4 - 1]) {
      perm_idx_0 = static_cast<signed char>(i1);
      perm_idx_1 = static_cast<signed char>(i3);
      perm_idx_2 = static_cast<signed char>(i2);
      perm_idx_3 = static_cast<signed char>(i4);
    } else {
      perm_idx_0 = static_cast<signed char>(i1);
      perm_idx_1 = static_cast<signed char>(i3);
      perm_idx_2 = static_cast<signed char>(i4);
      perm_idx_3 = static_cast<signed char>(i2);
    }
  } else {
    double d2;
    d2 = x4[i4 - 1];
    if (d <= d2) {
      if (x4[i2 - 1] <= d2) {
        perm_idx_0 = static_cast<signed char>(i3);
        perm_idx_1 = static_cast<signed char>(i1);
        perm_idx_2 = static_cast<signed char>(i2);
        perm_idx_3 = static_cast<signed char>(i4);
      } else {
        perm_idx_0 = static_cast<signed char>(i3);
        perm_idx_1 = static_cast<signed char>(i1);
        perm_idx_2 = static_cast<signed char>(i4);
        perm_idx_3 = static_cast<signed char>(i2);
      }
    } else {
      perm_idx_0 = static_cast<signed char>(i3);
      perm_idx_1 = static_cast<signed char>(i4);
      perm_idx_2 = static_cast<signed char>(i1);
      perm_idx_3 = static_cast<signed char>(i2);
    }
  }
  idx[0] = static_cast<int>(idx4[perm_idx_0 - 1]);
  idx[1] = static_cast<int>(idx4[perm_idx_1 - 1]);
  idx[2] = static_cast<int>(idx4[perm_idx_2 - 1]);
  idx[3] = static_cast<int>(idx4[perm_idx_3 - 1]);
  x[0] = x4[perm_idx_0 - 1];
  x[1] = x4[perm_idx_1 - 1];
  x[2] = x4[perm_idx_2 - 1];
  x[3] = x4[perm_idx_3 - 1];
  iwork[0] = 0;
  xwork[0] = 0.0;
  iwork[1] = 0;
  xwork[1] = 0.0;
  iwork[2] = 0;
  xwork[2] = 0.0;
  iwork[3] = 0;
  xwork[3] = 0.0;
  merge_block(idx, x, 0, 4, 2, iwork, xwork);
}

} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for sortIdx.cpp
//
// [EOF]
//
