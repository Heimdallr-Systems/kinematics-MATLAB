//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: inpolygon.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "inpolygon.h"
#include "abs.h"
#include "eml_int_forloop_overflow_check.h"
#include <cmath>

// Function Declarations
namespace Codegen {
namespace coder {
static void b_computeRange(const double xv[3], int nloops, const int first[3],
                           const int last[3], double *minxv, double *maxxv);

static void b_computeScaleFactors(const double xv[3], const double yv[3],
                                  int nloops, const int first[3],
                                  const int last[3], double scale[3]);

static void b_countLoops(const double xv[3], const double yv[3], int *nloops,
                         int first[3], int last[3]);

static bool b_scalarInpolygon(double xj, double yj, const double xv[3],
                              const double yv[3], int nloops,
                              const int first[3], const int last[3],
                              double minxv, double maxxv, double minyv,
                              double maxyv, const double scale[3]);

static void computeRange(const double xv[4], int nloops, const int first[4],
                         const int last[4], double *minxv, double *maxxv);

static void computeScaleFactors(const double xv[4], const double yv[4],
                                int nloops, const int first[4],
                                const int last[4], double scale[4]);

static void contrib(double x1, double b_y1, double x2, double y2,
                    signed char quad1, signed char quad2, double scale,
                    signed char *diffQuad, bool *onj);

static void countLoops(const double xv[4], const double yv[4], int *nloops,
                       int first[4], int last[4]);

static void findLast(int *k, int nx);

static bool scalarInpolygon(double xj, double yj, const double xv[4],
                            const double yv[4], int nloops, const int first[4],
                            const int last[4], double minxv, double maxxv,
                            double minyv, double maxyv, const double scale[4]);

static double scalemax(double a, double b);

} // namespace coder
} // namespace Codegen

// Function Definitions
//
// Arguments    : const double xv[3]
//                int nloops
//                const int first[3]
//                const int last[3]
//                double *minxv
//                double *maxxv
// Return Type  : void
//
namespace Codegen {
namespace coder {
static void b_computeRange(const double xv[3], int nloops, const int first[3],
                           const int last[3], double *minxv, double *maxxv)
{
  double minxv_tmp;
  minxv_tmp = xv[first[0] - 1];
  *minxv = minxv_tmp;
  *maxxv = minxv_tmp;
  if ((1 <= nloops) && (nloops > 2147483646)) {
    check_forloop_overflow_error(true);
  }
  for (int k{0}; k < nloops; k++) {
    int a;
    int b;
    bool overflow;
    a = first[k];
    b = last[k];
    overflow = ((a <= b) && (b > 2147483646));
    if (overflow) {
      check_forloop_overflow_error(true);
    }
    for (int j{a}; j <= b; j++) {
      double d;
      d = xv[j - 1];
      if (d < (*minxv)) {
        *minxv = d;
      } else if (d > (*maxxv)) {
        *maxxv = d;
      } else {
        /* no actions */
      }
    }
  }
}

//
// Arguments    : const double xv[3]
//                const double yv[3]
//                int nloops
//                const int first[3]
//                const int last[3]
//                double scale[3]
// Return Type  : void
//
static void b_computeScaleFactors(const double xv[3], const double yv[3],
                                  int nloops, const int first[3],
                                  const int last[3], double scale[3])
{
  scale[0] = 0.0;
  scale[1] = 0.0;
  scale[2] = 0.0;
  if ((1 <= nloops) && (nloops > 2147483646)) {
    check_forloop_overflow_error(true);
  }
  for (int j{0}; j < nloops; j++) {
    int a;
    int b;
    int i;
    bool overflow;
    a = first[j];
    i = last[j];
    b = i - 1;
    overflow = ((a <= (i - 1)) && ((i - 1) > 2147483646));
    if (overflow) {
      check_forloop_overflow_error(true);
    }
    for (int b_i{a}; b_i <= b; b_i++) {
      scale[b_i - 1] = scalemax(b_abs(0.5 * (xv[b_i - 1] + xv[b_i])),
                                b_abs(0.5 * (yv[b_i - 1] + yv[b_i]))) *
                       6.6613381477509392E-16;
    }
    int i1;
    int i2;
    i1 = first[j];
    i2 = last[j];
    scale[i2 - 1] = scalemax(b_abs(0.5 * (xv[i2 - 1] + xv[i1 - 1])),
                             b_abs(0.5 * (yv[i2 - 1] + yv[i1 - 1]))) *
                    6.6613381477509392E-16;
  }
}

//
// Arguments    : const double xv[3]
//                const double yv[3]
//                int *nloops
//                int first[3]
//                int last[3]
// Return Type  : void
//
static void b_countLoops(const double xv[3], const double yv[3], int *nloops,
                         int first[3], int last[3])
{
  int k;
  *nloops = 0;
  first[0] = 0;
  last[0] = 0;
  first[1] = 0;
  last[1] = 0;
  first[2] = 0;
  last[2] = 0;
  k = 1;
  while (k <= 3) {
    int kfirst;
    (*nloops)++;
    kfirst = k - 1;
    first[(*nloops) - 1] = k;
    findLast(&k, 3);
    if ((xv[k - 1] == xv[kfirst]) && (yv[k - 1] == yv[kfirst])) {
      last[(*nloops) - 1] = k - 1;
    } else {
      last[(*nloops) - 1] = k;
    }
    k += 2;
  }
}

//
// Arguments    : double xj
//                double yj
//                const double xv[3]
//                const double yv[3]
//                int nloops
//                const int first[3]
//                const int last[3]
//                double minxv
//                double maxxv
//                double minyv
//                double maxyv
//                const double scale[3]
// Return Type  : bool
//
static bool b_scalarInpolygon(double xj, double yj, const double xv[3],
                              const double yv[3], int nloops,
                              const int first[3], const int last[3],
                              double minxv, double maxxv, double minyv,
                              double maxyv, const double scale[3])
{
  signed char b_dquad;
  signed char dquad;
  bool b_onj;
  bool inj;
  bool onj;
  inj = false;
  if ((((xj >= minxv) && (xj <= maxxv)) && (yj >= minyv)) && (yj <= maxyv)) {
    int k;
    signed char sdq;
    sdq = 0;
    if ((1 <= nloops) && (nloops > 2147483646)) {
      check_forloop_overflow_error(true);
    }
    k = 0;
    int exitg2;
    do {
      exitg2 = 0;
      if (k <= (nloops - 1)) {
        double xv2;
        double xvFirst;
        double yv2;
        double yvFirst;
        int exitg1;
        int i;
        signed char quad2;
        signed char quadFirst;
        bool overflow;
        xvFirst = xv[first[k] - 1] - xj;
        yvFirst = yv[first[k] - 1] - yj;
        if (xvFirst > 0.0) {
          if (yvFirst > 0.0) {
            quadFirst = 0;
          } else {
            quadFirst = 3;
          }
        } else if (yvFirst > 0.0) {
          quadFirst = 1;
        } else {
          quadFirst = 2;
        }
        xv2 = xvFirst;
        yv2 = yvFirst;
        quad2 = quadFirst;
        if (first[k] > (last[k] - 1)) {
          overflow = false;
        } else {
          overflow = ((last[k] - 1) > 2147483646);
        }
        if (overflow) {
          check_forloop_overflow_error(true);
        }
        i = first[k];
        do {
          exitg1 = 0;
          if (i <= (last[k] - 1)) {
            double xv1;
            double yv1;
            signed char quad1;
            xv1 = xv2;
            yv1 = yv2;
            xv2 = xv[i] - xj;
            yv2 = yv[i] - yj;
            quad1 = quad2;
            if (xv2 > 0.0) {
              if (yv2 > 0.0) {
                quad2 = 0;
              } else {
                quad2 = 3;
              }
            } else if (yv2 > 0.0) {
              quad2 = 1;
            } else {
              quad2 = 2;
            }
            contrib(xv1, yv1, xv2, yv2, quad1, quad2, scale[i - 1], &b_dquad,
                    &b_onj);
            if (b_onj) {
              inj = true;
              exitg1 = 1;
            } else {
              sdq = static_cast<signed char>(sdq + b_dquad);
              i++;
            }
          } else {
            contrib(xv2, yv2, xvFirst, yvFirst, quad2, quadFirst,
                    scale[last[k] - 1], &dquad, &onj);
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = 1;
        } else if (onj) {
          inj = true;
          exitg2 = 1;
        } else {
          sdq = static_cast<signed char>(sdq + dquad);
          k++;
        }
      } else {
        inj = (sdq != 0);
        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }
  return inj;
}

//
// Arguments    : const double xv[4]
//                int nloops
//                const int first[4]
//                const int last[4]
//                double *minxv
//                double *maxxv
// Return Type  : void
//
static void computeRange(const double xv[4], int nloops, const int first[4],
                         const int last[4], double *minxv, double *maxxv)
{
  double minxv_tmp;
  minxv_tmp = xv[first[0] - 1];
  *minxv = minxv_tmp;
  *maxxv = minxv_tmp;
  if ((1 <= nloops) && (nloops > 2147483646)) {
    check_forloop_overflow_error(true);
  }
  for (int k{0}; k < nloops; k++) {
    int a;
    int b;
    bool overflow;
    a = first[k];
    b = last[k];
    overflow = ((a <= b) && (b > 2147483646));
    if (overflow) {
      check_forloop_overflow_error(true);
    }
    for (int j{a}; j <= b; j++) {
      double d;
      d = xv[j - 1];
      if (d < (*minxv)) {
        *minxv = d;
      } else if (d > (*maxxv)) {
        *maxxv = d;
      } else {
        /* no actions */
      }
    }
  }
}

//
// Arguments    : const double xv[4]
//                const double yv[4]
//                int nloops
//                const int first[4]
//                const int last[4]
//                double scale[4]
// Return Type  : void
//
static void computeScaleFactors(const double xv[4], const double yv[4],
                                int nloops, const int first[4],
                                const int last[4], double scale[4])
{
  scale[0] = 0.0;
  scale[1] = 0.0;
  scale[2] = 0.0;
  scale[3] = 0.0;
  if ((1 <= nloops) && (nloops > 2147483646)) {
    check_forloop_overflow_error(true);
  }
  for (int j{0}; j < nloops; j++) {
    int a;
    int b;
    int i;
    bool overflow;
    a = first[j];
    i = last[j];
    b = i - 1;
    overflow = ((a <= (i - 1)) && ((i - 1) > 2147483646));
    if (overflow) {
      check_forloop_overflow_error(true);
    }
    for (int b_i{a}; b_i <= b; b_i++) {
      scale[b_i - 1] = scalemax(b_abs(0.5 * (xv[b_i - 1] + xv[b_i])),
                                b_abs(0.5 * (yv[b_i - 1] + yv[b_i]))) *
                       6.6613381477509392E-16;
    }
    int i1;
    int i2;
    i1 = first[j];
    i2 = last[j];
    scale[i2 - 1] = scalemax(b_abs(0.5 * (xv[i2 - 1] + xv[i1 - 1])),
                             b_abs(0.5 * (yv[i2 - 1] + yv[i1 - 1]))) *
                    6.6613381477509392E-16;
  }
}

//
// Arguments    : double x1
//                double b_y1
//                double x2
//                double y2
//                signed char quad1
//                signed char quad2
//                double scale
//                signed char *diffQuad
//                bool *onj
// Return Type  : void
//
static void contrib(double x1, double b_y1, double x2, double y2,
                    signed char quad1, signed char quad2, double scale,
                    signed char *diffQuad, bool *onj)
{
  double cp;
  *onj = false;
  *diffQuad = static_cast<signed char>(quad2 - quad1);
  cp = (x1 * y2) - (x2 * b_y1);
  if (b_abs(cp) < scale) {
    *onj = (((x1 * x2) + (b_y1 * y2)) <= 0.0);
    if (((*diffQuad) == 2) || ((*diffQuad) == -2)) {
      *diffQuad = 0;
    } else {
      switch (*diffQuad) {
      case -3:
        *diffQuad = 1;
        break;
      case 3:
        *diffQuad = -1;
        break;
      default:
        /* no actions */
        break;
      }
    }
  } else if (cp < 0.0) {
    switch (*diffQuad) {
    case 2:
      *diffQuad = -2;
      break;
    case -3:
      *diffQuad = 1;
      break;
    case 3:
      *diffQuad = -1;
      break;
    default:
      /* no actions */
      break;
    }
  } else {
    switch (*diffQuad) {
    case -2:
      *diffQuad = 2;
      break;
    case -3:
      *diffQuad = 1;
      break;
    case 3:
      *diffQuad = -1;
      break;
    default:
      /* no actions */
      break;
    }
  }
}

//
// Arguments    : const double xv[4]
//                const double yv[4]
//                int *nloops
//                int first[4]
//                int last[4]
// Return Type  : void
//
static void countLoops(const double xv[4], const double yv[4], int *nloops,
                       int first[4], int last[4])
{
  int k;
  *nloops = 0;
  first[0] = 0;
  last[0] = 0;
  first[1] = 0;
  last[1] = 0;
  first[2] = 0;
  last[2] = 0;
  first[3] = 0;
  last[3] = 0;
  k = 1;
  while (k <= 4) {
    int kfirst;
    (*nloops)++;
    kfirst = k - 1;
    first[(*nloops) - 1] = k;
    findLast(&k, 4);
    if ((xv[k - 1] == xv[kfirst]) && (yv[k - 1] == yv[kfirst])) {
      last[(*nloops) - 1] = k - 1;
    } else {
      last[(*nloops) - 1] = k;
    }
    k += 2;
  }
}

//
// Arguments    : int *k
//                int nx
// Return Type  : void
//
static void findLast(int *k, int nx)
{
  while ((*k) < nx) {
    (*k)++;
  }
}

//
// Arguments    : double xj
//                double yj
//                const double xv[4]
//                const double yv[4]
//                int nloops
//                const int first[4]
//                const int last[4]
//                double minxv
//                double maxxv
//                double minyv
//                double maxyv
//                const double scale[4]
// Return Type  : bool
//
static bool scalarInpolygon(double xj, double yj, const double xv[4],
                            const double yv[4], int nloops, const int first[4],
                            const int last[4], double minxv, double maxxv,
                            double minyv, double maxyv, const double scale[4])
{
  signed char b_dquad;
  signed char dquad;
  bool b_onj;
  bool inj;
  bool onj;
  inj = false;
  if ((((xj >= minxv) && (xj <= maxxv)) && (yj >= minyv)) && (yj <= maxyv)) {
    int k;
    signed char sdq;
    sdq = 0;
    if ((1 <= nloops) && (nloops > 2147483646)) {
      check_forloop_overflow_error(true);
    }
    k = 0;
    int exitg2;
    do {
      exitg2 = 0;
      if (k <= (nloops - 1)) {
        double xv2;
        double xvFirst;
        double yv2;
        double yvFirst;
        int exitg1;
        int i;
        signed char quad2;
        signed char quadFirst;
        bool overflow;
        xvFirst = xv[first[k] - 1] - xj;
        yvFirst = yv[first[k] - 1] - yj;
        if (xvFirst > 0.0) {
          if (yvFirst > 0.0) {
            quadFirst = 0;
          } else {
            quadFirst = 3;
          }
        } else if (yvFirst > 0.0) {
          quadFirst = 1;
        } else {
          quadFirst = 2;
        }
        xv2 = xvFirst;
        yv2 = yvFirst;
        quad2 = quadFirst;
        if (first[k] > (last[k] - 1)) {
          overflow = false;
        } else {
          overflow = ((last[k] - 1) > 2147483646);
        }
        if (overflow) {
          check_forloop_overflow_error(true);
        }
        i = first[k];
        do {
          exitg1 = 0;
          if (i <= (last[k] - 1)) {
            double xv1;
            double yv1;
            signed char quad1;
            xv1 = xv2;
            yv1 = yv2;
            xv2 = xv[i] - xj;
            yv2 = yv[i] - yj;
            quad1 = quad2;
            if (xv2 > 0.0) {
              if (yv2 > 0.0) {
                quad2 = 0;
              } else {
                quad2 = 3;
              }
            } else if (yv2 > 0.0) {
              quad2 = 1;
            } else {
              quad2 = 2;
            }
            contrib(xv1, yv1, xv2, yv2, quad1, quad2, scale[i - 1], &b_dquad,
                    &b_onj);
            if (b_onj) {
              inj = true;
              exitg1 = 1;
            } else {
              sdq = static_cast<signed char>(sdq + b_dquad);
              i++;
            }
          } else {
            contrib(xv2, yv2, xvFirst, yvFirst, quad2, quadFirst,
                    scale[last[k] - 1], &dquad, &onj);
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = 1;
        } else if (onj) {
          inj = true;
          exitg2 = 1;
        } else {
          sdq = static_cast<signed char>(sdq + dquad);
          k++;
        }
      } else {
        inj = (sdq != 0);
        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }
  return inj;
}

//
// Arguments    : double a
//                double b
// Return Type  : double
//
static double scalemax(double a, double b)
{
  double s;
  if ((a > 1.0) && (b > 1.0)) {
    s = a * b;
  } else {
    s = std::fmax(b, a);
  }
  return s;
}

//
// Arguments    : double x
//                double y
//                const double xv[3]
//                const double yv[3]
// Return Type  : bool
//
bool b_inpolygon(double x, double y, const double xv[3], const double yv[3])
{
  double scale[3];
  double maxxv;
  double maxyv;
  double minxv;
  double minyv;
  int first[3];
  int last[3];
  int nloops;
  bool in;
  in = false;
  b_countLoops(xv, yv, &nloops, first, last);
  if (nloops != 0) {
    b_computeRange(xv, nloops, first, last, &minxv, &maxxv);
    b_computeRange(yv, nloops, first, last, &minyv, &maxyv);
    b_computeScaleFactors(xv, yv, nloops, first, last, scale);
    in = b_scalarInpolygon(x, y, xv, yv, nloops, first, last, minxv, maxxv,
                           minyv, maxyv, scale);
  }
  return in;
}

//
// Arguments    : double x
//                double y
//                const double xv[4]
//                const double yv[4]
// Return Type  : bool
//
bool inpolygon(double x, double y, const double xv[4], const double yv[4])
{
  double scale[4];
  double maxxv;
  double maxyv;
  double minxv;
  double minyv;
  int first[4];
  int last[4];
  int nloops;
  bool in;
  in = false;
  countLoops(xv, yv, &nloops, first, last);
  if (nloops != 0) {
    computeRange(xv, nloops, first, last, &minxv, &maxxv);
    computeRange(yv, nloops, first, last, &minyv, &maxyv);
    computeScaleFactors(xv, yv, nloops, first, last, scale);
    in = scalarInpolygon(x, y, xv, yv, nloops, first, last, minxv, maxxv, minyv,
                         maxyv, scale);
  }
  return in;
}

} // namespace coder
} // namespace Codegen

//
// File trailer for inpolygon.cpp
//
// [EOF]
//
