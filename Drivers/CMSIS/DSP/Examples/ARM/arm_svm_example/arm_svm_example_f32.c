/* ----------------------------------------------------------------------
* Copyright (C) 2019-2020 ARM Limited. All rights reserved.
*
* $Date:         09. December 2019
* $Revision:     V1.0.0
*
* Project:       CMSIS DSP Library
* Title:         arm_svm_example_f32.c
*
* Description:   Example code demonstrating how to use SVM functions.
*
* Target Processor: Cortex-M/Cortex-A
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* -------------------------------------------------------------------- */

/**
 * @ingroup groupExamples
 */

/**
 * @defgroup SVMExample SVM Example
 *
 * \par Description:
 * \par
 * Demonstrates the use of SVM functions. It is complementing the tutorial
 * about classical ML with CMSIS-DSP and python scikit-learn.
 *
 */


/** \example arm_svm_example_f32.c
  */

#include <math.h>
#include <stdio.h>
#include "arm_math.h"

/* 
  The polynomial SVM instance containing all parameters.
  Those parameters can be generated with the python library scikit-learn.
 */
arm_svm_polynomial_instance_f32 params;

/*
  Parameters generated by a training of the SVM classifier
  using scikit-learn and some random input data.
 */
#define NB_SUPPORT_VECTORS 11

/*
  Dimension of the vector space. A vector is your feature.
  It could, for instance, be the pixels of a picture or the FFT of a signal.
 */
#define VECTOR_DIMENSION 2

const float32_t dualCoefficients[NB_SUPPORT_VECTORS]={-0.01628988f, -0.0971605f,
  -0.02707579f,  0.0249406f,   0.00223095f,  0.04117345f,
  0.0262687f,   0.00800358f,  0.00581823f,  0.02346904f,  0.00862162f}; /* Dual coefficients */

const float32_t supportVectors[NB_SUPPORT_VECTORS*VECTOR_DIMENSION]={ 1.2510991f,   0.47782799f,
 -0.32711859f, -1.49880648f, -0.08905047f,  1.31907242f,
  1.14059333f,  2.63443767f, -2.62561524f,  1.02120701f,
 -1.2361353f,  -2.53145187f,
  2.28308122f, -1.58185875f,  2.73955981f,  0.35759327f,
  0.56662986f,  2.79702016f,
 -2.51380816f,  1.29295364f, -0.56658669f, -2.81944734f}; /* Support vectors */

/*
  Class A is identified with value 0.
  Class B is identified with value 1.
  
  This array is used by the SVM functions to do a conversion and ease the comparison
  with the Python code where different values could be used.
 */
const int32_t   classes[2]={0,1};


int32_t main(void)
{
  /* Array of input data */
  float32_t in[VECTOR_DIMENSION];

  /* Result of the classifier */
  int32_t result;
  

  /*
    Initialization of the SVM instance parameters.
    Additional parameters (intercept, degree, coef0 and gamma) are also coming from Python.
   */
  arm_svm_polynomial_init_f32(&params,
    NB_SUPPORT_VECTORS,
    VECTOR_DIMENSION,
    -1.661719f,        /* Intercept */
    dualCoefficients,
    supportVectors,
    classes,
    3,                 /* degree */
    1.100000f,         /* Coef0 */
    0.500000f          /* Gamma */
  );


  /*
    Input data.
    It is corresponding to a point inside the first class.
   */
  in[0] = 0.4f;
  in[1] = 0.1f;

  arm_svm_polynomial_predict_f32(&params, in, &result);

  /* Result should be 0 : First class */
#if defined(SEMIHOSTING)
  printf("Result = %d\n", result);
#endif

  /*
    This input vector is corresponding to a point inside the second class.
   */
  in[0] = 3.0f;
  in[1] = 0.0f;

  arm_svm_polynomial_predict_f32(&params, in, &result);
  
  /* Result should be 1 : Second class */
#if defined(SEMIHOSTING)
  printf("Result = %d\n", result);
#endif

#if !defined(SEMIHOSTING)
  while (1); /* main function does not return */
#endif
}



