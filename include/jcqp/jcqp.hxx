// BSD 2-Clause License

// Copyright (c) 2018, CNRS-LAAS
// Author: Justin Carpentier, Florent Lamiraux
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef JCQP_JCQP_HXX
# define JCQP_JCQP_HXX

namespace jcqp {

  template < typename Scalar, typename GMatrixType, typename g0VectorType,
             typename CEMatrixType, typename ce0VectorType,
             typename CIMatrixType, typename ci0VectorType,
             typename resVectorType >
    Scalar solve (const GMatrixType& G, const g0VectorType& g0,
                  const CEMatrixType& CE, const ce0VectorType& ce0,
                  const CIMatrixType& CI, const ci0VectorType& ci0,
                  resVectorType& x)
  {
    x.setZero ();
    Eigen::VectorXd result (.5 * x.transpose () * G * x + g0.transpose () * x);
    CE.transpose () * x + ce0;
    CI.transpose () * x + ci0;
    return result [0];
  }

} // namespace jcqp

#endif // JCQP_JCQP_HXX
