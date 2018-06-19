// BSD 2-Clause License

// Copyright (c) 2018, CNRS-LAAS
// Author: Florent Lamiraux
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

#define BOOST_TEST_MODULE trivial
#include <boost/test/included/unit_test.hpp>
#include <jcqp/jcqp.hh>

BOOST_AUTO_TEST_CASE (trivial) {
  Eigen::Matrix3d G (Eigen::Matrix3d::Identity ());
  Eigen::Vector3d g0; g0.setZero ();
  Eigen::MatrixXd CE (3,0);
  Eigen::VectorXd ce0 (0);
  Eigen::MatrixXd CI (3,0);
  Eigen::VectorXd ci0 (0);
  Eigen::Vector3d x, expectedX;
  expectedX.setZero ();

  double res = jcqp::solve <double> (G, g0, CE, ce0, CI, ci0, x);
  BOOST_CHECK (x.isApprox (expectedX));
  BOOST_CHECK (fabs (res) < 1e-6);
}
