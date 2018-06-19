# BSD 2-Clause License

# Copyright (c) 2018, CNRS-LAAS
# Author: Florent Lamiraux
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

INCLUDE (cmake/base.cmake)
INCLUDE (cmake/boost.cmake)
INCLUDE(cmake/test.cmake)

SET (PROJECT_NAME jcqp)
SET (PROJECT_DESCRIPTION "Quadratic program solver")
SET (PROJECT_URL "http://github.com/stack-of-tasks/pinocchio")

SET (DOXYGEN_USE_MATHJAX YES)
# Disable -Werror on Unix for now.
SET (CXX_DISABLE_WERROR True)

SETUP_PROJECT ()

SET (${PROJECT_NAME}_HEADERS
  include/jcqp/jcqp.hh
  include/jcqp/jcqp.hxx
  )

ADD_REQUIRED_DEPENDENCY ("eigen3 >= 3.0.5")

#ADD_SUBDIRECTORY (src)
SET(BOOST_COMPONENTS unit_test_framework)
SEARCH_FOR_BOOST()
ADD_SUBDIRECTORY (tests)

SETUP_PROJECT_FINALIZE ()
