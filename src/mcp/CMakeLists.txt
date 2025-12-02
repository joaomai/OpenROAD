# Copyright (c) 2021, The Regents of the University of California
# All rights reserved.
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

include("openroad")

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

project(mcp)

swig_lib(NAME      mcp
         NAMESPACE mcp
         I_FILE    src/MinCutPlacer.i
         SWIG_INCLUDES ${ODB_HOME}/src/swig/common
                       ${ODB_HOME}/src/swig/tcl
                       ${ODB_HOME}/include
         SCRIPTS   src/MinCutPlacer.tcl
)

target_sources(mcp
  PRIVATE
    src/MakeMinCutPlacer.cpp
    src/MinCutPlacer.cpp
)

messages(TARGET mcp)

target_include_directories(mcp
  PUBLIC
    include
  PRIVATE
    src
)

target_link_libraries(mcp
    OpenSTA
    odb
    gpl
    Eigen3::Eigen
    Threads::Threads
    OpenMP::OpenMP_CXX
)

if(ENABLE_TESTS)
  add_subdirectory(test)
endif()
