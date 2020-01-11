// This file is part of REMODE - REgularized MOnocular Depth Estimation.
//
// Copyright (C) 2014 Matia Pizzoli <matia dot pizzoli at gmail dot com>
// Robotics and Perception Group, University of Zurich, Switzerland
// http://rpg.ifi.uzh.ch
//
// REMODE is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// REMODE is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef RMD_CUDA_EXCEPTION_CUH_
#define RMD_CUDA_EXCEPTION_CUH_

#include <sstream>
#include <cuda_runtime.h>


struct CudaException : public std::exception
{
  CudaException(const std::string& what, cudaError err)
    : what_(what), err_(err) {}
  virtual ~CudaException() throw() {}
  virtual const char* what() const throw()
  {
    std::stringstream description;
    description << "CudaException: " << what_ << std::endl;
    if(err_ != cudaSuccess)
    {
      description << "cudaError code: " << cudaGetErrorString(err_);
      description << " (" << err_ << ")" << std::endl;
    }
    return description.str().c_str();
  }
  std::string what_;
  cudaError err_;
};

#endif /* RMD_CUDA_EXCEPTION_CUH_ */
