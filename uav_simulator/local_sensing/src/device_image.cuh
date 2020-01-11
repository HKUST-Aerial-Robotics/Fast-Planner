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

#ifndef DEVICE_IMAGE_CUH
#define DEVICE_IMAGE_CUH

#include <assert.h>
#include <cuda_runtime.h>
#include "cuda_exception.cuh"

struct Size
{
  int width;
  int height;
};

template<typename ElementType>
struct DeviceImage
{
  __host__
  DeviceImage(size_t width, size_t height)
    : width(width),
      height(height)
  {
    cudaError err = cudaMallocPitch(
          &data,
          &pitch,
          width*sizeof(ElementType),
          height);
    if(err != cudaSuccess)
      throw CudaException("Image: unable to allocate pitched memory.", err);

    stride = pitch / sizeof(ElementType);

    err = cudaMalloc(
          &dev_ptr,
          sizeof(*this));
    if(err != cudaSuccess)
      throw CudaException("DeviceData, cannot allocate device memory to store image parameters.", err);

    err = cudaMemcpy(
          dev_ptr,
          this,
          sizeof(*this),
          cudaMemcpyHostToDevice);
    if(err != cudaSuccess)
      throw CudaException("DeviceData, cannot copy image parameters to device memory.", err);
  }

  __device__
  ElementType & operator()(size_t x, size_t y)
  {
    return atXY(x, y);
  }

  __device__
  const ElementType & operator()(size_t x, size_t y) const
  {
    return atXY(x, y);
  }

  __device__
  ElementType & atXY(size_t x, size_t y)
  {
    return data[y*stride+x];
  }

  __device__
  const ElementType & atXY(size_t x, size_t y) const
  {
    return data[y*stride+x];
  }

  /// Upload aligned_data_row_major to device memory
  __host__
  void setDevData(const ElementType * aligned_data_row_major)
  {
    const cudaError err = cudaMemcpy2D(
          data,
          pitch,
          aligned_data_row_major,
          width*sizeof(ElementType),
          width*sizeof(ElementType),
          height,
          cudaMemcpyHostToDevice);
    if(err != cudaSuccess)
      throw CudaException("Image: unable to copy data from host to device.", err);
  }

  /// Download the data from the device memory to aligned_data_row_major, a preallocated array in host memory
  __host__
  void getDevData(ElementType* aligned_data_row_major) const
  {
    const cudaError err = cudaMemcpy2D(
          aligned_data_row_major,      // destination memory address
          width*sizeof(ElementType),   // pitch of destination memory
          data,                        // source memory address
          pitch,                       // pitch of source memory
          width*sizeof(ElementType),   // width of matrix transfer (columns in bytes)
          height,                      // height of matrix transfer
          cudaMemcpyDeviceToHost);
    if(err != cudaSuccess)
      throw CudaException("Image: unable to copy data from device to host.", err);
  }

  __host__
  ~DeviceImage()
  {
    cudaError err = cudaFree(data);
    if(err != cudaSuccess)
      throw CudaException("Image: unable to free allocated memory.", err);
    err = cudaFree(dev_ptr);
    if(err != cudaSuccess)
      throw CudaException("Image: unable to free allocated memory.", err);
  }

  __host__
  cudaChannelFormatDesc getCudaChannelFormatDesc() const
  {
    return cudaCreateChannelDesc<ElementType>();
  }

  __host__
  void zero()
  {
    const cudaError err = cudaMemset2D(
          data,
          pitch,
          0,
          width*sizeof(ElementType),
          height);
    if(err != cudaSuccess)
      throw CudaException("Image: unable to zero.", err);
  }

  __host__
  DeviceImage<ElementType> & operator=(const DeviceImage<ElementType> &other_image)
  {
    if(this != &other_image)
    {
      assert(width  == other_image.width &&
             height == other_image.height);
      const cudaError err = cudaMemcpy2D(data,
                                         pitch,
                                         other_image.data,
                                         other_image.pitch,
                                         width*sizeof(ElementType),
                                         height,
                                         cudaMemcpyDeviceToDevice);
      if(err != cudaSuccess)
        throw CudaException("Image, operator '=': unable to copy data from another image.", err);
    }
    return *this;
  }

  // fields
  size_t width;
  size_t height;
  size_t pitch;
  size_t stride;
  ElementType * data;
  DeviceImage<ElementType> *dev_ptr;
};

#endif // DEVICE_IMAGE_CUH
