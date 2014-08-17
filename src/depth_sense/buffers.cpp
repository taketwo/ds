/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <iostream>
#include <cstring>

#include "depth_sense/buffers.h"

pcl::io::depth_sense::SingleBuffer::SingleBuffer ()
{
}

float
pcl::io::depth_sense::SingleBuffer::operator[] (size_t idx) const
{
  return (data_[idx]);
}

void
pcl::io::depth_sense::SingleBuffer::push (const float* data)
{
  data_ = data;
}

pcl::io::depth_sense::MedianBuffer::MedianBuffer (size_t size,
                                                  size_t window_size,
                                                  float invalid_value)
: size_ (size)
, window_size_ (window_size)
, midpoint_ (window_size_ / 2)
, current_idx_ (0)
, invalid_value_ (invalid_value)
{
  assert (size_ > 0);
  assert (window_size_ > 0);

  buffer_.resize (window_size_);
  for (size_t i = 0; i < window_size_; ++i)
    buffer_[i].resize (size_, invalid_value_);

  indices_.resize (size_);
  for (size_t i = 0; i < size_; ++i)
    indices_[i].resize (window_size_, 0);
}

float
pcl::io::depth_sense::MedianBuffer::operator[] (size_t idx) const
{
  return (buffer_[indices_[idx][midpoint_]][idx]);
}

void
pcl::io::depth_sense::MedianBuffer::push (const float* data)
{
  for (size_t i = 0; i < size_; ++i)
  {
    const float& value = data[i];
    const float& old_value = buffer_[current_idx_][i];
    if (value == old_value)
      continue;
    std::vector<unsigned char>& indices = indices_[i];
    // Find a position for value so that indices is sorted
    int low = 0;
    int high = window_size_ - 1;
    int midpoint = 0;
    int insert_at;
    while (low <= high)
    {
      midpoint = low + (high - low) / 2;
      const float& midpoint_value = buffer_[indices[midpoint]][i];
      if (value > midpoint_value)
      {
        low = midpoint + 1;
      }
      else
      {
        high = midpoint - 1;
      }
    }
    if (value > buffer_[indices[midpoint]][i])
      insert_at = low;
    else
      insert_at = high;
    if (value > old_value)
    {
      for (int j = 0; j < window_size_; ++j)
        if (indices[j] == current_idx_)
        {
          for (int k = j; k < insert_at; ++k)
            std::swap (indices[k], indices[k + 1]);
          break;
        }
    }
    else
    {
      for (int j = window_size_ - 1; j >= 0; --j)
        if (indices[j] == current_idx_)
        {
          for (int k = j; k > insert_at; --k)
            std::swap (indices[k], indices[k - 1]);
          break;
        }
    }
  }

  memcpy (&buffer_[current_idx_][0], data, sizeof (float) * size_);
  if (++current_idx_ == window_size_)
    current_idx_ = 0;
}

