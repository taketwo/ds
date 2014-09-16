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

#include <pcl/pcl_macros.h>

#include "depth_sense/buffers.h"

pcl::io::depth_sense::Buffer::Buffer (size_t size)
: size_ (size)
, invalid_value_ (std::numeric_limits<float>::quiet_NaN ())
{
}

pcl::io::depth_sense::Buffer::~Buffer ()
{
}

const float*
pcl::io::depth_sense::Buffer::allocateAndFillWithNaNs (size_t size)
{
  const float nan = std::numeric_limits<float>::quiet_NaN ();
  float* nans = new float[size];
  for (size_t i = 0; i < size; ++i)
    nans[i] = nan;
  return (nans);
}

pcl::io::depth_sense::SingleBuffer::SingleBuffer (size_t size)
: Buffer (size)
, data_ (allocateAndFillWithNaNs (size))
{
}

pcl::io::depth_sense::SingleBuffer::~SingleBuffer ()
{
  boost::mutex::scoped_lock lock (data_mutex_);
  delete[] data_;
}

float
pcl::io::depth_sense::SingleBuffer::operator[] (size_t idx) const
{
  assert (idx < size_);
  return (data_[idx]);
}

void
pcl::io::depth_sense::SingleBuffer::push (const float* data)
{
  assert ((sizeof (data) / sizeof (float)) == size_);
  boost::mutex::scoped_lock lock (data_mutex_);
  delete[] data_;
  data_ = data;
}

pcl::io::depth_sense::MedianBuffer::MedianBuffer (size_t size,
                                                  size_t window_size)
: Buffer (size)
, window_size_ (window_size)
, midpoint_ (window_size_ / 2)
, data_current_idx_ (window_size_ - 1)
{
  assert (size_ > 0);
  assert (window_size_ > 0 &&
          window_size_ <= std::numeric_limits<unsigned char>::max ());

  for (size_t i = 0; i < window_size_; ++i)
    data_.push_back (allocateAndFillWithNaNs (size_));

  data_argsort_indices_.resize (size_);
  for (size_t i = 0; i < size_; ++i)
  {
    data_argsort_indices_[i].resize (window_size_);
    for (size_t j = 0; j < window_size_; ++j)
      data_argsort_indices_[i][j] = j;
  }

  data_invalid_count_.resize (size_, window_size_);
}

pcl::io::depth_sense::MedianBuffer::~MedianBuffer ()
{
  boost::mutex::scoped_lock lock (data_mutex_);
  for (size_t i = 0; i < window_size_; ++i)
    delete[] data_[i];
}

float
pcl::io::depth_sense::MedianBuffer::operator[] (size_t idx) const
{
  assert (idx < size_);
  int midpoint = (window_size_ - data_invalid_count_[idx]) / 2;
  return (data_[data_argsort_indices_[idx][midpoint]][idx]);
}

void
pcl::io::depth_sense::MedianBuffer::push (const float* data)
{
  assert ((sizeof (data) / sizeof (float)) == size_);
  boost::mutex::scoped_lock lock (data_mutex_);

  if (++data_current_idx_ >= window_size_)
    data_current_idx_ = 0;

  // New data will replace the column with index data_current_idx_. Before
  // overwriting it, we go through all the new-old value pairs and update
  // data_argsort_indices_ to maintain sorted order.
  for (size_t i = 0; i < size_; ++i)
  {
    const float& new_value = data[i];
    const float& old_value = data_[data_current_idx_][i];
    bool new_is_nan = pcl_isnan (new_value);
    bool old_is_nan = pcl_isnan (old_value);
    if (compare (new_value, old_value) == 0)
      continue;
    std::vector<unsigned char>& argsort_indices = data_argsort_indices_[i];
    // Rewrite the argsort indices before or after the position where we insert
    // depending on the relation between the old and new values
    if (compare (new_value, old_value) == 1)
    {
      for (int j = 0; j < window_size_; ++j)
        if (argsort_indices[j] == data_current_idx_)
        {
          int k = j + 1;
          while (k < window_size_ && compare (new_value, data_[argsort_indices[k]][i]) == 1)
          {
            std::swap (argsort_indices[k - 1], argsort_indices[k]);
            ++k;
          }
          break;
        }
    }
    else
    {
      for (int j = window_size_ - 1; j >= 0; --j)
        if (argsort_indices[j] == data_current_idx_)
        {
          int k = j - 1;
          while (k >= 0 && compare (new_value, data_[argsort_indices[k]][i]) == -1)
          {
            std::swap (argsort_indices[k], argsort_indices[k + 1]);
            --k;
          }
          break;
        }
    }

    if (new_is_nan && !old_is_nan)
      ++data_invalid_count_[i];
    else if (!new_is_nan && old_is_nan)
      --data_invalid_count_[i];
  }

  // Finally overwrite the data
  delete[] data_[data_current_idx_];
  data_[data_current_idx_] = data;
}

int pcl::io::depth_sense::MedianBuffer::compare (float a, float b)
{
  bool a_is_nan = pcl_isnan (a);
  bool b_is_nan = pcl_isnan (b);
  if (a_is_nan && b_is_nan)
    return 0;
  if (a_is_nan)
    return 1;
  if (b_is_nan)
    return -1;
  if (a == b)
    return 0;
  return a > b ? 1 : -1;
}

pcl::io::depth_sense::AverageBuffer::AverageBuffer (size_t size,
                                                    size_t window_size)
: Buffer (size)
, window_size_ (window_size)
, data_current_idx_ (window_size_ - 1)
{
  assert (size_ > 0);
  assert (window_size_ > 0 &&
          window_size_ <= std::numeric_limits<unsigned char>::max ());

  for (size_t i = 0; i < window_size_; ++i)
    data_.push_back (allocateAndFillWithNaNs (size_));

  data_sum_.resize (size_, 0);
  data_invalid_count_.resize (size_, window_size_);
}

pcl::io::depth_sense::AverageBuffer::~AverageBuffer ()
{
  boost::mutex::scoped_lock lock (data_mutex_);
  for (size_t i = 0; i < window_size_; ++i)
    delete[] data_[i];
}

float
pcl::io::depth_sense::AverageBuffer::operator[] (size_t idx) const
{
  assert (idx < size_);
  if (data_invalid_count_[idx] == window_size_)
    return (invalid_value_);
  else
    return (data_sum_[idx] / (window_size_ - data_invalid_count_[idx]));
}

void
pcl::io::depth_sense::AverageBuffer::push (const float* data)
{
  assert ((sizeof (data) / sizeof (float)) == size_);
  boost::mutex::scoped_lock lock (data_mutex_);

  if (++data_current_idx_ >= window_size_)
    data_current_idx_ = 0;

  // New data will replace the column with index data_current_idx_. Before
  // overwriting it, we go through the old values and subtract them from the
  // data_sum_
  for (size_t i = 0; i < size_; ++i)
  {
    const float& new_value = data[i];
    const float& old_value = data_[data_current_idx_][i];
    bool new_is_nan = pcl_isnan (new_value);
    bool old_is_nan = pcl_isnan (old_value);

    if (!old_is_nan)
      data_sum_[i] -= old_value;
    if (!new_is_nan)
      data_sum_[i] += new_value;

    if (new_is_nan && !old_is_nan)
      ++data_invalid_count_[i];
    else if (!new_is_nan && old_is_nan)
      --data_invalid_count_[i];
  }

  // Finally overwrite the data
  delete[] data_[data_current_idx_];
  data_[data_current_idx_] = data;
}

