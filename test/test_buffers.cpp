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

#include <gtest/gtest.h>

#include <cmath>

#include "depth_sense/buffers.h"

using namespace pcl::io::depth_sense;

template <typename Buffer> void
checkBuffer (Buffer& buffer, const float* data, const float* expected, size_t size)
{
  const float* dptr = data;
  const float* eptr = expected;
  for (size_t i = 0; i < size; ++i)
  {
    float* d = new float[size];
    memcpy (d, dptr, size * sizeof (float));
    buffer.push (d);
    for (size_t j = 0; j < buffer.size (); ++j)
      if (isnan (eptr[j]))
        EXPECT_TRUE (isnan (buffer[j]));
      else
        EXPECT_EQ (eptr[j], buffer[j]);
    dptr += buffer.size ();
    eptr += buffer.size ();
  }
}

TEST (BuffersTest, SingleBuffer)
{
  SingleBuffer sb (1);
  const float data[] = {5, 4, 3, 2, 1};
  checkBuffer (sb, data, data, sizeof (data) / sizeof (float));
}

TEST (BuffersTest, MedianBufferWindow1)
{
  MedianBuffer mb (1, 1);
  const float data[] = {5, 4, 3, 2, 1};
  checkBuffer (mb, data, data, sizeof (data) / sizeof (float));
}

TEST (BuffersTest, MedianBufferWindow2)
{
  {
    MedianBuffer mb (1, 2);
    const float data[] = {5, 4, 3, 2, 1};
    const float median[] = {5, 5, 4, 3, 2};
    checkBuffer (mb, data, median, sizeof (data) / sizeof (float));
  }
  {
    MedianBuffer mb (1, 2);
    const float data[] = {3, 4, 1, 3, 4};
    const float median[] = {3, 4, 4, 3, 4};
    checkBuffer (mb, data, median, sizeof (data) / sizeof (float));
  }
}

TEST (BuffersTest, MedianBufferWindow3)
{
  {
    MedianBuffer mb (1, 3);
    const float data[] = {5, 4, 3, 2, 1, 0, -1};
    const float median[] = {5, 5, 4, 3, 2, 1, 0};
    checkBuffer (mb, data, median, sizeof (data) / sizeof (float));
  }
  {
    MedianBuffer mb (1, 3);
    const float data[] = {3, 4, 1, 3, 4, 0, -1};
    const float median[] = {3, 4, 3, 3, 3, 3, 0};
    checkBuffer (mb, data, median, sizeof (data) / sizeof (float));
  }
  {
    MedianBuffer mb (1, 3);
    const float data[] = {-4, -1, 3, -4, 1, 3, 4, 0};
    const float median[] = {-4, -1, -1, -1, 1, 1, 3, 3};
    checkBuffer (mb, data, median, sizeof (data) / sizeof (float));
  }
}

TEST (BuffersTest, MedianBufferWindow4)
{
  {
    MedianBuffer mb (1, 4);
    const float data[] = {5, 4, 3, 2, 1, 0, -1};
    const float median[] = {5, 5, 4, 4, 3, 2, 1};
    checkBuffer (mb, data, median, sizeof (data) / sizeof (float));
  }
  {
    MedianBuffer mb (1, 4);
    const float data[] = {-4, -1, 3, -4, 1, 3, 4, 0};
    const float median[] = {-4, -1, -1, -1, 1, 3, 3, 3};
    checkBuffer (mb, data, median, sizeof (data) / sizeof (float));
  }
}

TEST (BuffersTest, MedianBufferPushNaN)
{
  const float nan = std::numeric_limits<float>::quiet_NaN ();
  MedianBuffer mb (1, 3);
  const float data[] = {5, 4, 3, nan, 1, nan, nan, nan, 9, 3, 1};
  const float median[] = {5, 5, 4, 4, 3, 1, 1, nan, 9, 9, 3};
  checkBuffer (mb, data, median, sizeof (data) / sizeof (float));
}

TEST (BuffersTest, MedianBufferSize3Window3)
{
  {
    MedianBuffer mb (3, 3);
    const float data[] = {3, 3, 3, 1, 1, 1, 0, 0, 0};
    const float median[] = {3, 3, 3, 3, 3, 3, 1, 1, 1};
    checkBuffer (mb, data, median, sizeof (data) / sizeof (float) / mb.size ());
  }
  {
    MedianBuffer mb (3, 3);
    const float data[] = {3, 2, 1, 1, 1, 1, 3, 2, 1, 1, 2, 3};
    const float median[] = {3, 2, 1, 3, 2, 1, 3, 2, 1, 1, 2, 1};
    checkBuffer (mb, data, median, sizeof (data) / sizeof (float) / mb.size ());
  }
}

TEST (BuffersTest, AverageBufferWindow1)
{
  AverageBuffer ab (1, 1);
  const float data[] = {5, 4, 3, 2, 1};
  checkBuffer (ab, data, data, sizeof (data) / sizeof (float));
}

TEST (BuffersTest, AverageBufferWindow2)
{
  {
    AverageBuffer ab (1, 2);
    const float data[] = {5, 4, 3, 2, 1};
    const float median[] = {5, 4.5, 3.5, 2.5, 1.5};
    checkBuffer (ab, data, median, sizeof (data) / sizeof (float));
  }
  {
    AverageBuffer ab (1, 2);
    const float data[] = {3, 4, 1, 3, 4};
    const float median[] = {3, 3.5, 2.5, 2, 3.5};
    checkBuffer (ab, data, median, sizeof (data) / sizeof (float));
  }
}

TEST (BuffersTest, AverageBufferWindow3)
{
  {
    AverageBuffer ab (1, 3);
    const float data[] = {5, 4, 3, 2, 1, 0, -1};
    const float median[] = {5, 4.5, 4, 3, 2, 1, 0};
    checkBuffer (ab, data, median, sizeof (data) / sizeof (float));
  }
  {
    AverageBuffer ab (1, 3);
    const float data[] = {3, 4, 2, 3, 4, -1, -3};
    const float median[] = {3, 3.5, 3, 3, 3, 2, 0};
    checkBuffer (ab, data, median, sizeof (data) / sizeof (float));
  }
}

TEST (BuffersTest, AverageBufferPushNaN)
{
  const float nan = std::numeric_limits<float>::quiet_NaN ();
  AverageBuffer ab (1, 3);
  const float data[] = {5, 4, 3, nan, 3, nan, nan, nan, 9, 3, 0};
  const float median[] = {5, 4.5, 4, 3.5, 3, 3, 3, nan, 9, 6, 4};
  checkBuffer (ab, data, median, sizeof (data) / sizeof (float));
}

int main (int argc, char **argv)
{
  testing::InitGoogleTest (&argc, argv);
  return RUN_ALL_TESTS ();
}

