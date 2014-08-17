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

#include "depth_sense/buffers.h"

using namespace pcl::io::depth_sense;

TEST (BuffersTest, SingleBuffer)
{
  SingleBuffer sb;
  const float data[] = {5, 4, 3, 2, 1};
  sb.push (data);
  for (size_t i = 0; i < sizeof (data) / sizeof (float); ++i)
    EXPECT_EQ (data[i], sb[i]);
}

TEST (BuffersTest, MedianBufferWindow1)
{
  MedianBuffer mb (1, 1);
  const float data[] = {5, 4, 3, 2, 1};
  for (size_t i = 0; i < sizeof (data) / sizeof (float); ++i)
  {
    mb.push (data + i);
    EXPECT_EQ (data[i], mb[0]);
  }
}

TEST (BuffersTest, MedianBufferWindow2)
{
  {
    MedianBuffer mb (1, 2);
    const float data[] = {5, 4, 3, 2, 1};
    const float median[] = {5, 5, 4, 3, 2};
    for (size_t i = 0; i < sizeof (data) / sizeof (float); ++i)
    {
      mb.push (data + i);
      EXPECT_EQ (median[i], mb[0]);
    }
  }
  {
    MedianBuffer mb (1, 2);
    const float data[] = {3, 4, 1, 3, 4};
    const float median[] = {3, 4, 4, 3, 4};
    for (size_t i = 0; i < sizeof (data) / sizeof (float); ++i)
    {
      mb.push (data + i);
      EXPECT_EQ (median[i], mb[0]);
    }
  }
}

TEST (BuffersTest, MedianBufferWindow3)
{
  {
    MedianBuffer mb (1, 3);
    const float data[] = {5, 4, 3, 2, 1, 0, -1};
    const float median[] = {5, 5, 4, 3, 2, 1, 0};
    for (size_t i = 0; i < sizeof (data) / sizeof (float); ++i)
    {
      mb.push (data + i);
      EXPECT_EQ (median[i], mb[0]);
    }
  }
  {
    MedianBuffer mb (1, 3);
    const float data[] = {3, 4, 1, 3, 4, 0, -1};
    const float median[] = {3, 4, 3, 3, 3, 3, 0};
    for (size_t i = 0; i < sizeof (data) / sizeof (float); ++i)
    {
      mb.push (data + i);
      EXPECT_EQ (median[i], mb[0]);
    }
  }
}

int main (int argc, char **argv)
{
  testing::InitGoogleTest (&argc, argv);
  return RUN_ALL_TESTS ();
}

