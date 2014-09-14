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

#ifndef PCL_IO_DEPTH_SENSE_BUFFERS_H
#define PCL_IO_DEPTH_SENSE_BUFFERS_H

#include <vector>
#include <limits>
#include <cassert>

#include <boost/cstdint.hpp>

namespace pcl
{

  namespace io
  {

    namespace depth_sense
    {

      class Buffer
      {

        public:

          virtual
          ~Buffer ();

          virtual float
          operator[] (size_t idx) const = 0;

          virtual void
          push (const float* data) = 0;

          inline size_t
          size () const
          {
            return (size_);
          }

        protected:

          Buffer (size_t size);

          static const float*
          allocateAndFillWithNaNs (size_t size);

          const size_t size_;
          const float invalid_value_;

      };

      class SingleBuffer : public Buffer
      {

        public:

          SingleBuffer (size_t size);

          virtual
          ~SingleBuffer ();

          virtual float
          operator[] (size_t idx) const;

          virtual void
          push (const float* data);

        private:

          const float* data_;

      };

      class MedianBuffer : public Buffer
      {

        public:

          MedianBuffer (size_t size, size_t window_size);

          virtual
          ~MedianBuffer ();

          virtual float
          operator[] (size_t idx) const;

          virtual void
          push (const float* data);

        private:

          /** Compare two floating point numbers.
            *
            * NaN is assumed to be larger than any other number. If both values
            * are NaNs, they are assumed to be equal.
            *
            * \return -1 if \c a < \c b, 0 if \c a == \c b, 1 if \c a > \c b */
          static int compare (float a, float b);

          const size_t window_size_;
          const size_t midpoint_;

          /// Data pushed into the buffer (last window_size_ chunks), logically
          /// organized as a circular buffer
          std::vector<const float*> data_;

          /// Index of the last pushed data chunk in the data_ circular buffer
          size_t data_current_idx_;

          /// Indices that the argsort function would produce for data_ (with
          /// dimensions swapped)
          std::vector<std::vector<unsigned char> > data_argsort_indices_;

          /// Number of invalid values in the buffer
          std::vector<unsigned char> data_invalid_count_;

      };

      class AverageBuffer : public Buffer
      {

        public:

          AverageBuffer (size_t size, size_t window_size);

          virtual
          ~AverageBuffer ();

          virtual float
          operator[] (size_t idx) const;

          virtual void
          push (const float* data);

        private:

          const size_t window_size_;

          /// Data pushed into the buffer (last window_size_ chunks), logically
          /// organized as a circular buffer
          std::vector<const float*> data_;

          /// Index of the last pushed data chunk in the data_ circular buffer
          size_t data_current_idx_;

          /// Current sum of the buffer
          std::vector<float> data_sum_;

          /// Number of invalid values in the buffer
          std::vector<unsigned char> data_invalid_count_;

      };

    }

  }

}


#endif /* PCL_IO_DEPTH_SENSE_BUFFERS_H */

