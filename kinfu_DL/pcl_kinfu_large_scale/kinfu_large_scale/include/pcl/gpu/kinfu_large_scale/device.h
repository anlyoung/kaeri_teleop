/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef PCL_KINFU_DEVICE_H_
#define PCL_KINFU_DEVICE_H_

#include <pcl/gpu/containers/device_array.h>
#include <iostream> // used by operator << in Struct Intr
#include <pcl/gpu/kinfu_large_scale/tsdf_buffer.h>

//using namespace pcl::gpu;

namespace pcl
{
  namespace device
  {
    namespace kinfuLS
    {
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Types
      typedef unsigned short ushort;
      typedef DeviceArray2D<float> MapArr;
      typedef DeviceArray2D<ushort> DepthMap;
      typedef float4 PointType;

      //TSDF fixed point divisor (if old format is enabled)
      const int DIVISOR = 32767;     // SHRT_MAX;
      
      //RGB images resolution
      const float  HEIGHT = 480.0f;
      const float  WIDTH = 640.0f;

      //Should be multiple of 32
      enum { VOLUME_X = 512, VOLUME_Y = 512, VOLUME_Z = 512 };
      enum { BOX_X = 32, BOX_Y = 32, BOX_Z = 32 };
          
      //Temporary constant (until we make it automatic) that holds the Kinect's focal length
      const float FOCAL_LENGTH = 575.816f;
    
      const float VOLUME_SIZE = 3.0f; // physical size represented by the TSDF volume. In meters
      const float DISTANCE_THRESHOLD = 1.5f; // when the camera target point is farther than DISTANCE_THRESHOLD from the current cube's center, shifting occurs. In meters
      const int SNAPSHOT_RATE = 45; // every 45 frames an RGB snapshot will be saved. -et parameter is needed when calling Kinfu Large Scale in command line.


      /** \brief Camera intrinsics structure
        */ 
      struct Intr
      {
        float fx, fy, cx, cy;
        Intr () {}
        Intr (float fx_, float fy_, float cx_, float cy_) : fx (fx_), fy (fy_), cx (cx_), cy (cy_) {}

        Intr operator () (int level_index) const
        { 
          int div = 1 << level_index; 
          return (Intr (fx / div, fy / div, cx / div, cy / div));
        }
        
        friend inline std::ostream&
        operator << (std::ostream& os, const Intr& intr)
        {
          os << "([f = " << intr.fx << ", " << intr.fy << "] [cp = " << intr.cx << ", " << intr.cy << "])";
          return (os);
        }
      };

      /** \brief 3x3 Matrix for device code
        */ 
      struct Mat33
      {
        float3 data[3];
      };
    }
  }
}

#endif /* PCL_KINFU_DEVICE_H_ */
