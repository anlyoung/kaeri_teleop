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

#include "device.hpp"
//#include <boost/graph/buffer_concepts.hpp>

namespace pcl
{
  namespace device
  {
    namespace kinfuLS
    {
      template<typename T>
      __global__ void
      initializeVolume (int3 voxels_size,PtrStep<T> volume)
      {
        int x = threadIdx.x + blockIdx.x * blockDim.x;
        int y = threadIdx.y + blockIdx.y * blockDim.y;
        
        
        if (x < voxels_size.x && y < voxels_size.y)
        {
            T *pos = volume.ptr(y) + x;
            int z_step = voxels_size.y * volume.step / sizeof(*pos);

  #pragma unroll
            for(int z = 0; z < voxels_size.z; ++z, pos+=z_step)
              pack_tsdf (0.f, 0, *pos);
        }
      }

      template<typename T>
      __global__ void
      clearSphereKernel(PtrStep<T> volume,int3 shift,float3 center,float radius)
      {
        int x = threadIdx.x + blockIdx.x * blockDim.x;
        int y = threadIdx.y + blockIdx.y * blockDim.y;

        if (x < VOLUME_X && y < VOLUME_Y)
        {
            int ax = x + shift.x;
            if (ax >= VOLUME_X)
                ax -= VOLUME_X;
            int ay = y + shift.y;
            if (ay >= VOLUME_Y)
                ay -= VOLUME_Y;

            T *pos = volume.ptr(ay) + ax;
            int z_step = VOLUME_Y * volume.step / sizeof(*pos);

  #pragma unroll
            for(int z = 0; z < VOLUME_Z; ++z)
            {
              int az = z + shift.z;
              if (az >= VOLUME_Z)
                az -= VOLUME_Z;

              float3 pt;
              pt.x = float(x);
              pt.y = float(y);
              pt.z = float(z);

              if (norm(pt - center) < radius)
                pack_tsdf(0.f, 0, *(pos + (az * z_step)));
            }
        }
      }

      template<typename T>
      __global__ void
      clearBBoxKernel(PtrStep<T> volume,int3 shift,float3 m,float3 M)
      {
        int x = threadIdx.x + blockIdx.x * blockDim.x;
        int y = threadIdx.y + blockIdx.y * blockDim.y;

        if (x < VOLUME_X && y < VOLUME_Y)
        {
            int ax = x + shift.x;
            if (ax >= VOLUME_X)
                ax -= VOLUME_X;
            int ay = y + shift.y;
            if (ay >= VOLUME_Y)
                ay -= VOLUME_Y;

            T *pos = volume.ptr(ay) + ax;
            int z_step = VOLUME_Y * volume.step / sizeof(*pos);

  #pragma unroll
            for(int z = 0; z < VOLUME_Z; ++z)
            {
              int az = z + shift.z;
              if (az >= VOLUME_Z)
                az -= VOLUME_Z;

              float3 pt;
              pt.x = float(x);
              pt.y = float(y);
              pt.z = float(z);

              if ((pt.x >= m.x) && (pt.y >= m.y) && (pt.z >= m.z) &&
                (pt.x < M.x) && (pt.y < M.y) && (pt.z < M.z))
                pack_tsdf(0.f, 0, *(pos + (az * z_step)));
            }
        }
      }

			//************** added by DH **************//
			template<typename T>
			__global__ void
			extractBoxKernel(PtrStep<T> src, PtrStep<T> dst, int3 origin, int3 c)
			{
				int x = threadIdx.x;
				int y = threadIdx.y;
				int z = blockIdx.x;
				
				int MIN_X, MAX_X, MIN_Y, MAX_Y, MIN_Z, MAX_Z;
				if (BOX_X%2==0) {
					MIN_X = -BOX_X/2;
					MAX_X = BOX_X/2-1;
				}
				else {
					MIN_X = -int(BOX_X/2.0);
					MAX_X = int(BOX_X/2.0);
				} 

				if (BOX_Y%2==0) {
					MIN_Y = -BOX_Y/2;
					MAX_Y = BOX_Y/2-1;
				}
				else {
					MIN_Y = -int(BOX_Y/2.0);
					MAX_Y = int(BOX_Y/2.0);
				} 

				if (BOX_Z%2==0) {
					MIN_Z = -BOX_Z/2;
					MAX_Z = BOX_Z/2-1;
				}
				else {
					MIN_Z = -int(BOX_Z/2.0);
					MAX_Z = int(BOX_Z/2.0);
				} 

				if (c.x+MAX_X < VOLUME_X && 0 < c.x+MIN_X && c.y+MAX_Y < VOLUME_Y && 0 < c.y+MIN_Y && c.z+MAX_Z < VOLUME_Z && 0 < c.z+MIN_Z){
					int ax = c.x + x + MIN_X + origin.x;
					if (ax >= VOLUME_X) ax -= VOLUME_X;
				  int ay = c.y + y + MIN_Y + origin.y;
					if (ay >= VOLUME_Y) ay -= VOLUME_Y;
					int *src_pos = src.ptr(ay) + ax;
          int *dst_pos = dst.ptr(y) + x;
					int z_step_src = VOLUME_Y * src.step / sizeof(*src_pos);
					int z_step_dst = BOX_Y * dst.step / sizeof(*dst_pos);
					int az = c.z + z + MIN_Z + origin.z;
					if (az >= VOLUME_Z) az -= VOLUME_Z;
					*(dst_pos+z*z_step_dst) = *(src_pos+az*z_step_src);
					//cudaMemcpy(dst_pos+z+5*z_step_dst,src_pos+az*z_step_src,sizeof(src_pos),cudaMemcpyDeviceToDevice);
				}
			}

			template<typename T>
			__global__ void
			copyBoxKernel(PtrStep<T> src, PtrStep<T> dst)
			{
				int x = threadIdx.x;
				int y = threadIdx.y;
				int z = blockIdx.x;
				
				int *src_pos = src.ptr(y) + x;
        int *dst_pos = dst.ptr(y) + x;
				int z_step_src = BOX_Y * src.step / sizeof(*src_pos);
				int z_step_dst = BOX_Y * dst.step / sizeof(*dst_pos);
				*(dst_pos+z*z_step_dst) = *(src_pos+z*z_step_src);
			}
			//************** added by DH **************//

      template<typename T>
      __global__ void
      clearSliceKernel (PtrStep<T> volume, pcl::gpu::kinfuLS::tsdf_buffer buffer, int3 minBounds, int3 maxBounds)
      {
        int x = threadIdx.x + blockIdx.x * blockDim.x;
        int y = threadIdx.y + blockIdx.y * blockDim.y;
            
        //compute relative indices
        int idX, idY;
        
        if(x <= minBounds.x)
          idX = x + buffer.voxels_size.x;
        else
          idX = x;
        
        if(y <= minBounds.y)
          idY = y + buffer.voxels_size.y;
        else
          idY = y;	 
                
        
        if ( x < buffer.voxels_size.x && y < buffer.voxels_size.y)
        {
            if( (idX >= minBounds.x && idX <= maxBounds.x) || (idY >= minBounds.y && idY <= maxBounds.y) )
            {
                // BLACK ZONE => clear on all Z values
          
                ///Pointer to the first x,y,0			
                T *pos = volume.ptr(y) + x;
                
                ///Get the step on Z
                int z_step = buffer.voxels_size.y * volume.step / sizeof(*pos);
                                    
                ///Get the size of the whole TSDF memory
                int size = buffer.tsdf_memory_end - buffer.tsdf_memory_start + 1;
                                  
                ///Move along z axis
      #pragma unroll
                for(int z = 0; z < buffer.voxels_size.z; ++z, pos+=z_step)
                {
                  ///If we went outside of the memory, make sure we go back to the begining of it
                  if(pos > buffer.tsdf_memory_end)
                    pos = pos - size;
                  
                  if (pos >= buffer.tsdf_memory_start && pos <= buffer.tsdf_memory_end) // quickfix for http://dev.pointclouds.org/issues/894
                    pack_tsdf (0.f, 0, *pos);
                }
            }
            else /* if( idX > maxBounds.x && idY > maxBounds.y)*/
            {
              
                ///RED ZONE  => clear only appropriate Z
              
                ///Pointer to the first x,y,0
                T *pos = volume.ptr(y) + x;
                
                ///Get the step on Z
                int z_step = buffer.voxels_size.y * volume.step / sizeof(*pos);
                            
                ///Get the size of the whole TSDF memory 
                int size = buffer.tsdf_memory_end - buffer.tsdf_memory_start + 1;
                              
                ///Move pointer to the Z origin
                pos+= minBounds.z * z_step;
                
                ///If the Z offset is negative, we move the pointer back
                if(maxBounds.z < 0)
                  pos += maxBounds.z * z_step;
                  
                ///We make sure that we are not already before the start of the memory
                if(pos < buffer.tsdf_memory_start)
                    pos = pos + size;

                int nbSteps = abs(maxBounds.z);
                
            #pragma unroll				
                for(int z = 0; z < nbSteps; ++z, pos+=z_step)
                {
                  ///If we went outside of the memory, make sure we go back to the begining of it
                  if(pos > buffer.tsdf_memory_end)
                    pos = pos - size;
                  
                  if (pos >= buffer.tsdf_memory_start && pos <= buffer.tsdf_memory_end) // quickfix for http://dev.pointclouds.org/issues/894
                    pack_tsdf (0.f, 0, *pos);
                }
            } //else /* if( idX > maxBounds.x && idY > maxBounds.y)*/
        } // if ( x < VOLUME_X && y < VOLUME_Y)
      } // clearSliceKernel
   
      void
      initVolume (int3 voxels_size,PtrStep<short2> volume)
      {
        dim3 block (16, 16);
        dim3 grid (1, 1, 1);
        grid.x = divUp (voxels_size.x, block.x);
        grid.y = divUp (voxels_size.y, block.y);

        initializeVolume<<<grid, block>>>(voxels_size,volume);
        cudaSafeCall ( cudaGetLastError () );
        cudaSafeCall (cudaDeviceSynchronize ());
      }

      void
      clearSphere(PtrStep<short2> volume,int3 tsdf_origin,float3 center,float radius)
      {
        dim3 block (32, 16);
        dim3 grid (1, 1, 1);
        grid.x = divUp (VOLUME_X, block.x);
        grid.y = divUp (VOLUME_Y, block.y);

        clearSphereKernel<<<grid, block>>>(volume,tsdf_origin,center,radius);
        cudaSafeCall ( cudaGetLastError () );
        cudaSafeCall (cudaDeviceSynchronize ());
      }

      void
      clearBBox(PtrStep<short2> volume,const int3& origin,const float3& m,const float3& M)
      {
        dim3 block (32, 16);
        dim3 grid (1, 1, 1);
        grid.x = divUp (VOLUME_X, block.x);
        grid.y = divUp (VOLUME_Y, block.y);

        clearBBoxKernel<<<grid, block>>>(volume,origin,m,M);
        cudaSafeCall ( cudaGetLastError () );
        cudaSafeCall (cudaDeviceSynchronize ());
      }
			//************** added by DH **************//
			void
      extractBox(PtrStep<int> src, PtrStep<int> dst,const int3& origin,const int3& c)
      {
				
				dim3 block (BOX_X, BOX_Y, 1);
				dim3 grid (BOX_Z, 1, 1);

				extractBoxKernel<<<grid, block>>>(src, dst, origin, c);
			  cudaSafeCall ( cudaGetLastError () );
				cudaSafeCall ( cudaDeviceSynchronize () );
				/*
				int MIN_X, MAX_X, MIN_Y, MAX_Y, MIN_Z, MAX_Z;
				if (BOX_X%2==0) {
					MIN_X = -BOX_X/2;
					MAX_X = BOX_X/2-1;
				}
				else {
					MIN_X = -int(BOX_X/2.0);
					MAX_X = int(BOX_X/2.0);
				} 

				if (BOX_Y%2==0) {
					MIN_Y = -BOX_Y/2;
					MAX_Y = BOX_Y/2-1;
				}
				else {
					MIN_Y = -int(BOX_Y/2.0);
					MAX_Y = int(BOX_Y/2.0);
				} 

				if (BOX_Z%2==0) {
					MIN_Z = -BOX_Z/2;
					MAX_Z = BOX_Z/2-1;
				}
				else {
					MIN_Z = -int(BOX_Z/2.0);
					MAX_Z = int(BOX_Z/2.0);
				} 			
				
				#pragma unroll
				if (c.x+MAX_X < VOLUME_X && 0 < c.x+MIN_X && c.y+MAX_Y < VOLUME_Y && 0 < c.y+MIN_Y && c.z+MAX_Z < VOLUME_Z && 0 < c.z+MIN_Z){
					for (int x = MIN_X; x < MAX_X; x++){
						int ax = c.x + x + origin.x;
						if (ax >= VOLUME_X) ax -= VOLUME_X;
						for (int y = MIN_Y; y < MAX_Y; y++){
							int ay = c.y + y + origin.y;
							if (ay >= VOLUME_Y) ay -= VOLUME_Y;
							int *src_pos = src.ptr(ay) + ax;
			        int *dst_pos = dst.ptr(y-MIN_Y) + x-MIN_X;
							int z_step_src = VOLUME_Y * src.step / sizeof(*src_pos);
							int z_step_dst = BOX_Y * dst.step / sizeof(*dst_pos);
							for (int z = MIN_Z; z < MAX_Z; z++){
								int az = c.z + z + origin.z;
								if (az >= VOLUME_Z) az -= VOLUME_Z;
								cudaMemcpy(dst_pos+(z-MIN_Z)*z_step_dst,src_pos+az*z_step_src,sizeof(src_pos),cudaMemcpyDeviceToDevice);
							}
						}
					}
			  cudaSafeCall ( cudaGetLastError() );
				}
				*/
      }

			void
      copyBox(PtrStep<int> src, PtrStep<int> dst)
      {
				
				dim3 block (BOX_X, BOX_Y, 1);
				dim3 grid (BOX_Z, 1, 1);

				copyBoxKernel<<<grid, block>>>(src, dst);
			  cudaSafeCall ( cudaGetLastError () );
				cudaSafeCall ( cudaDeviceSynchronize () );
      }
			//************** added by DH **************//
    }
  }
}


namespace pcl
{
  namespace device
  {
    namespace kinfuLS
    {
      struct Tsdf
      {
        enum
        {
          CTA_SIZE_X = 32, CTA_SIZE_Y = 8,
          MAX_WEIGHT = 1 << 7
        };

        mutable PtrStep<short2> volume;
        float3 cell_size;

        Intr intr;

        Mat33 Rcurr_inv;
        float3 tcurr;

        PtrStepSz<ushort> depth_raw; //depth in mm

        float tranc_dist_mm;

        __device__ __forceinline__ float3
        getVoxelGCoo (int x, int y, int z) const
        {
          float3 coo = make_float3 (x, y, z);
          coo += 0.5f;         //shift to cell center;

          coo.x *= cell_size.x;
          coo.y *= cell_size.y;
          coo.z *= cell_size.z;

          return coo;
        }

        __device__ __forceinline__ void
        operator () () const
        {
          int x = threadIdx.x + blockIdx.x * CTA_SIZE_X;
          int y = threadIdx.y + blockIdx.y * CTA_SIZE_Y;

          if (x >= VOLUME_X || y >= VOLUME_Y)
            return;

          short2 *pos = volume.ptr (y) + x;
          int elem_step = volume.step * VOLUME_Y / sizeof(*pos);

          for (int z = 0; z < VOLUME_Z; ++z, pos += elem_step)
          {
            float3 v_g = getVoxelGCoo (x, y, z);            //3 // p

            //tranform to curr cam coo space
            float3 v = Rcurr_inv * (v_g - tcurr);           //4

            int2 coo;           //project to current cam
            coo.x = __float2int_rn (v.x * intr.fx / v.z + intr.cx);
            coo.y = __float2int_rn (v.y * intr.fy / v.z + intr.cy);

            if (v.z > 0 && coo.x >= 0 && coo.y >= 0 && coo.x < depth_raw.cols && coo.y < depth_raw.rows)           //6
            {
              int Dp = depth_raw.ptr (coo.y)[coo.x];

              if (Dp != 0)
              {
                float xl = (coo.x - intr.cx) / intr.fx;
                float yl = (coo.y - intr.cy) / intr.fy;
                float lambda_inv = rsqrtf (xl * xl + yl * yl + 1);

                float sdf = 1000 * norm (tcurr - v_g) * lambda_inv - Dp; //mm

                sdf *= (-1);

                if (sdf >= -tranc_dist_mm)
                {
                  float tsdf = fmin (1.f, sdf / tranc_dist_mm);

                  int weight_prev;
                  float tsdf_prev;

                  //read and unpack
                  unpack_tsdf (*pos, tsdf_prev, weight_prev);

                  const int Wrk = 1;

                  float tsdf_new = (tsdf_prev * weight_prev + Wrk * tsdf) / (weight_prev + Wrk);
                  int weight_new = min (weight_prev + Wrk, MAX_WEIGHT);

                  pack_tsdf (tsdf_new, weight_new, *pos);
                }
              }
            }
          }
        }
      };

      template<typename T>
      __global__ void
      uploadKnownToTSDFSliceKernel (PtrStep<T> volume, pcl::gpu::kinfuLS::tsdf_buffer buffer, int3 minBounds, int3 maxBounds,
        PtrStep<short> known_status)
      {
        int x = threadIdx.x + blockIdx.x * blockDim.x;
        int y = threadIdx.y + blockIdx.y * blockDim.y;

        //compute relative indices
        int idX, idY;

        if(x <= minBounds.x)
          idX = x + buffer.voxels_size.x;
        else
          idX = x;

        if(y <= minBounds.y)
          idY = y + buffer.voxels_size.y;
        else
          idY = y;


        if ( x < buffer.voxels_size.x && y < buffer.voxels_size.y)
        {
            if( (idX >= minBounds.x && idX <= maxBounds.x) || (idY >= minBounds.y && idY <= maxBounds.y) )
            {
                // BLACK ZONE => clear on all Z values

                ///Pointer to the first x,y,0
                T *pos = volume.ptr(y) + x;

                ///Get the step on Z
                int z_step = buffer.voxels_size.y * volume.step / sizeof(*pos);

                ///Get the size of the whole TSDF memory
                int size = buffer.tsdf_memory_end - buffer.tsdf_memory_start + 1;

                short * ks = known_status.ptr(y) + x;
                short * max_ks = known_status.ptr(0) + buffer.voxels_size.x*buffer.voxels_size.y*buffer.voxels_size.z;

                ///Move along z axis
      #pragma unroll
                for(int z = 0; z < buffer.voxels_size.z; ++z, pos+=z_step, ks+=z_step)
                {
                  ///If we went outside of the memory, make sure we go back to the begining of it
                  if(pos > buffer.tsdf_memory_end)
                    pos = pos - size;

                  if (ks >= max_ks)
                    ks -= size;

                  const short increment = *ks;

                  if (increment && pos >= buffer.tsdf_memory_start && pos <= buffer.tsdf_memory_end) {
                    float tsdf;
                    int w;
                    unpack_tsdf(*pos, tsdf, w);
                    if (w == 0)
                      tsdf = 1.0;
                    pack_tsdf (tsdf, min(increment + w,(Tsdf::MAX_WEIGHT)), *pos);
                  }
                }
            }
            else /* if( idX > maxBounds.x && idY > maxBounds.y)*/
            {

                ///RED ZONE  => clear only appropriate Z

                ///Pointer to the first x,y,0
                T *pos = volume.ptr(y) + x;

                ///Get the step on Z
                int z_step = buffer.voxels_size.y * volume.step / sizeof(*pos);

                ///Get the size of the whole TSDF memory
                int size = buffer.tsdf_memory_end - buffer.tsdf_memory_start + 1;

                short * ks = known_status.ptr(y) + x;
                short * max_ks = known_status.ptr(0) + buffer.voxels_size.x*buffer.voxels_size.y*buffer.voxels_size.z;

                ///Move pointer to the Z origin
                pos+= minBounds.z * z_step;
                ks+= minBounds.z * z_step;

                ///If the Z offset is negative, we move the pointer back
                if(maxBounds.z < 0) {
                  pos += maxBounds.z * z_step;
                  ks += minBounds.z * z_step;
                }

                ///We make sure that we are not already before the start of the memory
                if(pos < buffer.tsdf_memory_start) {
                    pos = pos + size;
                    ks += size;
                }

                int nbSteps = abs(maxBounds.z);

            #pragma unroll
                for(int z = 0; z < nbSteps; ++z, pos+=z_step, ks+=z_step)
                {
                  ///If we went outside of the memory, make sure we go back to the begining of it
                  if(pos > buffer.tsdf_memory_end)
                    pos = pos - size;

                  if (ks >= max_ks)
                    ks -= size;

                  const short increment = *ks;

                  if (increment && pos >= buffer.tsdf_memory_start && pos <= buffer.tsdf_memory_end) {
                    float tsdf;
                    int w;
                    unpack_tsdf(*pos, tsdf, w);
                    if (w == 0)
                      tsdf = 1.0;
                    pack_tsdf (tsdf, min(increment + w,(Tsdf::MAX_WEIGHT)), *pos);
                  }
                }
            } //else /* if( idX > maxBounds.x && idY > maxBounds.y)*/
        } // if ( x < VOLUME_X && y < VOLUME_Y)
      } // clearSliceKernel

      __global__ void
      integrateTsdfKernel (const Tsdf tsdf) {
        tsdf ();
      }

      __global__ void
      tsdf2 (PtrStep<short2> volume, const float tranc_dist_mm, const Mat33 Rcurr_inv, float3 tcurr,
            const Intr intr, const PtrStepSz<ushort> depth_raw, const float3 cell_size)
      {
        int x = threadIdx.x + blockIdx.x * blockDim.x;
        int y = threadIdx.y + blockIdx.y * blockDim.y;

        if (x >= VOLUME_X || y >= VOLUME_Y)
          return;

        short2 *pos = volume.ptr (y) + x;
        int elem_step = volume.step * VOLUME_Y / sizeof(short2);

        float v_g_x = (x + 0.5f) * cell_size.x - tcurr.x;
        float v_g_y = (y + 0.5f) * cell_size.y - tcurr.y;
        float v_g_z = (0 + 0.5f) * cell_size.z - tcurr.z;

        float v_x = Rcurr_inv.data[0].x * v_g_x + Rcurr_inv.data[0].y * v_g_y + Rcurr_inv.data[0].z * v_g_z;
        float v_y = Rcurr_inv.data[1].x * v_g_x + Rcurr_inv.data[1].y * v_g_y + Rcurr_inv.data[1].z * v_g_z;
        float v_z = Rcurr_inv.data[2].x * v_g_x + Rcurr_inv.data[2].y * v_g_y + Rcurr_inv.data[2].z * v_g_z;

  //#pragma unroll
        for (int z = 0; z < VOLUME_Z; ++z)
        {
          float3 vr;
          vr.x = v_g_x;
          vr.y = v_g_y;
          vr.z = (v_g_z + z * cell_size.z);

          float3 v;
          v.x = v_x + Rcurr_inv.data[0].z * z * cell_size.z;
          v.y = v_y + Rcurr_inv.data[1].z * z * cell_size.z;
          v.z = v_z + Rcurr_inv.data[2].z * z * cell_size.z;

          int2 coo;         //project to current cam
          coo.x = __float2int_rn (v.x * intr.fx / v.z + intr.cx);
          coo.y = __float2int_rn (v.y * intr.fy / v.z + intr.cy);


          if (v.z > 0 && coo.x >= 0 && coo.y >= 0 && coo.x < depth_raw.cols && coo.y < depth_raw.rows)         //6
          {
            int Dp = depth_raw.ptr (coo.y)[coo.x]; //mm

            if (Dp != 0)
            {
              float xl = (coo.x - intr.cx) / intr.fx;
              float yl = (coo.y - intr.cy) / intr.fy;
              float lambda_inv = rsqrtf (xl * xl + yl * yl + 1);

              float sdf = Dp - norm (vr) * lambda_inv * 1000; //mm


              if (sdf >= -tranc_dist_mm)
              {
                float tsdf = fmin (1.f, sdf / tranc_dist_mm);

                int weight_prev;
                float tsdf_prev;

                //read and unpack
                unpack_tsdf (*pos, tsdf_prev, weight_prev);

                const int Wrk = 1;

                float tsdf_new = (tsdf_prev * weight_prev + Wrk * tsdf) / (weight_prev + Wrk);
                int weight_new = min (weight_prev + Wrk, Tsdf::MAX_WEIGHT);

                pack_tsdf (tsdf_new, weight_new, *pos);
              }
            }
          }
          pos += elem_step;
        }       /* for(int z = 0; z < VOLUME_Z; ++z) */
      }      /* __global__ */

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void
      integrateTsdfVolume (const PtrStepSz<ushort>& depth_raw, const Intr& intr, const float3& volume_size,
                                        const Mat33& Rcurr_inv, const float3& tcurr, float tranc_dist, 
                                        PtrStep<short2> volume)
      {
        Tsdf tsdf;

        tsdf.volume = volume;  
        tsdf.cell_size.x = volume_size.x / VOLUME_X;
        tsdf.cell_size.y = volume_size.y / VOLUME_Y;
        tsdf.cell_size.z = volume_size.z / VOLUME_Z;
        
        tsdf.intr = intr;

        tsdf.Rcurr_inv = Rcurr_inv;
        tsdf.tcurr = tcurr;
        tsdf.depth_raw = depth_raw;

        tsdf.tranc_dist_mm = tranc_dist*1000; //mm

        dim3 block (Tsdf::CTA_SIZE_X, Tsdf::CTA_SIZE_Y);
        dim3 grid (divUp (VOLUME_X, block.x), divUp (VOLUME_Y, block.y));

      #if 0
        //tsdf2<<<grid, block>>>(volume, tranc_dist, Rcurr_inv, tcurr, intr, depth_raw, tsdf.cell_size);
        integrateTsdfKernel<<<grid, block>>>(tsdf);
      #endif
        cudaSafeCall ( cudaGetLastError () );
        cudaSafeCall (cudaDeviceSynchronize ());
      }
    }
  }
}

namespace pcl
{
  namespace device
  {
    namespace kinfuLS
    {
      __global__ void
      scaleDepth (const PtrStepSz<ushort> depth, PtrStep<float> scaled, const Intr intr)
      {
        int x = threadIdx.x + blockIdx.x * blockDim.x;
        int y = threadIdx.y + blockIdx.y * blockDim.y;

        if (x >= depth.cols || y >= depth.rows)
          return;

        int Dp = depth.ptr (y)[x];

        float xl = (x - intr.cx) / intr.fx;
        float yl = (y - intr.cy) / intr.fy;
        float lambda = sqrtf (xl * xl + yl * yl + 1);

        scaled.ptr (y)[x] = Dp * lambda/1000.f; //meters
      }

      __global__ void
      tsdf23 (const PtrStepSz<float> depthScaled, PtrStep<short2> volume,
              const float tranc_dist, const Mat33 Rcurr_inv, const float3 tcurr, const Intr intr, const float3 cell_size, const pcl::gpu::kinfuLS::tsdf_buffer buffer)
      {
        int x = threadIdx.x + blockIdx.x * blockDim.x;
        int y = threadIdx.y + blockIdx.y * blockDim.y;

        if (x >= buffer.voxels_size.x || y >= buffer.voxels_size.y)
          return;

        float v_g_x = (x + 0.5f) * cell_size.x - tcurr.x;
        float v_g_y = (y + 0.5f) * cell_size.y - tcurr.y;
        float v_g_z = (0 + 0.5f) * cell_size.z - tcurr.z;

        float v_g_part_norm = v_g_x * v_g_x + v_g_y * v_g_y;

        float v_x = (Rcurr_inv.data[0].x * v_g_x + Rcurr_inv.data[0].y * v_g_y + Rcurr_inv.data[0].z * v_g_z) * intr.fx;
        float v_y = (Rcurr_inv.data[1].x * v_g_x + Rcurr_inv.data[1].y * v_g_y + Rcurr_inv.data[1].z * v_g_z) * intr.fy;
        float v_z = (Rcurr_inv.data[2].x * v_g_x + Rcurr_inv.data[2].y * v_g_y + Rcurr_inv.data[2].z * v_g_z);

        float z_scaled = 0;

        float Rcurr_inv_0_z_scaled = Rcurr_inv.data[0].z * cell_size.z * intr.fx;
        float Rcurr_inv_1_z_scaled = Rcurr_inv.data[1].z * cell_size.z * intr.fy;

        float tranc_dist_inv = 1.0f / tranc_dist;

        short2* pos = volume.ptr (y) + x;
        
        // shift the pointer to relative indices
        shift_tsdf_pointer(&pos, buffer);
        
        int elem_step = volume.step * buffer.voxels_size.y / sizeof(short2);

  //#pragma unroll
        for (int z = 0; z < buffer.voxels_size.z;
            ++z,
            v_g_z += cell_size.z,
            z_scaled += cell_size.z,
            v_x += Rcurr_inv_0_z_scaled,
            v_y += Rcurr_inv_1_z_scaled,
            pos += elem_step)
        {
          
          // As the pointer is incremented in the for loop, we have to make sure that the pointer is never outside the memory
          if(pos > buffer.tsdf_memory_end)
            pos -= (buffer.tsdf_memory_end - buffer.tsdf_memory_start + 1);
          
          float inv_z = 1.0f / (v_z + Rcurr_inv.data[2].z * z_scaled);
          if (inv_z < 0)
              continue;

          // project to current cam
          int2 coo =
          {
            __float2int_rn (v_x * inv_z + intr.cx),
            __float2int_rn (v_y * inv_z + intr.cy)
          };

          if (coo.x >= 0 && coo.y >= 0 && coo.x < depthScaled.cols && coo.y < depthScaled.rows)         //6
          {
            float Dp_scaled = depthScaled.ptr (coo.y)[coo.x]; //meters

            float sdf = Dp_scaled - sqrtf (v_g_z * v_g_z + v_g_part_norm);

            if (Dp_scaled != 0 && sdf >= -tranc_dist) //meters
            {
              float tsdf = fmin (1.0f, sdf * tranc_dist_inv);

              //read and unpack
              float tsdf_prev;
              int weight_prev;
              unpack_tsdf (*pos, tsdf_prev, weight_prev);

              const int Wrk = 1;

              float tsdf_new = (tsdf_prev * weight_prev + Wrk * tsdf) / (weight_prev + Wrk);
              int weight_new = min (weight_prev + Wrk, Tsdf::MAX_WEIGHT);

              pack_tsdf (tsdf_new, weight_new, *pos);
            }
          }
        }       // for(int z = 0; z < VOLUME_Z; ++z)
      }      // __global__

      __global__ void
      tsdf23normal_hack (const PtrStepSz<float> depthScaled, PtrStep<short2> volume,
                    const float tranc_dist, const Mat33 Rcurr_inv, const float3 tcurr, const Intr intr, const float3 cell_size)
      {
          int x = threadIdx.x + blockIdx.x * blockDim.x;
          int y = threadIdx.y + blockIdx.y * blockDim.y;

          if (x >= VOLUME_X || y >= VOLUME_Y)
              return;

          const float v_g_x = (x + 0.5f) * cell_size.x - tcurr.x;
          const float v_g_y = (y + 0.5f) * cell_size.y - tcurr.y;
          float v_g_z = (0 + 0.5f) * cell_size.z - tcurr.z;

          float v_g_part_norm = v_g_x * v_g_x + v_g_y * v_g_y;

          float v_x = (Rcurr_inv.data[0].x * v_g_x + Rcurr_inv.data[0].y * v_g_y + Rcurr_inv.data[0].z * v_g_z) * intr.fx;
          float v_y = (Rcurr_inv.data[1].x * v_g_x + Rcurr_inv.data[1].y * v_g_y + Rcurr_inv.data[1].z * v_g_z) * intr.fy;
          float v_z = (Rcurr_inv.data[2].x * v_g_x + Rcurr_inv.data[2].y * v_g_y + Rcurr_inv.data[2].z * v_g_z);

          float z_scaled = 0;

          float Rcurr_inv_0_z_scaled = Rcurr_inv.data[0].z * cell_size.z * intr.fx;
          float Rcurr_inv_1_z_scaled = Rcurr_inv.data[1].z * cell_size.z * intr.fy;

          float tranc_dist_inv = 1.0f / tranc_dist;

          short2* pos = volume.ptr (y) + x;
          int elem_step = volume.step * VOLUME_Y / sizeof(short2);

          //#pragma unroll
          for (int z = 0; z < VOLUME_Z;
              ++z,
              v_g_z += cell_size.z,
              z_scaled += cell_size.z,
              v_x += Rcurr_inv_0_z_scaled,
              v_y += Rcurr_inv_1_z_scaled,
              pos += elem_step)
          {
              float inv_z = 1.0f / (v_z + Rcurr_inv.data[2].z * z_scaled);
              if (inv_z < 0)
                  continue;

              // project to current cam
              int2 coo =
              {
                  __float2int_rn (v_x * inv_z + intr.cx),
                  __float2int_rn (v_y * inv_z + intr.cy)
              };

              if (coo.x >= 0 && coo.y >= 0 && coo.x < depthScaled.cols && coo.y < depthScaled.rows)         //6
              {
                  float Dp_scaled = depthScaled.ptr (coo.y)[coo.x]; //meters

                  float sdf = Dp_scaled - sqrtf (v_g_z * v_g_z + v_g_part_norm);

                  if (Dp_scaled != 0 && sdf >= -tranc_dist) //meters
                  {
                      float tsdf = fmin (1.0f, sdf * tranc_dist_inv);                                              

                      bool integrate = true;
                      if ((x > 0 &&  x < VOLUME_X-2) && (y > 0 && y < VOLUME_Y-2) && (z > 0 && z < VOLUME_Z-2))
                      {
                          const float qnan = numeric_limits<float>::quiet_NaN();
                          float3 normal = make_float3(qnan, qnan, qnan);

                          float Fn, Fp;
                          int Wn = 0, Wp = 0;
                          unpack_tsdf (*(pos + elem_step), Fn, Wn);
                          unpack_tsdf (*(pos - elem_step), Fp, Wp);

                          if (Wn > 16 && Wp > 16) 
                              normal.z = (Fn - Fp)/cell_size.z;

                          unpack_tsdf (*(pos + volume.step/sizeof(short2) ), Fn, Wn);
                          unpack_tsdf (*(pos - volume.step/sizeof(short2) ), Fp, Wp);

                          if (Wn > 16 && Wp > 16) 
                              normal.y = (Fn - Fp)/cell_size.y;

                          unpack_tsdf (*(pos + 1), Fn, Wn);
                          unpack_tsdf (*(pos - 1), Fp, Wp);

                          if (Wn > 16 && Wp > 16) 
                              normal.x = (Fn - Fp)/cell_size.x;

                          if (normal.x != qnan && normal.y != qnan && normal.z != qnan)
                          {
                              float norm2 = dot(normal, normal);
                              if (norm2 >= 1e-10)
                              {
                                  normal *= rsqrt(norm2);

                                  float nt = v_g_x * normal.x + v_g_y * normal.y + v_g_z * normal.z;
                                  float cosine = nt * rsqrt(v_g_x * v_g_x + v_g_y * v_g_y + v_g_z * v_g_z);

                                  if (cosine < 0.5)
                                      integrate = false;
                              }
                          }
                      }

                      if (integrate)
                      {
                          //read and unpack
                          float tsdf_prev;
                          int weight_prev;
                          unpack_tsdf (*pos, tsdf_prev, weight_prev);

                          const int Wrk = 1;

                          float tsdf_new = (tsdf_prev * weight_prev + Wrk * tsdf) / (weight_prev + Wrk);
                          int weight_new = min (weight_prev + Wrk, Tsdf::MAX_WEIGHT);

                          pack_tsdf (tsdf_new, weight_new, *pos);
                      }
                  }
              }
          }       // for(int z = 0; z < VOLUME_Z; ++z)
      }      // __global__

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void
      integrateTsdfVolume (const PtrStepSz<ushort>& depth, const Intr& intr,
                                        const float3& volume_size, const Mat33& Rcurr_inv, const float3& tcurr, 
                                        float tranc_dist,
                                        PtrStep<short2> volume, const pcl::gpu::kinfuLS::tsdf_buffer* buffer, DeviceArray2D<float>& depthScaled)
      {
        depthScaled.create (depth.rows, depth.cols);

        dim3 block_scale (32, 8);
        dim3 grid_scale (divUp (depth.cols, block_scale.x), divUp (depth.rows, block_scale.y));

        //scales depth along ray and converts mm -> meters. 
        scaleDepth<<<grid_scale, block_scale>>>(depth, depthScaled, intr);
        cudaSafeCall ( cudaGetLastError () );

        float3 cell_size;
        cell_size.x = volume_size.x / buffer->voxels_size.x;
        cell_size.y = volume_size.y / buffer->voxels_size.y;
        cell_size.z = volume_size.z / buffer->voxels_size.z;

        //dim3 block(Tsdf::CTA_SIZE_X, Tsdf::CTA_SIZE_Y);
        dim3 block (16, 16);
        dim3 grid (divUp (buffer->voxels_size.x, block.x), divUp (buffer->voxels_size.y, block.y));

        tsdf23<<<grid, block>>>(depthScaled, volume, tranc_dist, Rcurr_inv, tcurr, intr, cell_size, *buffer);    
        //tsdf23normal_hack<<<grid, block>>>(depthScaled, volume, tranc_dist, Rcurr_inv, tcurr, intr, cell_size);

        cudaSafeCall ( cudaGetLastError () );
        cudaSafeCall (cudaDeviceSynchronize ());
      }

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void 
      clearTSDFSlice (PtrStep<short2> volume, pcl::gpu::kinfuLS::tsdf_buffer* buffer, int shiftX, int shiftY, int shiftZ)
      {
        int newX = buffer->origin_GRID.x + shiftX;
        int newY = buffer->origin_GRID.y + shiftY;

        int3 minBounds, maxBounds;
        
        //X
        if(newX >= 0)
        {
        minBounds.x = buffer->origin_GRID.x;
        maxBounds.x = newX;    
        }
        else
        {
        minBounds.x = newX + buffer->voxels_size.x; 
        maxBounds.x = buffer->origin_GRID.x + buffer->voxels_size.x;
        }
        
        if(minBounds.x > maxBounds.x)
        std::swap(minBounds.x, maxBounds.x);

      
        //Y
        if(newY >= 0)
        {
        minBounds.y = buffer->origin_GRID.y;
        maxBounds.y = newY;
        }
        else
        {
        minBounds.y = newY + buffer->voxels_size.y; 
        maxBounds.y = buffer->origin_GRID.y + buffer->voxels_size.y;
        }
        
        if(minBounds.y > maxBounds.y)
        std::swap(minBounds.y, maxBounds.y);
        
        //Z
        minBounds.z = buffer->origin_GRID.z;
        maxBounds.z = shiftZ;
      
        // call kernel
        dim3 block (32, 16);
        dim3 grid (1, 1, 1);
        grid.x = divUp (buffer->voxels_size.x, block.x);      
        grid.y = divUp (buffer->voxels_size.y, block.y);
        
        clearSliceKernel<<<grid, block>>>(volume, *buffer, minBounds, maxBounds);
        cudaSafeCall ( cudaGetLastError () );
        cudaSafeCall (cudaDeviceSynchronize ());        
      }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void
      uploadKnownToTSDFSlice (PtrStep<short2> volume, pcl::gpu::kinfuLS::tsdf_buffer* buffer, int shiftX, int shiftY, int shiftZ,
        PtrStep<short> known_status)
      {
        int oldX = buffer->origin_GRID.x - shiftX;
        int oldY = buffer->origin_GRID.y - shiftY;
        int oldZ = buffer->origin_GRID.z - shiftZ;

        int3 minBounds, maxBounds;

        //X
        if(oldX >= 0)
        {
        minBounds.x = buffer->origin_GRID.x;
        maxBounds.x = oldX;
        }
        else
        {
        minBounds.x = oldX + buffer->voxels_size.x;
        maxBounds.x = buffer->origin_GRID.x + buffer->voxels_size.x;
        }

        if(minBounds.x > maxBounds.x)
        std::swap(minBounds.x, maxBounds.x);


        //Y
        if(oldY >= 0)
        {
        minBounds.y = buffer->origin_GRID.y;
        maxBounds.y = oldY;
        }
        else
        {
        minBounds.y = oldY + buffer->voxels_size.y;
        maxBounds.y = buffer->origin_GRID.y + buffer->voxels_size.y;
        }

        if(minBounds.y > maxBounds.y)
        std::swap(minBounds.y, maxBounds.y);

        while (oldZ < 0)
          oldZ += buffer->voxels_size.z;
        while (oldZ >= buffer->voxels_size.z)
          oldZ -= buffer->voxels_size.z;

        //Z
        minBounds.z = oldZ;
        maxBounds.z = shiftZ;

        // call kernel
        dim3 block (32, 16);
        dim3 grid (1, 1, 1);
        grid.x = divUp (buffer->voxels_size.x, block.x);
        grid.y = divUp (buffer->voxels_size.y, block.y);

        uploadKnownToTSDFSliceKernel<<<grid, block>>>(volume, *buffer, minBounds, maxBounds, known_status);
        cudaSafeCall ( cudaGetLastError () );
        cudaSafeCall (cudaDeviceSynchronize ());
      }
    }
  }
}
