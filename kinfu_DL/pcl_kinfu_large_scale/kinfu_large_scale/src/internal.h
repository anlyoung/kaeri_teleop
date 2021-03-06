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

#ifndef PCL_KINFU_INTERNAL_HPP_
#define PCL_KINFU_INTERNAL_HPP_

#include <pcl/gpu/containers/safe_call.hpp>
#include <pcl/gpu/kinfu_large_scale/device.h>

//using namespace pcl::gpu;

namespace pcl
{
  namespace device
  {
    namespace kinfuLS
    {
      /** \brief Light source collection
        */ 
      struct LightSource
      {
        float3 pos[1];
        int number;
      };

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Maps
    
      /** \brief Perfoms bilateral filtering of disparity map
        * \param[in] src soruce map
        * \param[out] dst output map
        */
      void 
      bilateralFilter (const DepthMap& src, DepthMap& dst);
      
      /** \brief Computes depth pyramid
        * \param[in] src source
        * \param[out] dst destination
        */
      void 
      pyrDown (const DepthMap& src, DepthMap& dst);

      /** \brief Computes vertex map
        * \param[in] intr depth camera intrinsics
        * \param[in] depth depth
        * \param[out] vmap vertex map
        */
      void 
      createVMap (const Intr& intr, const DepthMap& depth, MapArr& vmap);
      
      /** \brief Computes normal map using cross product
        * \param[in] vmap vertex map
        * \param[out] nmap normal map
        */
      void 
      createNMap (const MapArr& vmap, MapArr& nmap);

			//************** added by DH **************//
			void
			deleteRobot (const Intr& intr, const float3& r_l_e, const float3& r_l_f, const float3& r_h, const DepthMap& depth, DepthMap& depth_env); 
			//************** added by DH **************//

      /** \brief Computes normal map using Eigen/PCA approach
        * \param[in] vmap vertex map
        * \param[out] nmap normal map
        */
      void 
      computeNormalsEigen (const MapArr& vmap, MapArr& nmap);

      /** \brief Performs affine tranform of vertex and normal maps
        * \param[in] vmap_src source vertex map
        * \param[in] nmap_src source vertex map
        * \param[in] Rmat Rotation mat
        * \param[in] tvec translation
        * \param[out] vmap_dst destination vertex map
        * \param[out] nmap_dst destination vertex map
        */
      void 
      transformMaps (const MapArr& vmap_src, const MapArr& nmap_src, const Mat33& Rmat, const float3& tvec, MapArr& vmap_dst, MapArr& nmap_dst);

      /** \brief Performs depth truncation
        * \param[out] depth depth map to truncation
        * \param[in] max_distance truncation threshold, values that are higher than the threshold are reset to zero (means not measurement)
        */
      void 
      truncateDepth(DepthMap& depth, float max_distance);

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //   ICP 
              
      /** \brief (now it's exra code) Computes corespondances map
        * \param[in] vmap_g_curr current vertex map in global coo space
        * \param[in] nmap_g_curr current normals map in global coo space
        * \param[in] Rprev_inv inverse camera rotation at previous pose
        * \param[in] tprev camera translation at previous pose
        * \param[in] intr camera intrinsics
        * \param[in] vmap_g_prev previous vertex map in global coo space
        * \param[in] nmap_g_prev previous vertex map in global coo space
        * \param[in] distThres distance filtering threshold
        * \param[in] angleThres angle filtering threshold. Represents sine of angle between normals
        * \param[out] coresp
        */
      void 
      findCoresp (const MapArr& vmap_g_curr, const MapArr& nmap_g_curr, const Mat33& Rprev_inv, const float3& tprev, const Intr& intr, 
                  const MapArr& vmap_g_prev, const MapArr& nmap_g_prev, float distThres, float angleThres, PtrStepSz<short2> coresp);

      /** \brief (now it's exra code) Computation Ax=b for ICP iteration
        * \param[in] v_dst destination vertex map (previous frame cloud)
        * \param[in] n_dst destination normal map (previous frame normals) 
        * \param[in] v_src source normal map (current frame cloud) 
        * \param[in] coresp Corespondances
        * \param[out] gbuf temp buffer for GPU reduction
        * \param[out] mbuf output GPU buffer for matrix computed
        * \param[out] matrixA_host A
        * \param[out] vectorB_host b
        */
      void 
      estimateTransform (const MapArr& v_dst, const MapArr& n_dst, const MapArr& v_src, const PtrStepSz<short2>& coresp,
                        DeviceArray2D<float>& gbuf, DeviceArray<float>& mbuf, float* matrixA_host, float* vectorB_host);


      /** \brief Computation Ax=b for ICP iteration
        * \param[in] Rcurr Rotation of current camera pose guess 
        * \param[in] tcurr translation of current camera pose guess 
        * \param[in] vmap_curr current vertex map in camera coo space
        * \param[in] nmap_curr current vertex map in camera coo space
        * \param[in] Rprev_inv inverse camera rotation at previous pose
        * \param[in] tprev camera translation at previous pose
        * \param[in] intr camera intrinsics
        * \param[in] vmap_g_prev previous vertex map in global coo space
        * \param[in] nmap_g_prev previous vertex map in global coo space
        * \param[in] distThres distance filtering threshold
        * \param[in] angleThres angle filtering threshold. Represents sine of angle between normals
        * \param[out] gbuf temp buffer for GPU reduction
        * \param[out] mbuf output GPU buffer for matrix computed
        * \param[out] matrixA_host A
        * \param[out] vectorB_host b
        */
      void 
      estimateCombined (const Mat33& Rcurr, const float3& tcurr, const MapArr& vmap_curr, const MapArr& nmap_curr, const Mat33& Rprev_inv, const float3& tprev, const Intr& intr, 
                        const MapArr& vmap_g_prev, const MapArr& nmap_g_prev, float distThres, float angleThres, 
                        DeviceArray2D<float>& gbuf, DeviceArray<float>& mbuf, float* matrixA_host, float* vectorB_host);
    
      /** \brief Computation Ax=b for ICP iteration
        * \param[in] Rcurr Rotation of current camera pose guess 
        * \param[in] tcurr translation of current camera pose guess 
        * \param[in] vmap_curr current vertex map in camera coo space
        * \param[in] nmap_curr current vertex map in camera coo space
        * \param[in] Rprev_inv inverse camera rotation at previous pose
        * \param[in] tprev camera translation at previous pose
        * \param[in] intr camera intrinsics
        * \param[in] vmap_g_prev previous vertex map in global coo space
        * \param[in] nmap_g_prev previous vertex map in global coo space
        * \param[in] distThres distance filtering threshold
        * \param[in] angleThres angle filtering threshold. Represents sine of angle between normals
        * \param[out] gbuf temp buffer for GPU reduction
        * \param[out] mbuf output GPU buffer for matrix computed
        * \param[out] matrixA_host A
        * \param[out] vectorB_host b
        */
      void
      estimateCombined (const Mat33& Rcurr, const float3& tcurr, const MapArr& vmap_curr, const MapArr& nmap_curr, const Mat33& Rprev_inv, const float3& tprev, const Intr& intr,
                        const MapArr& vmap_g_prev, const MapArr& nmap_g_prev, float distThres, float angleThres,
                        DeviceArray2D<double>& gbuf, DeviceArray<double>& mbuf, double* matrixA_host, double* vectorB_host);

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // TSDF volume functions            

      /** \brief Perform tsdf volume initialization
        *  \param[out] array volume to be initialized
        */
      PCL_EXPORTS void
      initVolume(int3 voxels_size,PtrStep<short2> array);

      /**
       * @brief clear a sphere in a tsdf volume
       * @param volume the volume
       * @param origin the current volume origin (due to volume shifting)
       * @param center the center of the sphere
       * @param radius the radius of the sphere
       */
      void
      clearSphere(PtrStep<short2> volume,int3 origin,float3 center,float radius);

      /**
       * @brief clear a bounding box in a tsdf volume
       * @param volume the volume
       * @param origin the current volume origin (due to volume shifting)
       * @param min the min value of the bounding box
       * @param max the max value of the bounding box
       */
      void
      clearBBox(PtrStep<short2> volume,const int3& origin,const float3& min,const float3& max);

			//************** added by DH **************//
	  	void
      extractBox(PtrStep<int> src, PtrStep<int> dst,const int3& origin,const int3& center);

			void
      copyBox(PtrStep<int> src, PtrStep<int> dst);
			//************** added by DH **************//

      //first version
      /** \brief Performs Tsfg volume uptation (extra obsolete now)
        * \param[in] depth_raw Kinect depth image
        * \param[in] intr camera intrinsics
        * \param[in] volume_size size of volume in mm
        * \param[in] Rcurr_inv inverse rotation for current camera pose
        * \param[in] tcurr translation for current camera pose
        * \param[in] tranc_dist tsdf truncation distance
        * \param[in] volume tsdf volume to be updated
        */
      void 
      integrateTsdfVolume (const PtrStepSz<ushort>& depth_raw, const Intr& intr, const float3& volume_size, 
                          const Mat33& Rcurr_inv, const float3& tcurr, float tranc_dist, PtrStep<short2> volume);

      //second version
      /** \brief Function that integrates volume if volume element contains: 2 bytes for round(tsdf*SHORT_MAX) and 2 bytes for integer weight.
        * \param[in] depth Kinect depth image
        * \param[in] intr camera intrinsics
        * \param[in] volume_size size of volume in mm
        * \param[in] Rcurr_inv inverse rotation for current camera pose
        * \param[in] tcurr translation for current camera pose
        * \param[in] tranc_dist tsdf truncation distance
        * \param[in] volume tsdf volume to be updated
        * \param[in] buffer cyclical buffer structure
        * \param[out] depthScaled Buffer for scaled depth along ray
        */
      PCL_EXPORTS void 
      integrateTsdfVolume (const PtrStepSz<ushort>& depth, const Intr& intr, const float3& volume_size, 
                          const Mat33& Rcurr_inv, const float3& tcurr, float tranc_dist, PtrStep<short2> volume, const pcl::gpu::kinfuLS::tsdf_buffer* buffer, DeviceArray2D<float>& depthScaled);
      
      /** \brief Function that clears the TSDF values. The clearing takes place from the origin (in indices) to an offset in X,Y,Z values accordingly
        * \param[in] volume Pointer to TSDF volume in GPU
        * \param[in] buffer Pointer to the buffer struct that contains information about memory addresses of the tsdf volume memory block, which are used for the cyclic buffer.
        * \param[in] shiftX Offset in indices that will be cleared from the TSDF volume. The clearing start from buffer.OriginX and stops in OriginX + shiftX
        * \param[in] shiftY Offset in indices that will be cleared from the TSDF volume. The clearing start from buffer.OriginY and stops in OriginY + shiftY
        * \param[in] shiftZ Offset in indices that will be cleared from the TSDF volume. The clearing start from buffer.OriginZ and stops in OriginZ + shiftZ
        */
      PCL_EXPORTS void 
      clearTSDFSlice (PtrStep<short2> volume, pcl::gpu::kinfuLS::tsdf_buffer* buffer, int shiftX, int shiftY, int shiftZ);

      PCL_EXPORTS void
      uploadKnownToTSDFSlice (PtrStep<short2> volume, pcl::gpu::kinfuLS::tsdf_buffer* buffer, int shiftX, int shiftY, int shiftZ,
        PtrStep<short> known_status);
      
      /** \brief Initialzied color volume
        * \param[out] color_volume color volume for initialization
        */
      void 
      initColorVolume(PtrStep<uchar4> color_volume);    

      /** \brief Performs integration in color volume
        * \param[in] intr Depth camera intrionsics structure
        * \param[in] tranc_dist tsdf truncation distance
        * \param[in] R_inv Inverse camera rotation
        * \param[in] t camera translation      
        * \param[in] vmap Raycasted vertex map
        * \param[in] colors RGB colors for current frame
        * \param[in] volume_size volume size in meters
        * \param[in] color_volume color volume to be integrated
        * \param[in] max_weight max weight for running color average. Zero means not average, one means average with prev value, etc.
        */    
      void 
      updateColorVolume(const Intr& intr, float tranc_dist, const Mat33& R_inv, const float3& t, const MapArr& vmap, 
              const PtrStepSz<uchar3>& colors, const float3& volume_size, PtrStep<uchar4> color_volume, int max_weight = 1);

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Raycast and view generation        
      /** \brief Generation vertex and normal maps from volume for current camera pose
        * \param[in] intr camera intrinsices
        * \param[in] Rcurr current rotation
        * \param[in] tcurr current translation
        * \param[in] tranc_dist volume truncation distance
        * \param[in] volume_size volume size in mm
        * \param[in] volume tsdf volume
        * \param[in] buffer cyclical buffer structure
        * \param[out] vmap output vertex map
        * \param[out] nmap output normals map
        */
      void 
      raycast (const Intr& intr, const Mat33& Rcurr, const float3& tcurr, float tranc_dist, float min_range,
              const float3& volume_size,
              const PtrStep<short2>& volume, const pcl::gpu::kinfuLS::tsdf_buffer* buffer, MapArr& vmap, MapArr& nmap);

      /** \brief Generation of vertex and known maps from volume for current camera pose
        * \param[in] intr camera intrinsices
        * \param[in] Rcurr current rotation
        * \param[in] tcurr current translation
        * \param[in] tranc_dist volume truncation distance
        * \param[in] volume_size volume size in mm
        * \param[in] volume tsdf volume
        * \param[in] buffer cyclical buffer structure
        * \param[out] vmap output vertex map
        * \param[out] umap output known map
        */
      void
      unkRaycast (const Intr& intr, const Mat33& Rcurr, const float3& tcurr, float tranc_dist, float min_range,
              const float3& volume_size,
              const PtrStep<short2>& volume, const pcl::gpu::kinfuLS::tsdf_buffer* buffer, MapArr& vmap, MapArr& umap);


      void
      unkRaycastBBox (const Intr& intr, const Mat33& Rcurr, const float3& tcurr, float tranc_dist, float min_range,
              const float3& volume_size,
              const PtrStep<short2>& volume, const pcl::gpu::kinfuLS::tsdf_buffer* buffer, MapArr& vmap, MapArr& umap,
              const float3 & bbox_min,const float3 & bbox_max);

      /** \brief Renders 3D image of the scene
        * \param[in] vmap vertex map
        * \param[in] nmap normals map
        * \param[in] light pose of light source
        * \param[out] dst buffer where image is generated
        */
      void 
      generateImage (const MapArr& vmap, const MapArr& nmap, const LightSource& light, PtrStepSz<uchar3> dst);


      /** \brief Renders depth image from give pose
        * \param[in] R_inv inverse camera rotation
        * \param[in] t camera translation
        * \param[in] vmap vertex map
        * \param[out] dst buffer where depth is generated
        */
      void
      generateDepth (const Mat33& R_inv, const float3& t, const MapArr& vmap, DepthMap& dst);

      /** \brief Paints 3D view with color map
        * \param[in] colors rgb color frame from OpenNI   
        * \param[out] dst output 3D view
        * \param[in] colors_weight weight for colors   
        */
      void 
      paint3DView(const PtrStep<uchar3>& colors, PtrStepSz<uchar3> dst, float colors_weight = 0.5f);

      /** \brief Performs resize of vertex map to next pyramid level by averaging each four points
        * \param[in] input vertext map
        * \param[out] output resized vertex map
        */
      void 
      resizeVMap (const MapArr& input, MapArr& output);
      
      /** \brief Performs resize of vertex map to next pyramid level by averaging each four normals
        * \param[in] input normal map
        * \param[out] output vertex map
        */
      void 
      resizeNMap (const MapArr& input, MapArr& output);

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Push data to TSDF
      
          /** \brief Loads the values of a tsdf point cloud to the tsdf volume in GPU
        * \param[in] volume tsdf volume 
        * \param[in] cloud_gpu contains the data to be pushed to the tsdf volume
        * \param[in] buffer Pointer to the buffer struct that contains information about memory addresses of the tsdf volume memory block, which are used for the cyclic buffer.
        */     
      /*PCL_EXPORTS*/ void 
      pushCloudAsSliceGPU (const PtrStep<short2>& volume, pcl::gpu::DeviceArray<PointType> cloud_gpu, const pcl::gpu::kinfuLS::tsdf_buffer* buffer);
      
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Cloud extraction 

      /** \brief Perform point cloud extraction of a slice from tsdf volume
        * \param[in] volume tsdf volume on GPU
        * \param[in] volume_size size of the volume
        * \param[in] buffer Pointer to the buffer struct that contains information about memory addresses of the tsdf volume memory block, which are used for the cyclic buffer.
        * \param[in] shiftX Offset in indices that will be cleared from the TSDF volume. The clearing start from buffer.OriginX and stops in OriginX + shiftX
        * \param[in] shiftY Offset in indices that will be cleared from the TSDF volume. The clearing start from buffer.OriginY and stops in OriginY + shiftY
        * \param[in] shiftZ Offset in indices that will be cleared from the TSDF volume. The clearing start from buffer.OriginZ and stops in OriginZ + shiftZ
        * \param[out] output_xyz buffer large enought to store point cloud xyz values
        * \param[out] output_intensities buffer large enought to store point cloud intensity values
        * \param[inout] last_data_transfer_matrix a VOLUME_Y * VOLUME_X matrix, an int for every thread, to track progress
        * \param[out] data_transfer_finished contains 0 if the transfer is incomplete (due to full buffer), 1 otherwise
        * \return number of point stored to passed buffer
        */ 
      PCL_EXPORTS size_t
      extractSliceAsCloud (const PtrStep<short2>& volume, const float3& volume_size,
        const pcl::gpu::kinfuLS::tsdf_buffer* buffer, const int shiftX, const int shiftY, const int shiftZ,
        PtrSz<PointType> output_xyz, PtrSz<float> output_intensities,
        PtrStep<int> last_data_transfer_matrix, int & data_transfer_finished
        );

      /** \brief Returns the size of the data transfer matrix required by extractSliceAsCloud
        * \param[out] height the height of the matrix
        * \param[out] width the width of the matrix, in number of (integer) elements
        */
      PCL_EXPORTS void
      getDataTransferCompletionMatrixSize(size_t & height, size_t & width);

      /** \brief Returns the size of the data transfer matrix required by extractSliceAsCloud
        * \param[in] the volume size in voxels
        * \param[out] height the height of the matrix
        * \param[out] width the width of the matrix, in number of (integer) elements
        */
      PCL_EXPORTS void
      getDataTransferCompletionMatrixSize(const int3 & voxels_size,size_t & height, size_t & width);

      PCL_EXPORTS size_t
      extractIncompletePointsAsCloud (const PtrStep<short2>& volume, const float3& volume_size,
        const pcl::gpu::kinfuLS::tsdf_buffer* buffer,const bool edges_only,
        PtrSz<PointType> output_xyz, PtrSz<float4> output_normals,
        PtrStep<int> last_data_transfer_matrix, int & data_transfer_finished);

      /** \brief Performs normals computation for given poins using tsdf volume
        * \param[in] volume tsdf volume
        * \param[in] volume_size volume size
        * \param[in] input points where normals are computed
        * \param[out] output normals. Could be float4 or float8. If for a point normal can't be computed, such normal is marked as nan.
        */ 
      template<typename NormalType> 
      void 
      extractNormals (const PtrStep<short2>& volume, const float3& volume_size, const PtrSz<PointType>& input, NormalType* output);

      /** \brief Performs colors exctraction from color volume
        * \param[in] color_volume color volume
        * \param[in] volume_size volume size
        * \param[in] points points for which color are computed
        * \param[out] colors output array with colors.
        */
      void 
      exctractColors(const PtrStep<uchar4>& color_volume, const float3& volume_size, const PtrSz<PointType>& points, uchar4* colors);

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Utility
      struct float8  { float x, y, z, w, c1, c2, c3, c4; };
      struct float12 { float x, y, z, w, normal_x, normal_y, normal_z, n4, c1, c2, c3, c4; };

      /** \brief Conversion from SOA to AOS
        * \param[in] vmap SOA map
        * \param[out] output Array of 3D points. Can be float4 or float8.
        */
      template<typename T> 
      void 
      convert (const MapArr& vmap, DeviceArray2D<T>& output);

      /** \brief Merges pcl::PointXYZ and pcl::Normal to PointNormal
        * \param[in] cloud points cloud
        * \param[in] normals normals cloud
        * \param[out] output array of PointNomals.
        */
      void 
      mergePointNormal(const DeviceArray<float4>& cloud, const DeviceArray<float8>& normals, const DeviceArray<float12>& output);

      /** \brief  Check for qnan (unused now) 
        * \param[in] value
        */
      inline bool 
      valid_host (float value)
      {
        const int test_QNAN = 0x7fffffff;  //QNAN
        return (*reinterpret_cast<char *>(&value)) != (*reinterpret_cast<const char *>(&test_QNAN));
      }

      /** \brief synchronizes CUDA execution */
      inline 
      void 
      sync () { cudaSafeCall (cudaDeviceSynchronize ()); }


      template<class D, class Matx> D&
      device_cast (Matx& matx)
      {
        return (*reinterpret_cast<D*>(matx.data ()));
      }
   
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Marching cubes implementation

      /** \brief Binds marching cubes tables to texture references */
      void 
      bindTextures(const int *edgeBuf, const int *triBuf, const int *numVertsBuf);            
      
      /** \brief Unbinds */
      void 
      unbindTextures();
      
      /** \brief Scans tsdf volume and retrieves occuped voxes
        * \param[in] volume tsdf volume
        * \param[out] occupied_voxels buffer for occuped voxels. The function fulfills first row with voxel ids and second row with number of vertextes.
        * \return number of voxels in the buffer
        */
      int
      getOccupiedVoxels(const PtrStep<short2>& volume, DeviceArray2D<int>& occupied_voxels);

      /** \brief Computes total number of vertexes for all voxels and offsets of vertexes in final triangle array
        * \param[out] occupied_voxels buffer with occuped voxels. The function fulfills 3nd only with offsets      
        * \return total number of vertexes
        */
      int
      computeOffsetsAndTotalVertexes(DeviceArray2D<int>& occupied_voxels);

      /** \brief Generates final triangle array
        * \param[in] volume tsdf volume
        * \param[in] occupied_voxels occuped voxel ids (first row), number of vertexes(second row), offsets(third row).
        * \param[in] volume_size volume size in meters
        * \param[out] output triangle array            
        */
      int
      generateTrianglesWithNormals (const PtrStep<short2>& volume,
        const pcl::gpu::kinfuLS::tsdf_buffer & buffer, float tranc_dist,
        DeviceArray<PointType>& output, DeviceArray<PointType>& normals,
        PtrStep<int> last_data_transfer_matrix, int & data_transfer_finished);
    }
  }
}

#endif /* PCL_KINFU_INTERNAL_HPP_ */
