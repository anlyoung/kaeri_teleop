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

#include <pcl/gpu/kinfu_large_scale/tsdf_volume.h>
#include "internal.h"
#include <algorithm>
#include <Eigen/Core>

#include <iostream>

using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;
using pcl::device::kinfuLS::device_cast;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::gpu::kinfuLS::TsdfVolume::TsdfVolume(const Vector3i& resolution) : resolution_(resolution), volume_host_ (new std::vector<float>), weights_host_ (new std::vector<short>)
{
  int volume_x = resolution_(0);
  int volume_y = resolution_(1);
  int volume_z = resolution_(2);

  volume_.create (volume_y * volume_z, volume_x);
  
  const Vector3f default_volume_size = Vector3f::Constant (3.f); //meters
  const float    default_tranc_dist  = 0.03f; //meters

  setSize(default_volume_size);
  setTsdfTruncDist(default_tranc_dist);

  reset();
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::setSize(const Vector3f& size)
{  
  size_ = size;
  setTsdfTruncDist(tranc_dist_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::setTsdfTruncDist (float distance)
{
  float cx = size_(0) / resolution_(0);
  float cy = size_(1) / resolution_(1);
  float cz = size_(2) / resolution_(2);

  tranc_dist_ = std::max (distance, 2.1f * std::max (cx, std::max (cy, cz)));  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::gpu::DeviceArray2D<int> 
pcl::gpu::kinfuLS::TsdfVolume::data() const
{
  return volume_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const Eigen::Vector3f&
pcl::gpu::kinfuLS::TsdfVolume::getSize() const
{
    return size_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const Eigen::Vector3i&
pcl::gpu::kinfuLS::TsdfVolume::getResolution() const
{
  return resolution_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const Eigen::Vector3f
pcl::gpu::kinfuLS::TsdfVolume::getVoxelSize() const
{    
  return size_.array () / resolution_.array().cast<float>();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float
pcl::gpu::kinfuLS::TsdfVolume::getTsdfTruncDist () const
{
  return tranc_dist_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void 
pcl::gpu::kinfuLS::TsdfVolume::reset()
{
  pcl::device::kinfuLS::initVolume(make_int3(resolution_.x(),resolution_.y(),resolution_.z()),volume_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::clearSphere(const Eigen::Vector3i & tsdf_origin,const Eigen::Vector3f & center,float radius)
{
  int3 o;
  o.x = tsdf_origin.x();
  o.y = tsdf_origin.y();
  o.z = tsdf_origin.z();
  float3 c;
  c.x = center.x();
  c.y = center.y();
  c.z = center.z();

  pcl::device::kinfuLS::clearSphere(volume_,o,c,radius);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::clearBBox(const Eigen::Vector3i & origin,const Eigen::Vector3f & min,const Eigen::Vector3f & max)
{
  int3 o;
  o.x = origin.x();
  o.y = origin.y();
  o.z = origin.z();
  float3 m;
  m.x = min.x();
  m.y = min.y();
  m.z = min.z();
  float3 M;
  M.x = max.x();
  M.y = max.y();
  M.z = max.z();

  pcl::device::kinfuLS::clearBBox(volume_,o,m,M);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::fetchCloudHost (PointCloud<PointXYZI>& cloud, bool connected26) const
{
  PointCloud<PointXYZ>::Ptr cloud_ptr_ = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
  PointCloud<PointIntensity>::Ptr cloud_i_ptr_ = PointCloud<PointIntensity>::Ptr (new PointCloud<PointIntensity>);
  fetchCloudHost(*cloud_ptr_);
	cloud_i_ptr_->resize(cloud_ptr_->size());		// added by DH
  pcl::concatenateFields (*cloud_ptr_, *cloud_i_ptr_, cloud);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::fetchCloudHost (PointCloud<PointType>& cloud, bool connected26) const
{
  int volume_x = resolution_(0);
  int volume_y = resolution_(1);
  int volume_z = resolution_(2);

  int cols;
  std::vector<int> volume_host;
  volume_.download (volume_host, cols);

  cloud.points.clear ();
  cloud.points.reserve (10000);

  const int DIVISOR = pcl::device::kinfuLS::DIVISOR; // SHRT_MAX;

#define FETCH(x, y, z) volume_host[(x) + (y) * volume_x + (z) * volume_y * volume_x]

  Array3f cell_size = getVoxelSize();

  for (int x = 1; x < volume_x-1; ++x)
  {
    for (int y = 1; y < volume_y-1; ++y)
    {
      for (int z = 0; z < volume_z-1; ++z)
      {
        int tmp = FETCH (x, y, z);
        int W = reinterpret_cast<short2*>(&tmp)->y;
        int F = reinterpret_cast<short2*>(&tmp)->x;

        if (W == 0 || F == DIVISOR)
          continue;

        Vector3f V = ((Array3f(x, y, z) + 0.5f) * cell_size).matrix ();

        if (connected26)
        {
          int dz = 1;
          for (int dy = -1; dy < 2; ++dy)
            for (int dx = -1; dx < 2; ++dx)
            {
              int tmp = FETCH (x+dx, y+dy, z+dz);

              int Wn = reinterpret_cast<short2*>(&tmp)->y;
              int Fn = reinterpret_cast<short2*>(&tmp)->x;
              if (Wn == 0 || Fn == DIVISOR)
                continue;

              if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
              {
                Vector3f Vn = ((Array3f (x+dx, y+dy, z+dz) + 0.5f) * cell_size).matrix ();
                Vector3f point = (V * abs (Fn) + Vn * abs (F)) / (abs (F) + abs (Fn));

                pcl::PointXYZ xyz;
                xyz.x = point (0);
                xyz.y = point (1);
                xyz.z = point (2);

                cloud.points.push_back (xyz);
              }
            }
          dz = 0;
          for (int dy = 0; dy < 2; ++dy)
            for (int dx = -1; dx < dy * 2; ++dx)
            {
              int tmp = FETCH (x+dx, y+dy, z+dz);

              int Wn = reinterpret_cast<short2*>(&tmp)->y;
              int Fn = reinterpret_cast<short2*>(&tmp)->x;
              if (Wn == 0 || Fn == DIVISOR)
                continue;

              if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
              {
                Vector3f Vn = ((Array3f (x+dx, y+dy, z+dz) + 0.5f) * cell_size).matrix ();
                Vector3f point = (V * abs(Fn) + Vn * abs(F))/(abs(F) + abs (Fn));

                pcl::PointXYZ xyz;
                xyz.x = point (0);
                xyz.y = point (1);
                xyz.z = point (2);

                cloud.points.push_back (xyz);
              }
            }
        }
        else /* if (connected26) */
        {
          for (int i = 0; i < 3; ++i)
          {
            int ds[] = {0, 0, 0};
            ds[i] = 1;

            int dx = ds[0];
            int dy = ds[1];
            int dz = ds[2];

            int tmp = FETCH (x+dx, y+dy, z+dz);

            int Wn = reinterpret_cast<short2*>(&tmp)->y;
            int Fn = reinterpret_cast<short2*>(&tmp)->x;
            if (Wn == 0 || Fn == DIVISOR)
              continue;

            if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
            {
              Vector3f Vn = ((Array3f (x+dx, y+dy, z+dz) + 0.5f) * cell_size).matrix ();
              Vector3f point = (V * abs (Fn) + Vn * abs (F)) / (abs (F) + abs (Fn));

              pcl::PointXYZ xyz;
              xyz.x = point (0);
              xyz.y = point (1);
              xyz.z = point (2);

              cloud.points.push_back (xyz);
            }
          }
        } /* if (connected26) */
      }
    }
  }
#undef FETCH
  cloud.width  = (int)cloud.points.size ();
  cloud.height = 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::fetchNormals (const DeviceArray<PointType>& cloud, DeviceArray<PointType>& normals) const
{
  normals.create (cloud.size ());
  const float3 device_volume_size = device_cast<const float3> (size_);
  pcl::device::kinfuLS::extractNormals (volume_, device_volume_size, cloud, (pcl::device::kinfuLS::PointType*)normals.ptr ());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void 
pcl::gpu::kinfuLS::TsdfVolume::pushSlice (PointCloud<PointXYZI>::Ptr existing_data_cloud, const pcl::gpu::kinfuLS::tsdf_buffer* buffer) const
{
  size_t gpu_array_size = existing_data_cloud->points.size ();

  if(gpu_array_size == 0)
  {
    //std::cout << "[KinfuTracker](pushSlice) Existing data cloud has no points\n";//CREATE AS PCL MESSAGE
    return;
  }

  const pcl::PointXYZI *first_point_ptr = &(existing_data_cloud->points[0]);

  pcl::gpu::DeviceArray<pcl::PointXYZI> cloud_gpu;
  cloud_gpu.upload (first_point_ptr, gpu_array_size);

  DeviceArray<float4>& cloud_cast = (DeviceArray<float4>&) cloud_gpu;
  //volume().pushCloudAsSlice (cloud_cast, &buffer_);
  pcl::device::kinfuLS::pushCloudAsSliceGPU (volume_, cloud_cast, buffer);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

size_t
pcl::gpu::kinfuLS::TsdfVolume::fetchSliceAsCloud (DeviceArray<PointType>& cloud_buffer_xyz,
  DeviceArray<float>& cloud_buffer_intensity,
  const pcl::gpu::kinfuLS::tsdf_buffer* buffer, int shiftX, int shiftY, int shiftZ,
  DeviceArray2D<int>& last_data_transfer_matrix,int & finished) const
{
  if (cloud_buffer_xyz.empty ())
    cloud_buffer_xyz.create (DEFAULT_CLOUD_BUFFER_SIZE/2);

  if (cloud_buffer_intensity.empty ()) {
    cloud_buffer_intensity.create (DEFAULT_CLOUD_BUFFER_SIZE/2);  
  }

  float3 device_volume_size = device_cast<const float3> (size_);
  
  size_t size = pcl::device::kinfuLS::extractSliceAsCloud (volume_, device_volume_size, buffer, shiftX, shiftY, shiftZ,
    cloud_buffer_xyz, cloud_buffer_intensity, last_data_transfer_matrix, finished);

  std::cout << " SIZE IS " << size << std::endl;
  
  return (size);
}

size_t
pcl::gpu::kinfuLS::TsdfVolume::fetchIncompletePointsAsCloud (DeviceArray<PointType>& cloud_buffer_xyz,
  DeviceArray<PointType> &cloud_buffer_normals, const tsdf_buffer* buffer,const bool edges_only,
  DeviceArray2D<int>& last_data_transfer_matrix, int& finished ) const
{
  if (cloud_buffer_xyz.empty ())
    cloud_buffer_xyz.create (DEFAULT_CLOUD_BUFFER_SIZE/2);

  if (cloud_buffer_normals.empty ()) {
    cloud_buffer_normals.create (DEFAULT_CLOUD_BUFFER_SIZE/2);
  }

  float3 device_volume_size = device_cast<const float3> (size_);

  size_t size = pcl::device::kinfuLS::extractIncompletePointsAsCloud (volume_, device_volume_size, buffer, edges_only,
    cloud_buffer_xyz, cloud_buffer_normals, last_data_transfer_matrix, finished);

  std::cout << " SIZE IS " << size << std::endl;

  return (size);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::gpu::kinfuLS::TsdfVolume::PointCloudXYZI::Ptr
pcl::gpu::kinfuLS::TsdfVolume::fetchSliceAsPointCloud (DeviceArray<PointType>& cloud_buffer_xyz,
  DeviceArray<float>& cloud_buffer_intensities, DeviceArray2D<int>& last_data_transfer_matrix,
  const pcl::gpu::kinfuLS::tsdf_buffer* buffer,
  int offset_x, int offset_y, int offset_z ) const
{
  DeviceArray<PointXYZ> points;
  DeviceArray<float> intensities;
  int finished;

  // create data transfer completion matrix and set it to all 0
  {
    size_t height, width;
    pcl::device::kinfuLS::getDataTransferCompletionMatrixSize (height, width);
    if (last_data_transfer_matrix.empty ())
      last_data_transfer_matrix.create (height, width);
    cudaSafeCall ( cudaMemset2D (last_data_transfer_matrix.ptr(), last_data_transfer_matrix.step(), 0,
                               width * sizeof(int), height));
    cudaSafeCall ( cudaGetLastError () );
    cudaSafeCall ( cudaDeviceSynchronize ());
  }

  PointCloudXYZI::Ptr current_slice (new PointCloudXYZI);
  PointCloudXYZ::Ptr current_slice_xyz (new PointCloudXYZ);
  PointCloudIntensity::Ptr current_slice_intensities (new PointCloudIntensity);

  do
  {
    size_t downloaded_size = this->fetchSliceAsCloud (cloud_buffer_xyz, cloud_buffer_intensities,
      buffer, offset_x, offset_y, offset_z, last_data_transfer_matrix, finished);

    points = DeviceArray<PointXYZ> (cloud_buffer_xyz.ptr (), downloaded_size);
    intensities = DeviceArray<float> (cloud_buffer_intensities.ptr (), downloaded_size);

    // Retrieving XYZ
    {
      PointCloudXYZ slice_inc_xyz;
      points.download (slice_inc_xyz.points);
      slice_inc_xyz.width = downloaded_size;
      slice_inc_xyz.height = 1;
      (*current_slice_xyz) += slice_inc_xyz;
    }

    // Retrieving intensities
    // TODO change this mechanism by using PointIntensity directly (in spite of float)
    // when tried, this lead to wrong intenisty values being extracted by fetchSliceAsCloud () (padding pbls?)
    {
      std::vector<float , Eigen::aligned_allocator<float> > intensities_vector_inc;
      intensities.download (intensities_vector_inc);

      const size_t old_size = current_slice_intensities->size();
      current_slice_intensities->points.resize (old_size + downloaded_size);
      for(size_t i = 0 ; i < intensities_vector_inc.size () ; ++i)
        current_slice_intensities->points[i + old_size].intensity = intensities_vector_inc[i];
      current_slice_intensities->width = (int) old_size + downloaded_size;
      current_slice_intensities->height = 1;
    }

  } while (!finished);

  // Concatenating XYZ and Intensities
  pcl::concatenateFields (*current_slice_xyz, *current_slice_intensities, *current_slice);
  current_slice->width = (int) current_slice->points.size ();
  current_slice->height = 1;

  return current_slice;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::gpu::kinfuLS::TsdfVolume::PointCloudXYZNormal::Ptr
pcl::gpu::kinfuLS::TsdfVolume::fetchIncompletePointsAsPointCloud (DeviceArray<PointType>& cloud_buffer_xyz,
  DeviceArray<PointType>& cloud_buffer_normals, const bool edges_only, DeviceArray2D<int>& last_data_transfer_matrix,
  const pcl::gpu::kinfuLS::tsdf_buffer* buffer) const
{
  DeviceArray<PointXYZ> points;
  DeviceArray<PointXYZ> normals;
  int finished;

  // create data transfer completion matrix and set it to all 0
  {
    size_t height, width;
    pcl::device::kinfuLS::getDataTransferCompletionMatrixSize (height, width);
    if (last_data_transfer_matrix.empty ())
      last_data_transfer_matrix.create (height, width);
    cudaSafeCall ( cudaMemset2D (last_data_transfer_matrix.ptr(), last_data_transfer_matrix.step(), 0,
                               width * sizeof(int), height));
    cudaSafeCall ( cudaGetLastError () );
    cudaSafeCall ( cudaDeviceSynchronize ());
  }

  PointCloudXYZNormal::Ptr current_slice (new PointCloudXYZNormal);
  PointCloudXYZ::Ptr current_slice_xyz (new PointCloudXYZ);
  PointCloudXYZ::Ptr current_slice_normals (new PointCloudXYZ);

  do
  {
    size_t downloaded_size = this->fetchIncompletePointsAsCloud (cloud_buffer_xyz, cloud_buffer_normals,
      buffer, edges_only, last_data_transfer_matrix, finished);

    points = DeviceArray<PointXYZ> (cloud_buffer_xyz.ptr (), downloaded_size);
    normals = DeviceArray<PointXYZ> (cloud_buffer_normals.ptr (), downloaded_size);

    // Retrieving XYZ
    {
      PointCloudXYZ slice_inc_xyz;
      points.download (slice_inc_xyz.points);
      slice_inc_xyz.width = downloaded_size;
      slice_inc_xyz.height = 1;
      (*current_slice_xyz) += slice_inc_xyz;
    }

    // Retrieving XYZ
    {
      PointCloudXYZ slice_inc_normals;
      normals.download (slice_inc_normals.points);
      slice_inc_normals.width = downloaded_size;
      slice_inc_normals.height = 1;
      (*current_slice_normals) += slice_inc_normals;
    }

  } while (!finished);

  // Concatenating XYZ and normals
  const uint out_size = current_slice_xyz->size ();
  current_slice->points.resize (out_size);
  current_slice->width = (int) current_slice->points.size ();
  current_slice->height = 1;
  for (size_t i = 0; i < out_size; i++)
  {
    (*current_slice)[i].x = (*current_slice_xyz)[i].x;
    (*current_slice)[i].y = (*current_slice_xyz)[i].y;
    (*current_slice)[i].z = (*current_slice_xyz)[i].z;
    (*current_slice)[i].normal_x = (*current_slice_normals)[i].x;
    (*current_slice)[i].normal_y = (*current_slice_normals)[i].y;
    (*current_slice)[i].normal_z = (*current_slice_normals)[i].z;
  }

  return current_slice;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::fetchNormals (const DeviceArray<PointType>& cloud, DeviceArray<NormalType>& normals) const
{
  normals.create (cloud.size ());
  const float3 device_volume_size = device_cast<const float3> (size_);
  pcl::device::kinfuLS::extractNormals (volume_, device_volume_size, cloud, (pcl::device::kinfuLS::float8*)normals.ptr ());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::convertToTsdfCloud (pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const
{
  int sx = header_.resolution(0);
  int sy = header_.resolution(1);
  int sz = header_.resolution(2);

  const int step = 2;
  const int cloud_size = static_cast<int> (header_.getVolumeSize() / (step*step*step));

  cloud->clear();
  cloud->reserve (std::min (cloud_size/10, 500000));

  int volume_idx = 0, cloud_idx = 0;
  // #pragma omp parallel for // if used, increment over idx not possible! use index calculation
  for (int z = 0; z < sz; z+=step)
    for (int y = 0; y < sy; y+=step)
      for (int x = 0; x < sx; x+=step, ++cloud_idx)
      {
        volume_idx = sx*sy*z + sx*y + x;
        // pcl::PointXYZI &point = cloud->points[cloud_idx];

        if (weights_host_->at(volume_idx) == 0 || volume_host_->at(volume_idx) > 0.98 )
          continue;

        pcl::PointXYZI point;
        point.x = x; point.y = y; point.z = z;//*64;
        point.intensity = volume_host_->at(volume_idx);
        cloud->push_back (point);
      }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::downloadTsdf (std::vector<float>& tsdf) const
{
  //printf("volume size: %d, %d \n", volume_.cols(), volume_.rows());
  tsdf.resize (volume_.cols() * volume_.rows());
  volume_.download(&tsdf[0], volume_.cols() * sizeof(int));
	//printf("downloaded \n");
	#pragma omp parallel for
	float sum = 0.0;
  for(int i = 0; i < (int) tsdf.size(); ++i)
  {
    float tmp = reinterpret_cast<short2*>(&tsdf[i])->x;
    tsdf[i] = tmp/pcl::device::kinfuLS::DIVISOR;
		//std::cout << i << " " << tsdf[i] << std::endl;
  }
	//printf("tsdf volume sum: %f\n",sum);
}

void
pcl::gpu::kinfuLS::TsdfVolume::downloadTsdfLocal () const
{
  pcl::gpu::kinfuLS::TsdfVolume::downloadTsdf (*volume_host_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::downloadTsdfAndWeights (std::vector<float>& tsdf, std::vector<short>& weights) const
{
  int volumeSize = volume_.cols() * volume_.rows();
  tsdf.resize (volumeSize);
  weights.resize (volumeSize);
  volume_.download(&tsdf[0], volume_.cols() * sizeof(int));
  
  #pragma omp parallel for
  for(int i = 0; i < (int) tsdf.size(); ++i)
  {
    short2 elem = *reinterpret_cast<short2*>(&tsdf[i]);
    tsdf[i] = (float)(elem.x)/pcl::device::kinfuLS::DIVISOR;    
    weights[i] = (short)(elem.y);    
  }
}

void
pcl::gpu::kinfuLS::TsdfVolume::downloadWeights(std::vector<short>& weights) const
{
  int volumeSize = volume_.cols() * volume_.rows();
  std::vector<float> tsdf;
  tsdf.resize (volumeSize);
  weights.resize (volumeSize);
  volume_.download(&tsdf[0], volume_.cols() * sizeof(int));

  #pragma omp parallel for
  for(int i = 0; i < (int) tsdf.size(); ++i)
  {
    short2 elem = *reinterpret_cast<short2*>(&tsdf[i]);
    weights[i] = (short)(elem.y);
  }
}


void
pcl::gpu::kinfuLS::TsdfVolume::downloadTsdfAndWeightsLocal () const
{
  pcl::gpu::kinfuLS::TsdfVolume::downloadTsdfAndWeights (*volume_host_, *weights_host_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool
pcl::gpu::kinfuLS::TsdfVolume::save (const std::string &filename, bool binary) const
{
  pcl::console::print_info ("Saving TSDF volume to "); pcl::console::print_value ("%s ... ", filename.c_str());
  std::cout << std::flush;

  std::ofstream file (filename.c_str(), binary ? std::ios_base::binary : std::ios_base::out);

  if (file.is_open())
  {
    if (binary)
    {
      // HEADER
      // write resolution and size of volume
      file.write ((char*) &header_, sizeof (Header));
      /* file.write ((char*) &header_.resolution, sizeof(Eigen::Vector3i));
      file.write ((char*) &header_.volume_size, sizeof(Eigen::Vector3f));
      // write  element size
      int volume_element_size = sizeof(VolumeT);
      file.write ((char*) &volume_element_size, sizeof(int));
      int weights_element_size = sizeof(WeightT);
      file.write ((char*) &weights_element_size, sizeof(int)); */

      // DATA
      // write data
      file.write ((char*) &(volume_host_->at(0)), volume_host_->size()*sizeof(float));
      file.write ((char*) &(weights_host_->at(0)), weights_host_->size()*sizeof(short));
    }
    else
    {
      // write resolution and size of volume and element size
      file << header_.resolution(0) << " " << header_.resolution(1) << " " << header_.resolution(2) << std::endl;
      file << header_.volume_size(0) << " " << header_.volume_size(1) << " " << header_.volume_size(2) << std::endl;
      file << sizeof (float) << " " << sizeof(short) << std::endl;

      // write data
      for (std::vector<float>::const_iterator iter = volume_host_->begin(); iter != volume_host_->end(); ++iter)
        file << *iter << std::endl;
    }

    file.close();
  }
  else
  {
    pcl::console::print_error ("[saveTsdfVolume] Error: Couldn't open file %s.\n", filename.c_str());
    return false;
  }

  pcl::console::print_info ("done [%d voxels]\n", this->size ());

  return true;
}


bool
pcl::gpu::kinfuLS::TsdfVolume::load (const std::string &filename, bool binary)
{
  pcl::console::print_info ("Loading TSDF volume from "); pcl::console::print_value ("%s ... ", filename.c_str());
  std::cout << std::flush;

  std::ifstream file (filename.c_str());

  if (file.is_open())
  {
    if (binary)
    {
      // read HEADER
      file.read ((char*) &header_, sizeof (Header));
      /* file.read (&header_.resolution, sizeof(Eigen::Array3i));
      file.read (&header_.volume_size, sizeof(Eigen::Vector3f));
      file.read (&header_.volume_element_size, sizeof(int));
      file.read (&header_.weights_element_size, sizeof(int)); */

      // check if element size fits to data
      if (header_.volume_element_size != sizeof(float))
      {
        pcl::console::print_error ("[TSDFVolume::load] Error: Given volume element size (%d) doesn't fit data (%d)", sizeof(float), header_.volume_element_size);
        return false;
      }
      if ( header_.weights_element_size != sizeof(short))
      {
        pcl::console::print_error ("[TSDFVolume::load] Error: Given weights element size (%d) doesn't fit data (%d)", sizeof(short), header_.weights_element_size);
        return false;
      }

      // read DATA
      int num_elements = header_.getVolumeSize();
      volume_host_->resize (num_elements);
      weights_host_->resize (num_elements);
      file.read ((char*) &(*volume_host_)[0], num_elements * sizeof(float));
      file.read ((char*) &(*weights_host_)[0], num_elements * sizeof(short));
    }
    else
    {
      pcl::console::print_error ("[TSDFVolume::load] Error: ASCII loading not implemented.\n");
    }

    file.close ();
  }
  else
  {
    pcl::console::print_error ("[TSDFVolume::load] Error: Cloudn't read file %s.\n", filename.c_str());
    return false;
  }

  const Eigen::Vector3i &res = this->gridResolution();
  pcl::console::print_info ("done [%d voxels, res %dx%dx%d]\n", this->size(), res[0], res[1], res[2]);

  return true;
}
