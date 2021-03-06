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

#ifndef PCL_KINFU_KINFUTRACKER_HPP_
#define PCL_KINFU_KINFUTRACKER_HPP_

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include <Eigen/Core>
#include <vector>
//#include <boost/graph/buffer_concepts.hpp>

#include <pcl/gpu/kinfu_large_scale/device.h>

#include <pcl/gpu/kinfu_large_scale/float3_operations.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/kinfu_large_scale/pixel_rgb.h>
#include <pcl/gpu/kinfu_large_scale/tsdf_volume.h>
#include <pcl/gpu/kinfu_large_scale/color_volume.h>
#include <pcl/gpu/kinfu_large_scale/raycaster.h>

#include <pcl/gpu/kinfu_large_scale/cyclical_buffer.h>
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h>
//#include <pcl/gpu/kinfu_large_scale/standalone_marching_cubes.h>

namespace pcl
{
  namespace gpu
  {
    namespace kinfuLS
    {        
      /** \brief KinfuTracker class encapsulates implementation of Microsoft Kinect Fusion algorithm
        * \author Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
        */
      class PCL_EXPORTS KinfuTracker
      {
        public:

          /** \brief Pixel type for rendered image. */
          typedef pcl::gpu::kinfuLS::PixelRGB PixelRGB;

          typedef DeviceArray2D<PixelRGB> View;
          typedef DeviceArray2D<unsigned short> DepthMap;

          typedef pcl::PointXYZ PointType;
          typedef pcl::Normal NormalType;
          typedef PointCloud<PointNormal> PointCloudXYZNormal;

          void 
          performLastScan (){perform_last_scan_ = true; PCL_WARN ("Kinfu will exit after next shift\n");}
          
          bool
          isFinished (){return (finished_);}

          /** \brief Constructor
            * \param[in] volumeSize physical size of the volume represented by the tdsf volume. In meters.
            * \param[in] shiftingDistance when the camera target point is farther than shiftingDistance from the center of the volume, shiting occurs. In meters.
            * \note The target point is located at (0, 0, 0.6*volumeSize) in camera coordinates.
            * \param[in] rows height of depth image
            * \param[in] cols width of depth image
            */
          KinfuTracker (const Eigen::Vector3f &volumeSize, const float shiftingDistance, int rows = 480, int cols = 640);

          /** \brief Sets Depth camera intrinsics
            * \param[in] fx focal length x 
            * \param[in] fy focal length y
            * \param[in] cx principal point x
            * \param[in] cy principal point y
            */
          void
          setDepthIntrinsics (float fx, float fy, float cx = -1, float cy = -1);

          /** \brief Sets initial camera pose relative to volume coordiante space
            * \param[in] pose Initial camera pose
            */
          void
          setInitialCameraPose (const Eigen::Affine3f& pose);
                          
          /** \brief Sets truncation threshold for depth image for ICP step only! This helps 
            *  to filter measurements that are outside tsdf volume. Pass zero to disable the truncation.
            * \param[in] max_icp_distance Maximal distance, higher values are reset to zero (means no measurement). 
            */
          void
          setDepthTruncationForICP (float max_icp_distance = 0.f);

          /** \brief Sets ICP filtering parameters.
            * \param[in] distThreshold distance.
            * \param[in] sineOfAngle sine of angle between normals.
            */
          void
          setIcpCorespFilteringParams (float distThreshold, float sineOfAngle);
          
          /** \brief Sets integration threshold. TSDF volume is integrated iff a camera movement metric exceedes the threshold value. 
            * The metric represents the following: M = (rodrigues(Rotation).norm() + alpha*translation.norm())/2, where alpha = 1.f (hardcoded constant)
            * \param[in] threshold a value to compare with the metric. Suitable values are ~0.001          
            */
          void
          setCameraMovementThreshold(float threshold = 0.001f);

          /** \brief Performs initialization for color integration. Must be called before calling color integration. 
            * \param[in] max_weight max weighe for color integration. -1 means default weight.
            */
          void
          initColorIntegration(int max_weight = -1);        

          /** \brief Returns cols passed to ctor */
          int
          cols ();

          /** \brief Returns rows passed to ctor */
          int
          rows ();

          void clearSphere(const Eigen::Vector3f & center,float radius);

          void clearBBox(const Eigen::Vector3f & min,const Eigen::Vector3f & max);
					
					//************** added by DH **************//
          Eigen::Vector3d extractBox(const Eigen::Vector3f & center);								
					//************** added by DH **************//

          struct THint
          {
            enum Type
            {
              HINT_TYPE_NONE = 0,
              HINT_TYPE_HINT,
              HINT_TYPE_FORCED,
              MAX_HINT_TYPE
            };

            THint(Type t,const Eigen::Affine3f& h): transform(h),type(t),ignore_minimum_movement(false) {}
            THint(Type t,const Eigen::Affine3f& h,bool i): transform(h),type(t),ignore_minimum_movement(i) {}
            THint(): type(HINT_TYPE_NONE), ignore_minimum_movement(false) {}

            Eigen::Affine3f transform;
            Type type;
            bool ignore_minimum_movement;  // Usually, the kinfu won't update if the last movement
                                           // is under a certain threshold. Set this to true to
                                           // update anyway.

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
          };

          /** \brief Processes next frame.
            * \param[in] depth next frame with values in millimeters
            * \return true if can render 3D view.
            */
          bool operator() (const DepthMap& depth,const THint & hint = THint());

          /** \brief Processes next frame (both depth and color integration). Please call initColorIntegration before invpoking this.
            * \param[in] depth next depth frame with values in millimeters
            * \param[in] colors next RGB frame
            * \return true if can render 3D view.
            */
          bool operator() (const DepthMap& depth, const View& colors, const THint & hint = THint());

					//************** added by DH **************//
          bool operator() (const DepthMap& depth, const std::vector<Eigen::Vector3f> robot_joints, const THint & hint = THint());
					//************** added by DH **************//

          /** \brief Returns camera pose at given time, default the last pose
            * \param[in] time Index of frame for which camera pose is returned.
            * \return camera pose
            */
          Eigen::Affine3f
          getCameraPose (int time = -1) const;
          
          Eigen::Affine3f
          getLastEstimatedPose () const;

          /** \brief shifts the TSDF volume near a point, to prepare for further computations
            * \param[in] pose the virtual kinect pose
            * \param[in] distance the distance at which the volume must be placed if a shift occurs
            * \returns whether a shift was necessary
            */
          bool shiftNear (const Eigen::Affine3f & pose, float distance = 0.0f, bool forced = false);

          /** \brief is last shift complete or still in progress?
            * \returns if it's complete
            */
          bool isShiftComplete() const {return cyclical_.isOldCubeRetrieved(); }

          /** \brief if not isShiftComplete(), you may try updateShift() to update it
            *
            */
          void updateShift() {cyclical_.checkOldCubeRetrieval(tsdf_volume_);}

          /** \brief Returns number of poses including initial */
          size_t
          getNumberOfPoses () const;

          /** \brief Returns TSDF volume storage */
          const TsdfVolume& volume() const;

          /** \brief Returns TSDF volume storage */
          TsdfVolume& volume();

					//************** added by DH **************//					
          const TsdfVolume& volume_box() const;

          TsdfVolume& volume_box();
					//************** added by DH **************//

          /** \brief Returns color volume storage */
          const ColorVolume& colorVolume() const;

          /** \brief Returns color volume storage */
          ColorVolume& colorVolume();
          
          /** \brief Renders 3D scene to display to human
            * \param[out] view output array with image
            */
          void
          getImage (View& view) const;
          
          /** \brief Returns point cloud abserved from last camera pose
            * \param[out] cloud output array for points
            */
          void
          getLastFrameCloud (DeviceArray2D<PointType>& cloud) const;

          /** \brief Returns point cloud abserved from last camera pose
            * \param[out] normals output array for normals
            */
          void
          getLastFrameNormals (DeviceArray2D<NormalType>& normals) const;
          
          
          /** \brief Returns pointer to the cyclical buffer structure
            */
          tsdf_buffer* 
          getCyclicalBufferStructure () 
          {
            return (cyclical_.getBuffer ());
          }

          /** \brief Returns reference to the cyclical buffer
            */
          CyclicalBuffer & getCyclicalBuffer()
          {
            return cyclical_;
          }


          /** \brief Returns voxel size
            */
          float getVoxelSize() const;
          
          /** \brief Extract the world and save it.
            */
          void
          extractAndSaveWorld ();

					//************** added by DH **************//
					pcl::PointCloud<pcl::PointXYZI>::Ptr extractBox();
					//************** added by DH **************//

          /** \brief Returns a copy of the current world. Computationally expensive.
           */
          pcl::PointCloud<pcl::PointXYZI>::Ptr extractWorld();

          /**
           * \brief Returns the current world and resets the tracker. Less computationally expensive
           *   than the previous one.
           */
          pcl::PointCloud<pcl::PointXYZI>::Ptr extractWorldAndReset();

          void syncKnownPoints();

          void setWeightCubeListener(CyclicalBuffer::WeightCubeListener::Ptr listener)
          {
            cyclical_.setWeightCubeListener(listener);
          }

          /**
           * @brief enables known points extraction
           */
          void setExtractKnownPoints(bool e)
          {
            cyclical_.setExtractKnownPoints(e);
          }

          void setIncompletePointsListener(CyclicalBuffer::IncompletePointsListener::Ptr listener)
          {
            cyclical_.setIncompletePointsListener(listener);
          }
          
          /** \brief Returns true if ICP is currently lost */
          bool
          icpIsLost ()
          {
            return (lost_);
          }
          
          /** \brief Performs the tracker reset to initial  state. It's used if camera tracking fails. */
          void
          reset ();
          
          void
          setDisableICP () 
          { 
            disable_icp_ = !disable_icp_;
            PCL_WARN("ICP is %s\n", !disable_icp_?"ENABLED":"DISABLED");
          }

          /** \brief Return whether the last update resulted in a shift */
          inline bool
          hasShifted () const
          {
            return (has_shifted_);
          }

          void
          getDepthIntrinsics (float & fx,float & fy, float & cx, float & cy) const {fx = fx_; fy = fy_; cx = cx_; cy = cy_; }

          void getCurrentWorldBoundingBox(Eigen::Vector3f& bbox_min,Eigen::Vector3f& bbox_max) const
          {
            cyclical_.getCurrentWorldBoundingBox(bbox_min,bbox_max);
          }
          void getCurrentCubeBoundingBox(Eigen::Vector3f& bbox_min,Eigen::Vector3f& bbox_max) const
          {
            cyclical_.getCurrentCubeBoundingBox(bbox_min,bbox_max);
          }

        private:
          
          /** \brief Allocates all GPU internal buffers.
            * \param[in] rows_arg
            * \param[in] cols_arg          
            */
          void
          allocateBufffers (int rows_arg, int cols_arg);
                   
          /** \brief Number of pyramid levels */
          enum { LEVELS = 3 };

          /** \brief ICP Correspondences  map type */
          typedef DeviceArray2D<int> CorespMap;

          /** \brief Vertex or Normal Map type */
          typedef DeviceArray2D<float> MapArr;
          
          typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
          typedef Eigen::Vector3f Vector3f;
          
          /** \brief helper function that converts transforms from host to device types
            * \param[in] transformIn1 first transform to convert
            * \param[in] transformIn2 second transform to convert
            * \param[in] translationIn1 first translation to convert
            * \param[in] translationIn2 second translation to convert
            * \param[out] transformOut1 result of first transform conversion
            * \param[out] transformOut2 result of second transform conversion
            * \param[out] translationOut1 result of first translation conversion
            * \param[out] translationOut2 result of second translation conversion
            */
          inline void 
          convertTransforms (Matrix3frm& transform_in_1, Matrix3frm& transform_in_2, Eigen::Vector3f& translation_in_1, Eigen::Vector3f& translation_in_2,
                                         pcl::device::kinfuLS::Mat33& transform_out_1, pcl::device::kinfuLS::Mat33& transform_out_2, float3& translation_out_1, float3& translation_out_2);
          
          /** \brief helper function that converts transforms from host to device types
            * \param[in] transformIn1 first transform to convert
            * \param[in] transformIn2 second transform to convert
            * \param[in] translationIn translation to convert
            * \param[out] transformOut1 result of first transform conversion
            * \param[out] transformOut2 result of second transform conversion
            * \param[out] translationOut result of translation conversion
            */
          inline void 
          convertTransforms (Matrix3frm& transform_in_1, Matrix3frm& transform_in_2, Eigen::Vector3f& translation_in,
                                         pcl::device::kinfuLS::Mat33& transform_out_1, pcl::device::kinfuLS::Mat33& transform_out_2, float3& translation_out);
          
          /** \brief helper function that converts transforms from host to device types
            * \param[in] transformIn transform to convert
            * \param[in] translationIn translation to convert
            * \param[out] transformOut result of transform conversion
            * \param[out] translationOut result of translation conversion
            */
          inline void 
          convertTransforms (Matrix3frm& transform_in, Eigen::Vector3f& translation_in,
                                         pcl::device::kinfuLS::Mat33& transform_out, float3& translation_out);
          
          /** \brief helper function that pre-process a raw detph map the kinect fusion algorithm.
            * The raw depth map is first blured, eventually truncated, and downsampled for each pyramid level.
            * Then, vertex and normal maps are computed for each pyramid level.
            * \param[in] depth_raw the raw depth map to process
            * \param[in] cam_intrinsics intrinsics of the camera used to acquire the depth map
            */
          inline void 
          prepareMaps (const DepthMap& depth_raw, const pcl::device::kinfuLS::Intr& cam_intrinsics);
 
          /** \brief helper function that performs GPU-based ICP, using vertex and normal maps stored in v/nmaps_curr_ and v/nmaps_g_prev_
            * The function requires the previous local camera pose (translation and inverted rotation) as well as camera intrinsics.
            * It will return the newly computed pose found as global rotation and translation.
            * \param[in] cam_intrinsics intrinsics of the camera
            * \param[in] previous_global_rotation previous local rotation of the camera
            * \param[in] previous_global_translation previous local translation of the camera
            * \param[out] current_global_rotation computed global rotation
            * \param[out] current_global_translation computed global translation
            * \return true if ICP has converged.
            */
          inline bool 
          performICP(const pcl::device::kinfuLS::Intr& cam_intrinsics, Matrix3frm& previous_global_rotation, Vector3f& previous_global_translation, Matrix3frm& current_global_rotation, Vector3f& current_global_translation);
          
          
          /** \brief helper function that performs GPU-based ICP, using the current and the previous depth-maps (i.e. not using the synthetic depth map generated from the tsdf-volume)
            * The function requires camera intrinsics.
            * It will return the transformation between the previous and the current depth map.
            * \param[in] cam_intrinsics intrinsics of the camera
            * \param[out] resulting_rotation computed global rotation
            * \param[out] resulting_translation computed global translation
            * \return true if ICP has converged.
            */
          inline bool 
          performPairWiseICP(const pcl::device::kinfuLS::Intr cam_intrinsics, Matrix3frm& resulting_rotation, Vector3f& resulting_translation);
          
          /** \brief Helper function that copies v_maps_curr and n_maps_curr to v_maps_prev_ and n_maps_prev_ */
          inline void 
          saveCurrentMaps();

          /** \brief For small rounding errors */
          void normalizeRotationMatrix(Matrix3frm & rotation) const;
          
          /** \brief Cyclical buffer object */
          CyclicalBuffer cyclical_;
          
          /** \brief Height of input depth image. */
          int rows_;
          
          /** \brief Width of input depth image. */
          int cols_;
          
          /** \brief Frame counter */
          int global_time_;

          /** \brief Truncation threshold for depth image for ICP step */
          float max_icp_distance_;

          /** \brief Intrinsic parameters of depth camera. */
          float fx_, fy_, cx_, cy_;

          /** \brief Tsdf volume container. */
          TsdfVolume::Ptr tsdf_volume_;
					
					//************** added by DH **************//
					TsdfVolume::Ptr tsdf_volume_box;
					MarchingCubes marching_cubes_;
        	//************** added by DH **************//

          /** \brief Color volume container. */
          ColorVolume::Ptr color_volume_;
                  
          /** \brief Initial camera rotation in volume coo space. */
          Matrix3frm init_Rcam_;

          /** \brief Initial camera position in volume coo space. */
          Vector3f   init_tcam_;

          /** \brief array with IPC iteration numbers for each pyramid level */
          int icp_iterations_[LEVELS];
          
          /** \brief distance threshold in correspondences filtering */
          float  distThres_;
          
          /** \brief angle threshold in correspondences filtering. Represents max sine of angle between normals. */
          float angleThres_;
          
          /** \brief Depth pyramid. */
          std::vector<DepthMap> depths_curr_;
          
          /** \brief Vertex maps pyramid for current frame in global coordinate space. */
          std::vector<MapArr> vmaps_g_curr_;
          
          /** \brief Normal maps pyramid for current frame in global coordinate space. */
          std::vector<MapArr> nmaps_g_curr_;

          /** \brief Vertex maps pyramid for previous frame in global coordinate space. */
          std::vector<MapArr> vmaps_g_prev_;
          
          /** \brief Normal maps pyramid for previous frame in global coordinate space. */
          std::vector<MapArr> nmaps_g_prev_;
                  
          /** \brief Vertex maps pyramid for current frame in current coordinate space. */
          std::vector<MapArr> vmaps_curr_;
          
          /** \brief Normal maps pyramid for current frame in current coordinate space. */
          std::vector<MapArr> nmaps_curr_;
          
          /** \brief Vertex maps pyramid for previous frame in current coordinate space. */
          std::vector<MapArr> vmaps_prev_;
          
          /** \brief Normal maps pyramid for previous frame in current coordinate space. */
          std::vector<MapArr> nmaps_prev_;

          /** \brief Array of buffers with ICP correspondences for each pyramid level. */
          std::vector<CorespMap> coresps_;
          
          /** \brief Buffer for storing scaled depth image */
          DeviceArray2D<float> depthRawScaled_;
          
          /** \brief Temporary buffer for ICP */
          DeviceArray2D<double> gbuf_;
          
          /** \brief Buffer to store MLS matrix. */
          DeviceArray<double> sumbuf_;

          /** \brief Array of camera rotation matrices for each moment of time. */
          std::vector<Matrix3frm> rmats_;
          
          /** \brief Array of camera translations for each moment of time. */
          std::vector<Vector3f> tvecs_;

          /** \brief Camera movement threshold. TSDF is integrated iff a camera movement metric exceedes some value. */
          float integration_metric_threshold_;          
                  
          /** \brief When set to true, KinFu will extract the whole world and mesh it. */
          bool perform_last_scan_;
          
          /** \brief When set to true, KinFu notifies that it is finished scanning and can be stopped. */
          bool finished_;

          /** \brief // when the camera target point is farther than DISTANCE_THRESHOLD from the current cube's center, shifting occurs. In meters . */
          float shifting_distance_;

          /** \brief Size of the TSDF volume in meters. */
          float volume_size_;
          
          /** \brief True if ICP is lost */
          bool lost_;
          
          /** \brief Last estimated rotation (estimation is done via pairwise alignment when ICP is failing) */
          Matrix3frm last_estimated_rotation_;
          
          /** \brief Last estimated translation (estimation is done via pairwise alignment when ICP is failing) */
          Vector3f last_estimated_translation_;
               
          
          bool disable_icp_;

          /** \brief True or false depending on if there was a shift in the last pose update */
          bool has_shifted_;

          bool just_shifted_;
          
        public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      };
    }
  }
};

#endif /* PCL_KINFU_KINFUTRACKER_HPP_ */
