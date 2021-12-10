// STL
#include <vector>
#include <algorithm>

// Eigen
#include <Eigen/Core>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// PCL/GPU
#include <pcl/gpu/kinfu_large_scale/standalone_marching_cubes.h>
#include <pcl/gpu/kinfu_large_scale/impl/standalone_marching_cubes.hpp>

// ROS
#include <sensor_msgs/fill_image.h>

class MarchingCubeDL{
public:
	typedef boost::shared_ptr<ros::Publisher> PublisherPtr;
  typedef pcl::gpu::kinfuLS::KinfuTracker KinfuTracker;
  typedef unsigned int uint;
  typedef uint64_t uint64;
  typedef pcl::PolygonMesh Mesh;
  typedef pcl::PointNormal PointXYZNormal;
  typedef pcl::PointCloud<pcl::PointXYZI> TsdfCloud;
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;
  typedef pcl::PointCloud<pcl::PointNormal> PointCloudXYZNormal;
  typedef pcl::gpu::kinfuLS::RayCaster RayCaster;
  typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
  typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > Vector3iVector;
  struct Triangle
  {
    uint64 & operator[](const std::size_t index) {return indices[index]; }
    uint64 operator[](const std::size_t index) const {return indices[index]; }

  	Eigen::Vector3d toEigen() const
		{
			Eigen::Vector3d v(indices[0], indices[1], indices[2]);
			return v; 
		}	
    private:
    uint64 indices[3];
  };
  typedef std::vector<Triangle> TriangleVector;
  typedef boost::shared_ptr<TriangleVector> TriangleVectorPtr;
  typedef boost::shared_ptr<const TriangleVector> TriangleVectorConstPtr;

	MarchingCubeDL(){
	}

	~MarchingCubeDL(){
	}

	void extractBoxMesh(KinfuTracker * kinfu, std::vector<Eigen::Vector3d> & triangle_meshes, bool silent){
		//pcl::PointCloud<pcl::PointXYZI>::Ptr kinfu_cloud = kinfu->extractBox();
		
		std::vector<Mesh::Ptr> meshes;
	
		if(!silent) ROS_INFO("kinfu: Marching cubes...");
		//if (!marchingCubesBox(kinfu->getVoxelSize()*pcl::device::kinfuLS::BOX_X, kinfu_cloud, meshes, silent))
		if (!marchingCubesBox(kinfu->getVoxelSize()*pcl::device::kinfuLS::BOX_X, kinfu->volume_box(), meshes, silent))
			return; // ERROR
		if(!silent) std::cout << meshes.size() << " meshes generated" << std::endl;
		for (int i = 0; i < meshes.size(); i++){
			if(!silent) std::cout << "mesh" << i << " has " << meshes[i]->polygons.size() << " polygons" << std::endl;
			if(!silent) std::cout << "mesh" << i << " has " << meshes[i]->cloud.height << " by " << meshes[i]->cloud.width << " points" << std::endl;
		}
		//kinfu_cloud->clear(); // save some memory

		std::vector<PointCloudXYZNormal::Ptr> clouds;
		std::vector<TriangleVectorPtr> clouds_triangles;

		if(!silent) ROS_INFO("kinfu: Divide triangles and points...");
		for (uint i = 0; i < meshes.size(); i++)
		{
		  clouds.push_back(PointCloudXYZNormal::Ptr(new PointCloudXYZNormal()));
		  clouds_triangles.push_back(TriangleVectorPtr(new TriangleVector()));

		  separateMesh<pcl::PointNormal>(meshes[i],clouds[i],clouds_triangles[i]);
		}
		meshes.clear(); // save some memory
		//std::cout << clouds_triangles.size() << " triangles generated" << std::endl;
		//std::cout << clouds.size() << " points generated" << std::endl;

		TriangleVectorPtr triangles(new TriangleVector());
		PointCloudXYZNormal::Ptr cloud_(new PointCloudXYZNormal());

		if(!silent) ROS_INFO("kinfu: Merging points...");
		mergePointCloudsAndMesh<pcl::PointNormal>(clouds,cloud_,&clouds_triangles,&(*triangles));
		
		//sensor_msgs::PointCloud2 vertices;
		//pcl::toROSMsg(*cloud_,vertices);
		Eigen::Array3f cell_size = kinfu->volume_box().getVoxelSize();
		Eigen::Array3f box_size = kinfu->volume_box().getSize();
		for (uint64 i = 0; i < (*triangles).size(); i++){
		  Eigen::Vector3d idx = (*triangles)[i].toEigen();
		
			for (int j = 0; j < 3; j++){
				pcl::PointNormal pt = (*cloud_)[idx(j)];
				//Eigen::Vector3d vtx(pt.x*cell_size(0), pt.y*cell_size(1), pt.z*cell_size(2));
				Eigen::Vector3d vtx(pt.x-box_size(0)/2.0, pt.y-box_size(1)/2.0, pt.z-box_size(2)/2.0);
				triangle_meshes.push_back(vtx);
			}
			/*
			for (int j = 0; j < 3; j++){
				pcl::PointNormal pt = (*cloud_)[idx(2-j)];
				//Eigen::Vector3d vtx(pt.x*cell_size(0), pt.y*cell_size(1), pt.z*cell_size(2));
				Eigen::Vector3d vtx(pt.x-box_size(0)/2.0, pt.y-box_size(1)/2.0, pt.z-box_size(2)/2.0);
				triangle_meshes.push_back(vtx);
			}
			*/
		}

		if(!silent) ROS_INFO("kinfu: Extract Mesh Worker complete.\n");
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool marchingCubesBox(double volume_size, pcl::gpu::kinfuLS::TsdfVolume& volume, std::vector<Mesh::Ptr> &output_meshes, bool silent) 
	{
		try
		{
		  //Creating the standalone marching cubes instance
		  if(!silent) std::cout << "Processing world with volume size set to " << volume_size << "meters\n";

	    pcl::gpu::kinfuLS::StandaloneMarchingCubes<pcl::PointXYZI> marching_cubes (pcl::device::kinfuLS::BOX_X, pcl::device::kinfuLS::BOX_Y, pcl::device::kinfuLS::BOX_Z, volume_size);

		  //output_meshes.push_back(marching_cubes.getMeshFromTSDFCloud(*cloud, silent));
			output_meshes.push_back(marching_cubes.getMeshFromTSDFVolume(volume, silent));

		  if(!silent) std::cout << "Marching cube done!\n";
		  return true;
		}
		catch (const pcl::PCLException& /*e*/) { PCL_ERROR ("PCLException... Exiting...\n"); return false; }
		catch (const std::bad_alloc& /*e*/) { PCL_ERROR ("Bad alloc... Exiting...\n"); return false; }
		catch (const std::exception& /*e*/) { PCL_ERROR ("Exception... Exiting...\n"); return false; }
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	template <class PointT>
	void separateMesh(Mesh::ConstPtr mesh, typename pcl::PointCloud<PointT>::Ptr points, TriangleVectorPtr triangles){
		// convert the point cloud
		pcl::fromPCLPointCloud2(mesh->cloud,*points);

		if (triangles)
		{
		  // convert the triangles
		  const uint mesh_size = mesh->polygons.size();
		  triangles->reserve(mesh_size);

		  for (uint triangle_i = 0; triangle_i < mesh_size; triangle_i++)
		  {
		    const pcl::Vertices &v = mesh->polygons[triangle_i];
		    if (v.vertices.size() != 3)
		    {
		      ROS_ERROR("WARNING: polygon %u has %u vertices, only triangles are supported.",triangle_i,uint(v.vertices.size()));
		      continue;
		    }

		    Triangle tri;

		    for (uint i = 0; i < 3; i++)
		      tri[i] = v.vertices[i];

		    triangles->push_back(tri);
		  }
		}
	}
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	template <class PointT>
	void mergePointCloudsAndMesh(std::vector<typename pcl::PointCloud<PointT>::Ptr> &pointclouds,
		typename pcl::PointCloud<PointT>::Ptr out_cloud,std::vector<TriangleVectorPtr> * meshes,TriangleVector * out_mesh)
	{
		uint offset = 0;
		const uint pointcloud_count = pointclouds.size();

		out_cloud->clear();

		if (out_mesh)
		  out_mesh->clear();

		for (uint pointcloud_i = 0; pointcloud_i < pointcloud_count; pointcloud_i++)
		{
		  const uint pointcloud_size = pointclouds[pointcloud_i]->size();

		  // copy the points
		  (*out_cloud) += *(pointclouds[pointcloud_i]);

		  if (out_mesh)
		  {
		    // copy the triangles, shifting vertex id by an offset
		    const uint mesh_size = (*meshes)[pointcloud_i]->size();
		    out_mesh->reserve(out_mesh->size() + mesh_size);

		    for (uint triangle_i = 0; triangle_i < mesh_size; triangle_i++)
		    {
		      Triangle tri;
		      const Triangle & v = (*(*meshes)[pointcloud_i])[triangle_i];

		      for (uint i = 0; i < 3; i++)
		        tri[i] = v[i] + offset;

		      out_mesh->push_back(tri);
		    }

		    offset += pointcloud_size;
		  }
		}
	}
};
