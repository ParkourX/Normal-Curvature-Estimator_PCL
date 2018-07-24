#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <fstream>
#include <string>
#include <tclap/CmdLine.h>
// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;


// Align a rigid object to a scene with clutter and occlusions
int main(int argc, char **argv)
{
	std::string sourcefile = "";
	std::string outputDir = "";
	float radius = 0.1f;
	float leaf = 0.005f;
	try {

		TCLAP::CmdLine cmd("Arash Cloud Library \n Command description message ", ' ', "0.1");
		TCLAP::ValueArg<std::string> sourceArg("s", "source_file", "Source", false, "None", "string");
		TCLAP::ValueArg<std::string> outputDirArg("o", "output_directory", "Output_Directory", false, "None", "string");
		TCLAP::ValueArg<float> radArg("r", "radius", "Radius", false, 0.1f, "float");
		TCLAP::ValueArg<float> leafArg("l", "leaf_size", "Leaf", false, 0.005f, "float");

		cmd.add(sourceArg);
		cmd.add(outputDirArg);
		cmd.add(radArg);
		cmd.add(leafArg);
		cmd.parse(argc, argv);

		sourcefile = sourceArg.getValue();
		outputDir = outputDirArg.getValue();
		radius = radArg.getValue();
		leaf = leafArg.getValue();
		pcl::console::print_highlight("Parameters processed succesfully ...\n");

	}
	catch (TCLAP::ArgException &e)  // catch any exceptions
	{
		std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
	}

	
	// Point clouds
	PointCloudT::Ptr object(new PointCloudT);
	PointCloudT::Ptr object_aligned(new PointCloudT);
	PointCloudT::Ptr scene(new PointCloudT);
	FeatureCloudT::Ptr object_features(new FeatureCloudT);
	FeatureCloudT::Ptr scene_features(new FeatureCloudT);


	// Load object and scene
	pcl::console::print_highlight("Loading point clouds...\n");
	std::ifstream file(sourcefile);
	std::string str;
	while (std::getline(file, str))
	{
		// Process str
		float x, y, z;
		sscanf(str.c_str(), "%f %f %f\n", &x, &y, &z);
		PointNT p;
	    p.x = x; p.y = y; p.z = z;
		scene->push_back(p);
	}

	//// Downsample
	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<PointNT> grid;
	//const float leaf = 0.05f;//0.005
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(scene);
	grid.filter(*scene);

	// Estimate normals for scene
	pcl::console::print_highlight("Estimating scene normals...\n");
	pcl::NormalEstimationOMP<PointNT, PointNT> nest;
	nest.setRadiusSearch(radius);
	nest.setViewPoint(0, 500, 0);
	nest.setInputCloud(scene);
	
	nest.compute(*scene);
	
	std::string fileNameWithoutExtension = sourcefile.substr(0, sourcefile.rfind("."));
	pcl::io::savePCDFileASCII<PointNT>(outputDir+"/"+ fileNameWithoutExtension+".pcd", *scene);
	pcl::console::print_highlight("Output has saved succesfully...\n");
	

	return (0);
}