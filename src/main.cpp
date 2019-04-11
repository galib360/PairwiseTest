#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/moment_of_inertia_estimation.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
using namespace std;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// This is a tutorial so we can afford having global variables
//our visualizer
pcl::visualization::PCLVisualizer *p;
//its left and right viewports
int vp_1, vp_2, vp_3, vp_4;

//convenient structure to handle our pointclouds
struct PCD {
	PointCloud::Ptr cloud;
	std::string f_name;

	PCD() :
			cloud(new PointCloud) {
	}
	;
};

struct PCDComparator {
	bool operator ()(const PCD& p1, const PCD& p2) {
		return (p1.f_name < p2.f_name);
	}
};

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation: public pcl::PointRepresentation<PointNormalT> {
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
	MyPointRepresentation() {
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PointNormalT &p, float * out) const {
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target,
		const PointCloud::Ptr cloud_source) {
	p->removePointCloud("vp1_target");
	p->removePointCloud("vp1_source");

	PointCloudColorHandlerCustom<PointT> tgt_h(cloud_target, 0, 255, 0);
	PointCloudColorHandlerCustom<PointT> src_h(cloud_source, 255, 0, 0);
	p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
	p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);

	PCL_INFO("Press q to begin the registration.\n");
	p->spin();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target,
		const PointCloudWithNormals::Ptr cloud_source) {
	p->removePointCloud("source");
	p->removePointCloud("target");

	PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(
			cloud_target, "curvature");
	if (!tgt_color_handler.isCapable())
		PCL_WARN("Cannot create curvature color handler!");

	PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(
			cloud_source, "curvature");
	if (!src_color_handler.isCapable())
		PCL_WARN("Cannot create curvature color handler!");

	p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
	p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);

	p->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
 * \param argc the number of arguments (pass from main ())
 * \param argv the actual command line arguments (pass from main ())
 * \param models the resultant vector of point cloud datasets
 */
void loadData(string scene, string model,
		std::vector<PCD, Eigen::aligned_allocator<PCD> > &models) {
	std::string extension(".pcd");
	// Suppose the first argument is the actual test model
//  for (int i = 1; i < argc; i++)
//  {
	std::string fname = scene;
	std::string fname2 = model;
	// Needs to be at least 5: .plot

	std::transform(fname.begin(), fname.end(), fname.begin(),
			(int (*)(int))tolower);std
	::transform(fname2.begin(), fname2.end(), fname2.begin(),
			(int (*)(int))tolower);

			//check that the argument is a pcd file
if(	fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
	{
		// Load the cloud and saves it into the global list of models
		PCD m;
		m.f_name = scene;
		pcl::io::loadPCDFile (scene, *m.cloud);
		//remove NAN points from the cloud
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

		models.push_back (m);
	}
	if (fname2.compare(fname2.size() - extension.size(), extension.size(),
			extension) == 0) {
		// Load the cloud and saves it into the global list of models
		PCD m2;
		m2.f_name = model;
		pcl::io::loadPCDFile(model, *m2.cloud);
		//remove NAN points from the cloud
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*m2.cloud, *m2.cloud, indices);

		models.push_back(m2);
	}
	//}
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
 * \param cloud_src the source PointCloud
 * \param cloud_tgt the target PointCloud
 * \param output the resultant aligned source PointCloud
 * \param final_transform the resultant transform between source and target
 */
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt,
		PointCloud::Ptr output, Eigen::Matrix4f &final_transform,
		bool downsample) {
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);
	pcl::VoxelGrid<PointT> grid;

//  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
//  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
//  pcl::io::loadPCDFile ("dataRoom/wholeroom.pcd", *cloud);
//  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//  sor.setLeafSize (0.4f, 0.4f, 0.4f);
//  sor.setInputCloud (cloud);
//  sor.filter (*cloud_filtered);

	Eigen::Vector3i srcBBmax;
	Eigen::Vector3i srcBBmin;
	Eigen::Vector3i tgtBBmax;
	Eigen::Vector3i tgtBBmin;

	if (downsample) {
		grid.setLeafSize(0.4, 0.4, 0.4);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);
		srcBBmax = grid.getMaxBoxCoordinates();
		srcBBmin = grid.getMinBoxCoordinates();

		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
		tgtBBmax = grid.getMaxBoxCoordinates();
		tgtBBmin = grid.getMinBoxCoordinates();
	} else {
		src = cloud_src;
		tgt = cloud_tgt;
	}
	//cout<<tgtBBmax[0]<<endl;

	// Compute surface normals and curvature
	PointCloudWithNormals::Ptr points_with_normals_src(
			new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(
			new PointCloudWithNormals);

	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);

	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src);

	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

	//BB
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(tgt);
	feature_extractor.compute();

	std::vector<float> moment_of_inertia;
	std::vector<float> eccentricity;
	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

	feature_extractor.getMomentOfInertia(moment_of_inertia);
	feature_extractor.getEccentricity(eccentricity);
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB,
			rotational_matrix_OBB);
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors(major_vector, middle_vector,
			minor_vector);
	feature_extractor.getMassCenter(mass_center);

	//Visualize sampled PointCloud
	p->removePointCloud("subsource");
	p->removePointCloud("subtarget");
	PointCloudColorHandlerCustom<PointT> subcloud_tgt_h(tgt, 255, 0, 0); //changed color
	PointCloudColorHandlerCustom<PointT> subcloud_src_h(src, 0, 255, 0);
	//PointCloudColorHandlerCustom<PointT> grid_h (grid, 0, 0, 255);

	p->addPointCloud(tgt, subcloud_tgt_h, "subtarget", vp_3);
	p->addPointCloud(src, subcloud_src_h, "subsource", vp_4);

	//BB visu
	p->addCube (tgtBBmin[0], tgtBBmax[0], tgtBBmin[1], tgtBBmax[1], tgtBBmin[2], tgtBBmax[2], 1.0, 1.0, 1.0, "tgtBB", vp_3);
	p->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "tgtBB", vp_3);

	p->addCube (srcBBmin[0], srcBBmax[0], srcBBmin[1], srcBBmax[1], srcBBmin[2], srcBBmax[2], 1.0, 1.0, 1.0, "srcBB", vp_4);
	p->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "srcBB", vp_4);

//	p->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 1.0, "AABB", vp_3);
//	p->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB", vp_3);

//	int cubeID =0;
//	for(float startx = min_point_AABB.x; startx< max_point_AABB.x; startx+=0.4){
//		for(float starty = min_point_AABB.x; starty< max_point_AABB.x; starty+=0.4){
//			for(float startz = min_point_AABB.x; startz< max_point_AABB.x; startz+=0.4){
//				p->addCube (startx, startx+0.4, starty, starty+0.4, startz, startz+0.4, 1.0, 1.0, 1.0, to_string(cubeID), vp_3);
//				p->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, to_string(cubeID), vp_3);
//				cubeID++;
//			}
//		}
//	}
	//p->addCube()

	//
	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	//
	// Align
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	reg.setTransformationEpsilon(1e-6);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(1.5);
	// Set the point representation
	reg.setPointRepresentation(
			boost::make_shared<const MyPointRepresentation>(
					point_representation));

	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);

	//
	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations(3);
	for (int i = 0; i < 200; ++i) {
		PCL_INFO("Iteration Nr. %d.\n", i);

		// save cloud for visualization purpose
		points_with_normals_src = reg_result;

		// Estimate
		reg.setInputSource(points_with_normals_src);
		reg.align(*reg_result);

		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation() * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum())
				< reg.getTransformationEpsilon())
			reg.setMaxCorrespondenceDistance(
					reg.getMaxCorrespondenceDistance() - 0.001);

		prev = reg.getLastIncrementalTransformation();

		// visualize current state
		showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}

	//
	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	//
	// Transform target back in source frame
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

	p->removePointCloud("source");
	p->removePointCloud("target");

	PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 255, 0, 0); //changed color
	PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 0, 255, 0);
	p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
	p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);

	PCL_INFO("Press q to continue the registration.\n");
	p->spin();

	p->removePointCloud("source");
	p->removePointCloud("target");

	//add the source to the transformed target
	*output += *cloud_src;

	final_transform = targetToSource;
}

/* ---[ */
int main() {
	// Load data
	std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
	string scene = "dataRoom/wholeroom.pcd";  //target
	string model = "dataRoom/partroom.pcd";  //source
	loadData(model, scene, data);
//  loadData (scene, model, data);

	// Check user input
//  if (data.empty ())
//  {
//    PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
//    PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
//    return (-1);
//  }
	PCL_INFO("Loaded %d datasets.", (int )data.size());

	// Create a PCLVisualizer object
	p = new pcl::visualization::PCLVisualizer(
			"Pairwise Incremental Registration example");
	p->createViewPort(0.0, 0, 0.5, 0.5, vp_1);
	p->createViewPort(0.5, 0, 1.0, 0.5, vp_2);
	p->createViewPort(0.0, 0.5, 0.5, 1.0, vp_3);
	p->createViewPort(0.5, 0.5, 1.0, 1.0, vp_4);

	PointCloud::Ptr result(new PointCloud), source, target;
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(),
			pairTransform;

	for (size_t i = 1; i < data.size(); ++i) {
		source = data[i - 1].cloud;
		target = data[i].cloud;

		// Add visualization data
		showCloudsLeft(source, target);

		PointCloud::Ptr temp(new PointCloud);
		PCL_INFO("Aligning %s (%d) with %s (%d).\n", data[i - 1].f_name.c_str(),
				source->points.size(), data[i].f_name.c_str(),
				target->points.size());
		pairAlign(source, target, temp, pairTransform, true);

		//transform current pair into the global transform
		pcl::transformPointCloud(*temp, *result, GlobalTransform);

		//update the global transform
		GlobalTransform = GlobalTransform * pairTransform;
		cout << "pairwise transform: " << endl;
		cout << pairTransform << endl;
		cout << "Global Transform: " << endl;
		cout << GlobalTransform << endl;

		//save aligned pair, transformed into the first cloud's frame
		std::stringstream ss;
		ss << i << ".pcd";
		pcl::io::savePCDFile(ss.str(), *result, true);

	}
}
