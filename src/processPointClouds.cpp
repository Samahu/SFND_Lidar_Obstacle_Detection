// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);


    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> region(true);
    region.setInputCloud(cloud_filtered);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.filter(*cloud_region);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setInputCloud(cloud_region);
    roof.setMin(Eigen::Vector4f( -1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (auto i : indices)
      inliers->indices.push_back(i);

    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud (cloud_region);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter(*cloud_region);
  

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstaclesCloud(new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter(*planeCloud);
  
  extract.setNegative (true);
  extract.filter(*obstaclesCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(
    obstaclesCloud, planeCloud);
  return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RANSAC(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations
  	for (auto i = 0; i < maxIterations; ++i)
    {
		// Randomly sample subset and fit line
      	std::unordered_set<int> subset;
      	while (subset.size() < 3)
          subset.insert(rand() % cloud->points.size());
      
      	auto itr = subset.begin();
      
      	auto p1 = cloud->points[*(itr++)];
      	auto p2 = cloud->points[*(itr++)];
      	auto p3 = cloud->points[*(itr++)];
      
      	Eigen::Vector3f v1 = p1.getArray3fMap();
      	Eigen::Vector3f v2 = p2.getArray3fMap();
      	Eigen::Vector3f v3 = p3.getArray3fMap();
      
      	auto v1_v2 = v2 - v1;
      	auto v1_v3 = v3 - v1;
      	auto c = v1_v2.cross(v1_v3);
      
      	auto A = c.x();
		auto B = c.y();
		auto C = c.z();
		auto D = -( c.x() * p1.x + c.y() * p1.y + c.z() * p1.z );
      
      	// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
      	std::unordered_set<int> currentInliers;
      	for (auto j = 0; j < cloud->points.size(); ++j)
        {
          auto p = cloud->points[j];
          auto d = fabs(A*p.x+B*p.y+C*p.z+D) / sqrt(A*A+B*B+C*C);
                    
          if (d <= distanceTol)
            currentInliers.insert(j);
        }
      
      	if (currentInliers.size() > inliersResult.size())
          inliersResult = currentInliers;
    }

  	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(
  	typename pcl::PointCloud<PointT>::Ptr cloud,
  	int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory  
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    // seg.segment (*inliers, *coefficients);
  
  	auto indices = RANSAC(cloud, maxIterations, distanceThreshold);
  	for (auto i : indices)
      inliers->indices.push_back(i);
  
    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    }
  
  	/*
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
     */

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    auto segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
  typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new typename pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
  
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new typename pcl::PointCloud<PointT>);
        for (auto pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back ((*cloud)[*pit]);
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count()
      << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}