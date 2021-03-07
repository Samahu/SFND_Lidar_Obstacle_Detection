/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations
  	for (auto i = 0; i < maxIterations; ++i)
    {
		// Randomly sample subset and fit line
		auto p1_index = static_cast<int>(cloud->points.size() * ((double) (rand() - 1) / RAND_MAX));
      	auto p2_index = static_cast<int>(cloud->points.size() * ((double) (rand() - 1) / RAND_MAX));
      
      	if (p1_index == p2_index)
        {
          cout << "invalid sample!!";
          continue;
        }
      
      	auto p1 = cloud->points[p1_index];
      	auto x1 = p1.x;
      	auto y1 = p1.y;
      	auto p2 = cloud->points[p2_index];
      	auto x2 = p2.x;
      	auto y2 = p2.y;
      	auto A = y1 - y2;
      	auto B = x2 - x1;
      	auto C = x1 * y2 - x2 * y1;

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
      	std::unordered_set<int> currentInliers;
      	for (auto j = 0; j < cloud->points.size(); ++j)
        {
          auto p = cloud->points[j];
          
          auto x0 = p.x;
          auto y0 = p.y;
          auto d = fabs(A * x0 + B * y0 + C) / sqrt(A * A + B * B);

          if (d <= distanceTol)
          {
            currentInliers.insert(j);
          }
        }
      
      	if (currentInliers.size() > inliersResult.size())
          inliersResult = currentInliers;
    }

  	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
		auto p1_index = *(itr++);
      	auto p2_index = *(itr++);
      	auto p3_index = *(itr++);
      	cout << "selected: " << p1_index << ", " << p2_index << ", " << p3_index << endl;
      
      	auto p1 = cloud->points[p1_index];
      	auto p2 = cloud->points[p2_index];
      	auto p3 = cloud->points[p3_index];
      
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
      
      	cout << "currentInliers.size(): " << currentInliers.size() << " < " << inliersResult.size() << endl;
      	if (currentInliers.size() > inliersResult.size())
          inliersResult = currentInliers;
    }

  	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
