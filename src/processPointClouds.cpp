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
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr CloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*CloudRegion);
    
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1.1, 1));
    roof.setMin(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(CloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices )
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(CloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*CloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return CloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obsCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for(int index : inliers->indices )
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obsCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obsCloud,planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    // Create the segmentation object
    /*pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);*/

    pcl::PointIndices::Ptr inliers = segment(cloud, maxIterations, distanceThreshold);

    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::segment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    pcl::PointIndices::Ptr inliersReturn (new pcl::PointIndices ());
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

  while(maxIterations--)
  {
    std::unordered_set<int> inliers;
    while(inliers.size()<3)
      inliers.insert(rand()%cloud->points.size());

    auto iterator = inliers.begin();

    float x1,y1,z1,x2,y2,z2,x3,y3,z3;

    x1 = cloud->points[*iterator].x;
    y1 = cloud->points[*iterator].y;
    z1 = cloud->points[*iterator].z;
    iterator++;
    x2 = cloud->points[*iterator].x;
    y2 = cloud->points[*iterator].y;
    z2 = cloud->points[*iterator].z;
    iterator++;
    x3 = cloud->points[*iterator].x;
    y3 = cloud->points[*iterator].y;
    z3 = cloud->points[*iterator].z;


    float coeffA = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
    float coeffB = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
    float coeffC = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
    float coeffD = -1*(coeffA*x1 + coeffB*y1 + coeffC*z1);
    for(int index = 0; index < cloud->points.size() ; index++)
    {
      if(inliers.count(index)>0)
        continue;

      float x4 = cloud->points[index].x;
      float y4 = cloud->points[index].y;
      float z4 = cloud->points[index].z;
      float distance = fabs(coeffA*x4 + coeffB*y4 + coeffC*z4 + coeffD)/sqrt(coeffA*coeffA + coeffB*coeffB + coeffC*coeffC);

      if(distance <= distanceTol)
        inliers.insert(index);
    }

    if(inliers.size() > inliersResult.size())
    {
      inliersResult = inliers;
    }
  }

  for(int index : inliersResult )
    inliersReturn->indices.push_back(index);
    
    return inliersReturn;

}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(uint iterator, typename pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr cluster,std::vector<bool>& isProcessed, KdTreeImpl* tree, float distanceTol)
{
  isProcessed[iterator] = true;
  cluster->indices.push_back(iterator);

  std::vector<int> nearestIds = tree->search({cloud->points[iterator].x,cloud->points[iterator].y,cloud->points[iterator].z},distanceTol);

  for(int id : nearestIds)
  {
    if(!isProcessed[id])
      clusterHelper(id, cloud, cluster, isProcessed, tree, distanceTol);
  }
}

template<typename PointT>
std::vector<pcl::PointIndices> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTreeImpl* tree, float distanceTol, int minSize, int maxSize)
{

    // TODO: Fill out this function to return list of indices for each cluster
  std::vector<bool> isProcessed(cloud->points.size(),false);
    
  std::vector<pcl::PointIndices> clusters;
 
  uint iterator=0;
  while(iterator<cloud->points.size())
  {
    if(isProcessed[iterator])
    {
      iterator++;
      continue;
    }
    pcl::PointIndices::Ptr cluster {new pcl::PointIndices};

    clusterHelper(iterator, cloud, cluster, isProcessed, tree, distanceTol);

    if((cluster->indices.size() > minSize) && (cluster->indices.size() < maxSize))
        clusters.push_back(*cluster);
    iterator++;

  }

    return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
/*
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    
*/   
    KdTreeImpl* tree = new KdTreeImpl();
  
    for (int i=0; i<cloud->points.size(); i++) 
        tree->insert({cloud->points[i].x,cloud->points[i].y,cloud->points[i].z},i);
   
    std::vector<pcl::PointIndices> cluster_indices = euclideanCluster(cloud, tree, clusterTolerance, minSize, maxSize);
    
    for(pcl::PointIndices getIndices : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for(int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

    clusters.push_back(cloudCluster);
    }

    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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