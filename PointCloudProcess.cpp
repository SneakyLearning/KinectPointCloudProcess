#include "pointCloudProcess.h"

void pointCloudProcess::removeOutlier(int meank=25,double threshold=0.1)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter;
	filter.setInputCloud(cloud);
	filter.setMeanK(25);
	filter.setStddevMulThresh(0.1);
	filter.filter(*cloud);
	pcl::visualization::PCLVisualizer viewer("simple");
	viewer.addPointCloud(cloud);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
	return;
}

void pointCloudProcess::drawWeldCloud(int maxiterations, double threshold)
{
	ModelCoefficients::Ptr cofficients(new ModelCoefficients());
	PointIndices::Ptr inliers(new PointIndices());
	SACSegmentation<PointXYZRGB> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(SACMODEL_PLANE);
	seg.setMethodType(SAC_RANSAC);
	seg.setMaxIterations(maxiterations);
	seg.setDistanceThreshold(threshold);
	ExtractIndices<PointXYZRGB> extract;
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *cofficients);
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud);
	pcl::visualization::PCLVisualizer viewer("simple");
	viewer.addPointCloud(cloud);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
	return;
}

void pointCloudProcess::donFilter()
{
	NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
	search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);
	tree->setInputCloud(cloud);
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setViewPoint(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
	PointCloud<PointNormal>::Ptr normals_small_scale(new PointCloud<PointNormal>), normals_large_scale(new PointCloud<PointNormal>);
	ne.setRadiusSearch(0.005);
	cout << "computing normals of small size" << endl;
	ne.compute(*normals_small_scale);
	ne.setRadiusSearch(1.0);
	cout << "computing normals of large size" << endl;
	ne.compute(*normals_large_scale);
	DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> don;
	don.setInputCloud(cloud);
	don.setNormalScaleLarge(normals_large_scale);
	don.setNormalScaleSmall(normals_small_scale);
	if (!don.initCompute())
	{
		std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
		exit(EXIT_FAILURE);
	}
	cout << "computing don" << endl;
	PointCloud<PointNormal>::Ptr doncloud(new pcl::PointCloud<PointNormal>);
	don.computeFeature(*doncloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Showing the difference of curvature of two scale"));
	visualization::PointCloudColorHandlerGenericField<PointNormal> handler_k(doncloud, "curvature");
	MView->setBackgroundColor(1, 1, 1);
	MView->addPointCloud(doncloud, handler_k);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5);
	MView->spin();
	ConditionOr<PointNormal>::Ptr range_cond(new ConditionOr<PointNormal>());
	range_cond->addComparison(FieldComparison<PointNormal>::ConstPtr(new FieldComparison<PointNormal>("curvature", ComparisonOps::LT, 0.1)));
	ConditionalRemoval<PointNormal> condrem;
	condrem.setCondition(range_cond);
	condrem.setInputCloud(doncloud);
	PointCloud<PointNormal>::Ptr doncloud_filtered(new PointCloud<PointNormal>);
	condrem.filter(*doncloud_filtered);
	doncloud = doncloud_filtered;
	//pcl::io::savePCDFileASCII("???", *doncloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView2(new pcl::visualization::PCLVisualizer("Showing the results of keeping relative small curvature points"));
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointNormal> handler_k2(doncloud, "curvature");
	MView2->setBackgroundColor(1, 1, 1);
	MView2->addPointCloud(doncloud, handler_k2);
	MView2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
	MView2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5);
	MView2->spin();
}

void pointCloudProcess::drawWeldLine()
{
	//ModelCoefficients::Ptr coefficents(new ModelCoefficients);
	//PointIndices::Ptr inliers(new PointIndices);
	//SACSegmentation<PointXYZ> seg;
	//seg.setOptimizeCoefficients(true);
	//seg.setModelType(SACMODEL_LINE);
	//seg.setMethodType(SAC_RANSAC);
	//seg.setDistanceThreshold(0.005);
	//seg.setInputCloud(cloud);
	//seg.segment(*inliers, *coefficents);
	//ExtractIndices<PointXYZ> extract;
	//extract.setInputCloud(cloud);
	//extract.setIndices(inliers);
	//extract.setNegative(false);
	//extract.filter(*cloud_filtered);
	//PointXYZ min, max;
	//getMinMax3D(*origin_max, min, max);
	//PointXYZ p1(((min.y - coefficents->values[1]) / coefficents->values[4] * coefficents->values[3]) + coefficents->values[0], min.y, ((min.y - coefficents->values[1]) / coefficents->values[4] * coefficents->values[5]) + coefficents->values[2]);
	//PointXYZ p2(((max.y - coefficents->values[1]) / coefficents->values[4] * coefficents->values[3]) + coefficents->values[0], max.y, ((max.y - coefficents->values[1]) / coefficents->values[4] * coefficents->values[5]) + coefficents->values[2]);
	//pcl::visualization::PCLVisualizer viewer("simple");
	//viewer.addPointCloud(origin);
	//viewer.addLine<PointXYZ>(p1, p2, 0, 1, 0, "line", 0);
	////viewer.addLine<PointXYZ>(min, max, 1, 0, 0, "line2", 0);
	//while (!viewer.wasStopped())
	//{
	//	viewer.spinOnce(100);
	//}
}
