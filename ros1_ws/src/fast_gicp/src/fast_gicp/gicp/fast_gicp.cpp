#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/impl/fast_gicp_impl.hpp>

template class fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>;
template class fast_gicp::FastGICP<pcl::PointNormal, pcl::PointNormal>;
template class fast_gicp::FastGICP<pcl::PointXYZRGB, pcl::PointXYZRGB>;
// rosiwit local patch: SLAM 的 PointType = pcl::PointXYZINormal, 需要显式实例化 (2026-11-26)
template class fast_gicp::FastGICP<pcl::PointXYZINormal, pcl::PointXYZINormal>;
