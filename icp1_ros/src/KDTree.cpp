#include "KDTree.hpp"
#include <nanoflann.hpp>
#include <cmath>
#include <limits>

// Constructor to initialize with the point cloud (P)
PointCloudAdaptor2D::PointCloudAdaptor2D(const PointCloud& points)
    : points(points) {}

// Returns the number of points in the cloud
size_t PointCloudAdaptor2D::kdtree_get_point_count() const {
    return points.size();
}

// Returns the i-th dimension of the j-th point (2D: x, y)
double PointCloudAdaptor2D::kdtree_get_pt(size_t idx, size_t dim) const {
    return points[idx][dim];
}

// Optional bounding box (not used in this case)
template <class BBOX>
bool PointCloudAdaptor2D::kdtree_get_bbox(BBOX&) const {
    return false;
}

// Function to find the closest points using k-d tree
std::vector<std::vector<double>> find_closest_points_kdtree_2d(
    const PointCloud& Q,
    const PointCloud& P
) {
    std::vector<std::vector<double>> closest_points;

    // Build k-d tree
    PointCloudAdaptor2D adaptor(P);
    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, PointCloudAdaptor2D>,
        PointCloudAdaptor2D,
        2 // Fixed dimension: 2D
    >;

    KDTree kdtree(2, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kdtree.buildIndex();

    // Search for the closest point in P for each point in Q
    for (const auto& q : Q) {
        size_t closest_idx;
        double min_dist_sq;
        nanoflann::KNNResultSet<double> resultSet(1); // Nearest neighbor
        resultSet.init(&closest_idx, &min_dist_sq);
        kdtree.findNeighbors(resultSet, q.data(), nanoflann::SearchParameters(10));

        closest_points.push_back(P[closest_idx]);
    }

    return closest_points;
}
