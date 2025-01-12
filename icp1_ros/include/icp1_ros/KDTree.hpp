#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <vector>
#include <nanoflann.hpp>

using PointCloud = std::vector<std::vector<double>>;

// Wrapper class for nanoflann to adapt a 2D PointCloud
class PointCloudAdaptor2D {
public:
    // Constructor accepting the point cloud (P)
    explicit PointCloudAdaptor2D(const PointCloud& points);

    // Must return the number of points in the cloud
    size_t kdtree_get_point_count() const;

    // Return the i-th dimension of the j-th point
    double kdtree_get_pt(size_t idx, size_t dim) const;

    // Optional bounding box (not used in this case)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const;

private:
    const PointCloud& points;  // Reference to the point cloud
};

// Function to find the closest points using k-d tree
std::vector<std::vector<double>> find_closest_points_kdtree_2d(
    const PointCloud& Q,
    const PointCloud& P
);


#endif // KDTREE_HPP
