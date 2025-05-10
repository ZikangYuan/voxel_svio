#pragma once

// c++ include
#include <iostream>
#include <cmath>

// lib include
#include <Eigen/Core>
#include <tsl/robin_map.h>

// function include
#include "utility.h"
#include "state.h"
#include "frame.h"

class mapPoint : public vec
{
public:

  mapPoint();

  size_t feature_id;

  int unique_camera_id = -1;

  int anchor_cam_id = -1;

  double anchor_clone_timestamp = -1;

  bool has_had_anchor_change = false;

  bool should_marg = false;

  int update_fail_count = 0;

  Eigen::Vector3d uv_normalized;

  Eigen::Vector3d uv_normalized_fej;

  float color;

  std::shared_ptr<frame> host_frame;

  short voxel_idx[3];

  void update(const Eigen::VectorXd &dx) override;

  Eigen::Matrix<double, 3, 1> getPointXYZ(bool get_fej);

  void setPointXYZ(Eigen::Matrix<double, 3, 1> p_FinG, bool is_fej);
};

struct voxel {

    voxel() = default;

    voxel(short x, short y, short z) : x(x), y(y), z(z) {}

    bool operator==(const voxel &vox) const { return x == vox.x && y == vox.y && z == vox.z; }

    inline bool operator<(const voxel &vox) const {
        return x < vox.x || (x == vox.x && y < vox.y) || (x == vox.x && y == vox.y && z < vox.z);
    }

    inline static voxel coordinates(const Eigen::Vector3d &point, double voxel_size) {
        return {short(point.x() / voxel_size),
                short(point.y() / voxel_size),
                short(point.z() / voxel_size)};
    }

    short x;
    short y;
    short z;
};

struct voxelBlock {

    explicit voxelBlock(int num_points_ = 20) : num_points(num_points_) { points.reserve(num_points_); }

    std::vector<std::shared_ptr<mapPoint>> points;

    bool IsFull() const { return num_points <= points.size(); }

    void AddPoint(std::shared_ptr<mapPoint> map_point_ptr)
    {
        points.push_back(map_point_ptr);
    }

    inline int NumPoints() const { return points.size(); }

    inline int Capacity() { return num_points; }

    double last_visit_time = -1;

private:
    int num_points;
};

typedef tsl::robin_map<voxel, voxelBlock> voxelHashMap;

namespace std {

    template<> struct hash<voxel> {
        std::size_t operator()(const voxel &vox) const
        {
            const size_t kP1 = 73856093;
            const size_t kP2 = 19349669;
            const size_t kP3 = 83492791;
            return vox.x * kP1 + vox.y * kP2 + vox.z * kP3;
        }
    };
}