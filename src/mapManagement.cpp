#include "mapManagement.h"

bool mapManagement::addPointToVoxel(voxelHashMap &voxel_map, std::shared_ptr<mapPoint> map_point_ptr, double voxel_size, int max_num_points_in_voxel, 
	double min_distance_points, pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_history)
{
	Eigen::Vector3d point = map_point_ptr->getPointXYZ(false);

	short kx = static_cast<short>(point[0] / voxel_size);
	short ky = static_cast<short>(point[1] / voxel_size);
	short kz = static_cast<short>(point[2] / voxel_size);

	map_point_ptr->voxel_idx[0] = kx;
	map_point_ptr->voxel_idx[1] = ky;
	map_point_ptr->voxel_idx[2] = kz;

	voxelHashMap::iterator search = voxel_map.find(voxel(kx, ky, kz));

	if(search != voxel_map.end())
	{
		auto &voxel_block = (search.value());

		if(!voxel_block.IsFull())
		{
			double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
			for (int i(0); i < voxel_block.NumPoints(); ++i)
			{
				auto &map_point_ptr_ = voxel_block.points[i];
				double sq_dist = (map_point_ptr_->getPointXYZ(false) - point).squaredNorm();

				if (sq_dist < sq_dist_min_to_points)
				{
					sq_dist_min_to_points = sq_dist;
				}
			}
			if(sq_dist_min_to_points > (min_distance_points * min_distance_points))
			{
				voxel_block.AddPoint(map_point_ptr);
				return true;
			}
			else
			{
				return false;
			}
		}
	}
	else
	{
		voxelBlock block(max_num_points_in_voxel);
		block.AddPoint(map_point_ptr);
		voxel_map[voxel(kx, ky, kz)] = std::move(block);

		// display
        pcl::PointXYZI point_temp;

        point_temp.x = kx >= 0 ? (kx + 0.5) * voxel_size : (kx - 0.5) * voxel_size;
        point_temp.y = ky >= 0 ? (ky + 0.5) * voxel_size : (ky - 0.5) * voxel_size;
        point_temp.z = kz >= 0 ? (ky + 0.5) * voxel_size : (kz - 0.5) * voxel_size;
        point_temp.intensity = 1;

        voxels_history->points.push_back(point_temp);
        // display

		return true;
    }
}

void mapManagement::changeHostVoxel(voxelHashMap &voxel_map, std::shared_ptr<mapPoint> map_point_ptr, double voxel_size, int max_num_points_in_voxel, 
	double min_distance_points, pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_history)
{
	Eigen::Vector3d point = map_point_ptr->getPointXYZ(false);

	short kx = static_cast<short>(point[0] / voxel_size);
	short ky = static_cast<short>(point[1] / voxel_size);
	short kz = static_cast<short>(point[2] / voxel_size);

	if (kx == map_point_ptr->voxel_idx[0] && ky == map_point_ptr->voxel_idx[1] && kz == map_point_ptr->voxel_idx[2]) return;

	voxelHashMap::iterator search_old = voxel_map.find(voxel(map_point_ptr->voxel_idx[0], map_point_ptr->voxel_idx[1], map_point_ptr->voxel_idx[2]));

	auto &voxel_block_old = (search_old.value());
	auto it_old = voxel_block_old.points.begin();

	bool has_found = false;

	while (it_old != voxel_block_old.points.end())
	{
		if ((*it_old) == map_point_ptr)
		{
			has_found = true;
			it_old = voxel_block_old.points.erase(it_old);
			break;
		}

		it_old++;
	}

	assert(has_found);

	map_point_ptr->voxel_idx[0] = kx;
	map_point_ptr->voxel_idx[1] = ky;
	map_point_ptr->voxel_idx[2] = kz;

	voxelHashMap::iterator search_new = voxel_map.find(voxel(kx, ky, kz));

	if(search_new != voxel_map.end())
	{
		auto &voxel_block_new = (search_new.value());
		voxel_block_new.AddPoint(map_point_ptr);
	}
	else
	{
		voxelBlock block_temp(max_num_points_in_voxel);
		block_temp.AddPoint(map_point_ptr);
		voxel_map[voxel(kx, ky, kz)] = std::move(block_temp);

		// display
        pcl::PointXYZI point_temp;

        point_temp.x = kx >= 0 ? (kx + 0.5) * voxel_size : (kx - 0.5) * voxel_size;
        point_temp.y = ky >= 0 ? (ky + 0.5) * voxel_size : (ky - 0.5) * voxel_size;
        point_temp.z = kz >= 0 ? (ky + 0.5) * voxel_size : (kz - 0.5) * voxel_size;
        point_temp.intensity = 1;

        voxels_history->points.push_back(point_temp);
        // display
	}
}

void mapManagement::deleteFromVoxel(voxelHashMap &voxel_map, std::shared_ptr<mapPoint> map_point_ptr)
{
	voxelHashMap::iterator search = voxel_map.find(voxel(map_point_ptr->voxel_idx[0], map_point_ptr->voxel_idx[1], map_point_ptr->voxel_idx[2]));

	auto &voxel_block = (search.value());
	auto it = voxel_block.points.begin();

	bool has_found = false;

	while (it != voxel_block.points.end())
	{
		if (*it == map_point_ptr)
		{
			has_found = true;
			it = voxel_block.points.erase(it);
			break;
		}

		it++;
	}

	assert(has_found);
}