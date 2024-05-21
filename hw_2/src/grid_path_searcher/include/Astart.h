#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"
#include "node.h"
using namespace std;
class AstarPathFinder
{	
	private:

	protected:
		//uint8_t * data; //按照Z->Y->X的顺序,构建一维点云栅格数据存储格式
		//GridNodePtr *** GridNodeMap; //构建3维数组，用于存储Astar算法的节点信息
        /*data 用于存储栅格地图中的每个栅格单元的状态（例如，是否被占据）。通过这个一维向量，可以高效地管理和访问三维栅格地图中的数据。
        GridNodeMap：一个三维的GridNodePtr向量，表示网格地图中的所有节点。
        每个节点是一个指向GridNode结构体的指针，用于存储A*搜索算法的节点信息。*/

		/*线性化索引
		为了将三维栅格坐标 (x, y, z) 映射到一维向量中的索引，需要使用特定的公式：
		int index = x * GLYZ_SIZE + y * GLZ_SIZE + z;*/
        vector<int> data;
        vector<vector<vector<GridNodePtr>>> GridNodeMap;

		Eigen::Vector3i goalIdx;  //目标点的栅格坐标


		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;//栅格坐标系(与世界坐标系相差分辨率)下，整个点云地图的宽、长、高

	/*GLXYZ_SIZE：整个地图的总栅格数量（通常为GLX_SIZE * GLY_SIZE * GLZ_SIZE）。
			GLYZ_SIZE：地图在Y和Z方向的栅格数量乘积（通常为GLY_SIZE * GLZ_SIZE），用于快速计算三维索引的一部分。*/ 
		int GLXYZ_SIZE, GLYZ_SIZE;
		
		double resolution, inv_resolution; //resolution：地图的分辨率，即每个栅格的边长。inv_resolution：分辨率的倒数，用于坐标转换。
		double gl_xl, gl_yl, gl_zl;  //表示/世界坐标系下点云地图在 X, Y, Z 方向上的下边界坐标。
		double gl_xu, gl_yu, gl_zu;   //表示地图在 X, Y, Z 方向上的上边界坐标。

    /*在路径搜索完成后，terminatePtr 会指向目标节点（终止节点）。
		通过 terminatePtr，可以从目标节点回溯到起始节点，以构建完整的路径。*/
		GridNodePtr terminatePtr;
		std::multimap<double, GridNodePtr> openSet;

		double getHeu(GridNodePtr node1, GridNodePtr node2); // 计算启发式代价函数。
		void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);	//	获取当前节点的所有邻居节点。

    	bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const; //判断某个位置是否被占据
		bool isOccupied(const Eigen::Vector3i & index) const; //判断某个栅格坐标是否被占据。
		bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const; //判断某个位置是否为空闲。
		bool isFree(const Eigen::Vector3i & index) const; // 判断某个栅格坐标是否为空闲。
		
		Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index); //将栅格索引转换为实际坐标。
		Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);  //实际坐标转换为栅格索引。

	public:
		AstarPathFinder(){};
		~AstarPathFinder(){};
		void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt); // A*搜索算法的主函数。
		void resetGrid(GridNodePtr ptr);  //重置单个网格节点。
		void resetUsedGrids(); //重置所有已使用的网格节点。

		void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id); //初始化网格地图。
		void setObs(const double coord_x, const double coord_y, const double coord_z); //设置障碍物。

		Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);  //坐标取整。
		std::vector<Eigen::Vector3d> getPath();  // 获取计算出的路径。
		std::vector<Eigen::Vector3d> getVisitedNodes();  //获取所有访问过的节点。
};

#endif