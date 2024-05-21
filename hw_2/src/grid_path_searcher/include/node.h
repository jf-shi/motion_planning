#ifndef _NODE_H_
#define _NODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"

#define inf 1>>20  //定义一个非常大的数值，用于初始化代价值
struct GridNode;
typedef GridNode* GridNodePtr;

// 定义节点结构体
struct GridNode
{
    int id; // 节点状态，1表示在开放列表中，-1表示在封闭列表中 1--> open set, -1 --> closed set
    Eigen::Vector3d coord; // 节点的实际坐标
    Eigen::Vector3i dir; // 节点的扩展方向
    Eigen::Vector3i index; // 节点的栅格索引
    
    double gScore, fScore; // gScore是从起点到当前节点的代价，fScore是估计的总代价
    GridNodePtr cameFrom; // 指向上一个节点的指针
    std::multimap<double, GridNodePtr>::iterator nodeMapIt; // 节点在开放列表中的迭代器

    // 构造函数，初始化节点的索引和坐标
    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord)
    {
        id = 0; // 初始状态为未处理
        index = _index;
        coord = _coord;
        dir = Eigen::Vector3i::Zero(); // 初始方向为零向量

        gScore = inf; // 初始代价为无穷大
        fScore = inf; // 初始总代价为无穷大
        cameFrom = NULL; // 初始上一个节点为空
    }

    // 默认构造函数
    GridNode() {};

    // 析构函数
    ~GridNode() {};
};


#endif
