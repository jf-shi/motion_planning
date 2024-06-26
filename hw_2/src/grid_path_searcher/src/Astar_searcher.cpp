#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf; 
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;
//static_cast<int> 用于将 uint8_t 类型转换为 int 类型以便于输出，因为直接输出 uint8_t 类型的值可能会被解释为字符。因为 uint8_t 被解释为字符类型。
    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);  
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));   //这个函数将输入坐标舍入到最近的栅格中心点。它首先将世界坐标转换为栅格索引，然后再将栅格索引转换回世界坐标。
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}
//有效的 x 轴索引范围是从 0 到 GLX_SIZE - 1
inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    *
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    *
    *
    */
    if (currentPtr == nullptr)
    {
        ROS_ERROR("AstarPathFinder::AstarGetSucc: currentPtr is NULL");
        return;
    }
    GridNodePtr neighborPtr = nullptr;
    Eigen::Vector3i current_idx = currentPtr -> index;
    Eigen::Vector3d current_coord = currentPtr -> coord;
    Eigen::Vector3i neighbor_idx;
    Eigen::Vector3d neighbor_coord;
    double edge_cost = 0.0; //计算边的代价
    
    for(int i = -1; i <= 1; i++)
        for(int j = -1; j <= 1; j++)
            for(int k = -1; k <= 1; k++){
                
                
                if(i == 0 && j == 0 && k == 0) //跳过当前节点
                    continue;
                //跳过越界节点
                neighbor_idx = current_idx + Eigen::Vector3i(i,j,k);
                if (neighbor_idx(0) < 0 || neighbor_idx(0) >= GLX_SIZE || neighbor_idx(1) < 0 || neighbor_idx(1) >= GLY_SIZE 
                    || neighbor_idx(2) < 0 || neighbor_idx(2) >= GLZ_SIZE)
                    continue;
                
                if (isOccupied(neighbor_idx)) //跳过障碍物点
                    continue;

                
                neighborPtr = GridNodeMap[neighbor_idx(0)][neighbor_idx(1)][neighbor_idx(2)];
                if (neighborPtr == nullptr)
                    continue;
                
                if (neighborPtr -> id == -1)
                    continue;
                neighbor_coord = neighborPtr -> coord;
                edge_cost =  (neighbor_coord - current_coord).lpNorm<2>(); //计算边的代价
                edgeCostSets.push_back(edge_cost);
                neighborPtrSets.push_back(neighborPtr);

            }

}//


//注意要判断node1和node2是否存在
double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */
   enum class HeuristicType { Manhattan = 1,
                        Euclidean = 2, 
                        Diagonal = 3, 
                        Dijkstra = 4};
   HeuristicType heu_ = HeuristicType::Euclidean; //选择启发函数
   double heu_value = 0.0;
    if (!node1 || !node2)
    {
            ROS_ERROR("AstarPathFinder::getHeu: node1 or node2 is NULL");
            return numeric_limits<double>::infinity();
    }

    Eigen::Vector3d coord_node1 = node1 -> coord;
    Eigen::Vector3d coord_node2 = node2 -> coord;
    double cor_x = coord_node1(0) - coord_node2(0);
    double cor_y = coord_node1(1) - coord_node2(1);
    double cor_z = coord_node1(2) - coord_node2(2);
    //计算对角线距离的步长
    double D = 1.0;
    double D2 = sqrt(2.0);
    double D3 = sqrt(3.0);
   
    switch (heu_)
    {
        case HeuristicType::Manhattan:
            heu_value = abs(cor_x) + abs(cor_y) + abs(cor_z);
            break;
        case HeuristicType::Euclidean:
            heu_value = sqrt(cor_x * cor_x + cor_y * cor_y + cor_z * cor_z);
            break;
        case HeuristicType::Diagonal:

            heu_value =  D * (cor_x + cor_y + cor_z) +  (D2 - 2 * D) * min(min(cor_x, cor_y), cor_z) \
                    + (D3 - 3 * D) * min(max(min(cor_x, cor_y), min(cor_y, cor_z)), cor_x); 
    default: //默认计算欧式距离
            heu_value = sqrt(cor_x * cor_x + cor_y * cor_y + cor_z * cor_z);
        break;
    }

    // tie_breaker learned
    enum class Tie_breaker { tie_breaker_1 = 1, 
                      tie_breaker_2 = 2, 
                      tie_breaker_3 = 3};
    Tie_breaker tie_breaker_ = Tie_breaker::tie_breaker_1;//tie_breaker_1为上面的几种启发函数的tie_breaker，tie_breaker_2及_3为未来可能的实现
    double tiebreaker_p = 0.0;

    switch (tie_breaker_)
    {
        case Tie_breaker::tie_breaker_1:
            {if (heu_ == HeuristicType::Manhattan)
            {
                tiebreaker_p = Eigen::Vector3d(GLX_SIZE, GLY_SIZE, GLZ_SIZE).lpNorm<1>();
            }else if (heu_ == HeuristicType::Euclidean){
                tiebreaker_p = Eigen::Vector3d(GLX_SIZE, GLY_SIZE, GLZ_SIZE).lpNorm<2>();
            }else if (heu_ == HeuristicType::Diagonal){
                tiebreaker_p = D * (GLX_SIZE + GLY_SIZE + GLZ_SIZE) +  (D2 - 2 * D) * min(min(GLX_SIZE, GLY_SIZE), GLZ_SIZE) \
                    + (D3 - 3 * D) * min(max(min(GLX_SIZE, GLY_SIZE), min(GLY_SIZE, GLZ_SIZE)), GLX_SIZE); 
            }
            tiebreaker_p = D / tiebreaker_p;
            heu_value *= (1 + tiebreaker_p);
            break;
            }
        case Tie_breaker::tie_breaker_2: //未来可能的实现
            break;
        default: //默认计算Euclidean距离的tie_breaker
           
            break;
            
    }
    return heu_value;
}



void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = nullptr;
    GridNodePtr neighborPtr = nullptr;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]  = startPtr; //因为不是在地图取出来的，需要将start node加入地图中
    Eigen::Vector3i current_idx; //current的栅格索引
    double tem_fvalue = 0.0; //用于暂存节点的f值，与当前节点的f值比较，如果小于当前节点的f值，则更新当前节点的f值
    // this is the main loop
    while ( !openSet.empty() ){
        /*、
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
        currentPtr = openSet.begin()-> second;
        openSet.erase(openSet.begin());
        currentPtr -> id= -1;
        current_idx = currentPtr -> index;
        GridNodeMap[current_idx(0)][current_idx(1)][current_idx(2)] -> id = -1; //将当前节点加入closed set,表示已经被访问过了

        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }
        //get the succetion获得当前节点的邻居节点
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     

        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */ 
        //在这个循环里面，id只会为0/1，为-1的都是遍历完之后被erase掉了的节点
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            *        
            */
            
           
            
            neighborPtr = neighborPtrSets[i];
            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                neighborPtr -> gScore = currentPtr -> gScore + edgeCostSets[i];
                neighborPtr -> fScore = neighborPtr -> gScore + getHeu(neighborPtr, endPtr);
                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                neighborPtr -> id = 1; //add to open set
                neighborPtr -> cameFrom = neighborPtr;
                continue;
            }
            else if(neighborPtr -> id == 1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                tem_fvalue  = currentPtr -> gScore + edgeCostSets[i] + getHeu(neighborPtr, endPtr);
                if(tem_fvalue < neighborPtr -> fScore){
                    neighborPtr -> gScore = currentPtr -> gScore + edgeCostSets[i];
                    neighborPtr -> fScore = tem_fvalue;
                    neighborPtr -> cameFrom = currentPtr;
                }
                continue;
                
            }
            else{//this node is in closed set
                /*
                *
                please write your code below
                *        
                */
                
                continue;
            }
        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */
    auto currPtr = terminatePtr;
    while(currPtr != nullptr) {
        gridPath.push_back(currPtr);
        currPtr = currPtr -> cameFrom;
    }
    
    
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}