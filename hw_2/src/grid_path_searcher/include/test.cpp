#include <iostream>
#include <vector>
#include <queue>

using namespace std;

// 定义四个方向
int dir[4][2] = { {0, 1}, {1, 0}, {0, -1}, {-1, 0} };

// BFS函数
void bfs(vector<vector<char>>& grid, vector<vector<bool>>& visited, int x, int y) {
    queue<pair<int, int>> que; // 定义队列
    que.push({x, y}); // 起始节点加入队列
    visited[x][y] = true; // 只要加入队列，立刻标记为访问过的节点
    while (!que.empty()) { // 开始遍历队列里的元素
        pair<int, int> cur = que.front(); que.pop(); // 从队列取元素
        int curx = cur.first;
        int cury = cur.second; // 当前节点坐标
        for (int i = 0; i < 4; i++) { // 开始向当前节点的四个方向遍历
            int nextx = curx + dir[i][0];
            int nexty = cury + dir[i][1]; // 获取周边四个方向的坐标
            if (nextx < 0 || nextx >= grid.size() || nexty < 0 || nexty >= grid[0].size()) continue;  // 坐标越界了，直接跳过
            if (!visited[nextx][nexty] && grid[nextx][nexty] == '1') { // 如果节点没被访问过且是'1'
                que.push({nextx, nexty});  // 队列添加该节点为下一轮要遍历的节点
                visited[nextx][nexty] = true; // 只要加入队列立刻标记，避免重复访问
            }
        }
    }
}

int main() {
    // 定义grid地图
    vector<vector<char>> grid = {
        {'1', '1', '0', '0', '0'},
        {'1', '1', '0', '0', '0'},
        {'0', '0', '1', '0', '0'},
        {'0', '0', '0', '1', '1'}
    };

    // 初始化visited数组，所有元素初始为false
    vector<vector<bool>> visited(grid.size(), vector<bool>(grid[0].size(), false));

    // 调用bfs函数从(0, 0)开始
    bfs(grid, visited, 0, 0);

    // 输出visited数组，查看哪些节点被访问过
    for (int i = 0; i < visited.size(); ++i) {
        for (int j = 0; j < visited[i].size(); ++j) {
            cout << visited[i][j] << " ";
        }
        cout << endl;
    }

    return 0;
}


