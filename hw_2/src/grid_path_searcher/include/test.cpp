#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

int minSubArrayLen(int target, vector<int>& nums) {
	int minLen = INT_MAX; // 定义连续子数组最小值
	int i = 0; // 滑动窗口起始位置
	int j = 0; // 滑动窗口终止位置
	int sum = 0; // 用于统计连续子数组和并和target进行比较
	for (; j < nums.size(); j++) { // 终止位置移动
		sum += nums[j];
		while (sum >= target) { // 当和满足条件时
			minLen = min(minLen, j - i + 1); // 记录最小长度
			sum -= nums[i]; // 开始滑动窗口
			i++; // 起始位置移动
		}
	}
	return minLen == INT_MAX ? 0 : minLen;85
}

int main() {
	/*
		输入描述：
		第一行一个正整数，表示数组长度n
		第二行n个正整数，表示正整数数组内元素
		第三行一个正整数，表示条件值target
	*/
	int n;
	cout << "请输入数组长度：";	
	cin >> n;
	vector<int> nums(n);
	int target;
	for (int i = 0; i < n; i++) cin >> nums[i];
	cin >> target;

	cout << minSubArrayLen(target, nums) << endl;
	return 0;
}
