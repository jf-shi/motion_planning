#include <iostream>
#include <vector>
using namespace std;

/*
int main(int argc, char *argv[]) {
    return 0;
}
*/
class Solution {
public:
int removeElement(vector <int>& nums, int val) {
    int slowerindex = 0;
    for(int fasterindex = 0; fasterindex < nums.size(); fasterindex++) {
        if (nums[fasterindex]!= val)
        {
            nums[slowerindex++] = nums[fasterindex];
            

        }
        
    }
    return slowerindex;
}
};

int main() {
    int n;
    cin >> n;
    vector <int> nums(n);
    for (int i = 0; i < n; i++) {
        cin >> nums[i];
    }
    int val;
    cin >> val;
    Solution s;
    int nums_size = s.removeElement(nums, val);
    cout << "nums[0]: " << nums_size << endl;
    
    return 0;
}