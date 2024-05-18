#include <iostream>
#include <vector>
#include <string>
#include <deque>

#include <algorithm>
#include <functional>
#include <numeric>
#include <map>
#include <set>
#include <list>
using namespace std;

void printList(const list<int>& L) {
for (list<int>::const_iterator it = L.begin(); it != L.end(); it++) {
cout << *it << " ";
}
cout << endl;
}
//交换
void test02()
{
list<int>L1;
L1.push_back(10);
L1.push_back(20);
L1.push_back(30);
L1.push_back(40);
list<int>L2;
L2.assign(10, 100);
cout << "交换前： " << endl;
printList(L1);
printList(L2);
cout << endl;
L1.swap(L2);
cout << "交换后： " << endl;
printList(L1);
printList(L2);
}
int main() {
//test01();
test02();
return 0;
}