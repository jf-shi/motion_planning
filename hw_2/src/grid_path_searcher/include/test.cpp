#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

//利用模板提供通用的交换函数
template<class T>
void mySwap(T& a, T& b)
{
T temp = a;
a = b;
b = temp;
}
// 1、自动类型推导，必须推导出一致的数据类型T,才可以使用
void test01()
{
int a = 10;
int b = 20;
char c = 'c';
mySwap(a, b); // 正确，可以推导出一致的T
//mySwap(a, c); // 错误，推导不出一致的T类型
}
// 2、模板必须要确定出T的数据类型，才可以使用
template<class T>
void func()
{
cout << "func 调用" << endl;
}
void test02()
{
//func(); //错误，模板不能独立使用，必须确定出T的类型
func<int>(); //利用显示指定类型的方式，给T一个类型，才可以使用该模板
}
int main() {
test01();
test02();
system("pause");
return 0;
}
#include <string>
//类模板
template<class NameType, class AgeType = int>
class Person
{
public:
Person(NameType name, AgeType age)
{
this->mName = name;
this->mAge = age;
}
void showPerson()
{
cout << "name: " << this->mName << " age: " << this->mAge << endl;
}
public:
NameType mName;
AgeType mAge;
};
//1、指定传入的类型
void printPerson1(Person<string, int> &p)
{
p.showPerson();
}
void test01()
{
Person <string, int >p("孙悟空", 100);
printPerson1(p);
typeid(p).name(); //输出类型名称
}