
## C++ Learn Note

- string对象不能直接 cout, 必须#include< string >才能cout，或转成char*型输出  char* b = (char *)a.c_str()
- 数组不能直接用=赋值 ， 用strcpy（a,b）
- string中 c_str()包含'\0' data()可不包含'\0'
- const char* 与 char* const 区别 ：https://www.cnblogs.com/belfuture/p/5862110.html  
                                    https://blog.csdn.net/swibyn/article/details/20052371
- 1.加了const的成员函数可以被非const对象和const对象调用，但不加const的成员函数只能被非const对象调用<br>
  2.const对象不可调用非const成员函数
- 为什么虚函数不能加static？<br>
   因为static函数不需要对象来调用，可通过类名直接调用，所有没有隐藏this指针（在编译时绑定），而虚函数(在链接时绑定)需要对象来调用，隐藏了this指针。所以不能虚函数不能加static。
- 没有虚函数的类不适合做基类。
- 并发是一种能力，时间片轮转和并行是实现并发的方法
