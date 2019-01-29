#include "../PointCloudTool/PointCloudToolInterface.h"
#include <cstdlib>
#include <iostream>

int main(int argc, char *argv[])
{
	std::cout << "begin" << std::endl;
	try
	{
		system("rmdir /s/q  C:\\Users\\wang\\Desktop\\北航项目程序打包\\数据\\测试数据\\济宁数据\\las\\2-1");
	}
	catch (...)
	{
		std::cout << "rmdir error!"  << std::endl;
	}
	
	PointCloudToolInterface ins;
	ins.check_distance_towers("C:\\Users\\wang\\Desktop\\北航项目程序打包\\数据\\测试数据\\济宁数据\\las\\2-1.las", 
		"", 
		"C:\\Users\\wang\\Desktop\\北航项目程序打包\\数据\\测试数据\\济宁数据\\待纠偏数据"
		,"C:\\Users\\wang\\Desktop\\北航项目程序打包\\数据\\测试数据\\济宁数据\\classdir\\config.xml", 
		50, false);


	std::cout << "end" << std::endl;
	return 0;
}