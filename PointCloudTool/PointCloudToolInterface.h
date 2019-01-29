#pragma once

#ifdef DLL_EXPORTS
#define DLL_API __declspec(dllexport)
#else
#define DLL_API __declspec(dllimport)
#endif

enum ERRORNUM
{
	NORMAL = 0,			// 正常
	LOADSETTING = 1,	// 加载配置文件失败
	LASFILE = 2,	    // 读取las文件失败
	OUTDIR = 3,			// outdir读取失败
	EXCELDIR = 4,		// 铁塔纠偏目录不存在
	TRAINFILE = 5,		// 训练文件加载失败
	PDFEXPORT = 6		// 导出pdf
};


class DLL_API PointCloudToolInterface
{
public:
	PointCloudToolInterface();
	~PointCloudToolInterface();

	// [traindir]		输入要训练的文件夹， 一共二级目录，第一级是文件夹每个类别名字，第二级里放las文件,给目标点云标成红颜色。
	// [gridsize]		运算时的网格精度，越精细，速度越慢，默认0.3。
	ERRORNUM train(char *traindir, float gridsize = 0.3);

	// [lasfile]		点云文件
	// [outdir]			结果目录位置
	// [towerdir]		待纠偏的铁塔excel所在目录
	// [trainfile]		训练文件
	// [geo_zone]		地理分带号
	// [geo_southhemi]  是否南半球
	// [method]			训练方法：0，1，2 。 2是最精细。
	// [gridsize]		运算时的网格精度，越精细，速度越慢，默认0.3。
	// [overwrite_excel]覆盖铁塔纠偏原始文件。
	// [sign_modified]	纠偏后是否把背景标记。
	ERRORNUM check_distance_towers(char *lasfile, char *outdir, char *towerdir, char *trainfile, unsigned short geo_zone, bool geo_southhemi = false
		, int method = 2, float gridsize = 0.3, bool overwrite_excel = false, bool sign_modified = true);

};

