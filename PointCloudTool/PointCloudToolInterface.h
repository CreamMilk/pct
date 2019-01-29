#pragma once

#ifdef DLL_EXPORTS
#define DLL_API __declspec(dllexport)
#else
#define DLL_API __declspec(dllimport)
#endif

enum ERRORNUM
{
	NORMAL = 0,			// ����
	LOADSETTING = 1,	// ���������ļ�ʧ��
	LASFILE = 2,	    // ��ȡlas�ļ�ʧ��
	OUTDIR = 3,			// outdir��ȡʧ��
	EXCELDIR = 4,		// ������ƫĿ¼������
	TRAINFILE = 5,		// ѵ���ļ�����ʧ��
	PDFEXPORT = 6		// ����pdf
};


class DLL_API PointCloudToolInterface
{
public:
	PointCloudToolInterface();
	~PointCloudToolInterface();

	// [traindir]		����Ҫѵ�����ļ��У� һ������Ŀ¼����һ�����ļ���ÿ��������֣��ڶ������las�ļ�,��Ŀ����Ʊ�ɺ���ɫ��
	// [gridsize]		����ʱ�����񾫶ȣ�Խ��ϸ���ٶ�Խ����Ĭ��0.3��
	ERRORNUM train(char *traindir, float gridsize = 0.3);

	// [lasfile]		�����ļ�
	// [outdir]			���Ŀ¼λ��
	// [towerdir]		����ƫ������excel����Ŀ¼
	// [trainfile]		ѵ���ļ�
	// [geo_zone]		����ִ���
	// [geo_southhemi]  �Ƿ��ϰ���
	// [method]			ѵ��������0��1��2 �� 2���ϸ��
	// [gridsize]		����ʱ�����񾫶ȣ�Խ��ϸ���ٶ�Խ����Ĭ��0.3��
	// [overwrite_excel]����������ƫԭʼ�ļ���
	// [sign_modified]	��ƫ���Ƿ�ѱ�����ǡ�
	ERRORNUM check_distance_towers(char *lasfile, char *outdir, char *towerdir, char *trainfile, unsigned short geo_zone, bool geo_southhemi = false
		, int method = 2, float gridsize = 0.3, bool overwrite_excel = false, bool sign_modified = true);

};

