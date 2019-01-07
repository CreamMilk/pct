#pragma once
#include <string>
#include <QString>
#include <map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

class ChartSetConv
{
public:
	std::string static to_utf8(const std::wstring& str);
	std::string static to_utf8(const wchar_t* buffer, int len);
	std::wstring static String2WString(const std::string& s);
	
	std::string static WString2String(const std::wstring& ws);
	std::string static C2W(const std::string ws);
};


class StringUtil
{
public:
	void static StringReplace(std::string &strBase, std::string strSrc, std::string strDes);
};

class FileUtil
{
public:
	bool static DelDir(const QString &path);
	void static ReMakeDir(QString dir);
	std::string static getFileMD5(const std::string& filename);
	bool static CopyDirectory(const QString &fromDir, const QString &toDir);
};


class GeoUtil
{
public:
	double static rad(double d);
	double static GetLoglatDistance(float fLati1, float fLong1, float fLati2, float fLong2);
	std::map<QString, std::vector<boost::property_tree::ptree>>::iterator static GetNearImage(std::map<QString, std::vector<boost::property_tree::ptree>> &pos_images, double log, double lat, double z, double yuzhi = 5);
};

