#include "CommonFuns.h"
#include <windows.h>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMessageBox>
#include <chrono>
#include <iostream>


std::string ChartSetConv::to_utf8(const wchar_t* buffer, int len)
{
	int nChars = ::WideCharToMultiByte(
		CP_UTF8,
		0,
		buffer,
		len,
		NULL,
		0,
		NULL,
		NULL);
	if (nChars == 0)return"";

	std::string newbuffer;
	newbuffer.resize(nChars);
	::WideCharToMultiByte(
		CP_UTF8,
		0,
		buffer,
		len,
		const_cast<char*>(newbuffer.c_str()),
		nChars,
		NULL,
		NULL);

	return newbuffer;
}

std::string ChartSetConv::to_utf8(const std::wstring& str)
{
	return to_utf8(str.c_str(), (int)str.size());
}

std::string ChartSetConv::WString2String(const std::wstring& ws)
{
	std::string strLocale = setlocale(LC_ALL, "");
	const wchar_t* wchSrc = ws.c_str();
	size_t nDestSize = wcstombs(NULL, wchSrc, 0) + 1;
	char *chDest = new char[nDestSize];
	memset(chDest, 0, nDestSize);
	wcstombs(chDest, wchSrc, nDestSize);
	std::string strResult = chDest;
	delete[]chDest;
	setlocale(LC_ALL, strLocale.c_str());
	return strResult;
}

std::string ChartSetConv::C2W(const std::string ws)
{
	return to_utf8(String2WString(ws));
}

// string => wstring
std::wstring ChartSetConv::String2WString(const std::string& s)
{
	std::string strLocale = setlocale(LC_ALL, "");
	const char* chSrc = s.c_str();
	size_t nDestSize = mbstowcs(NULL, chSrc, 0) + 1;
	wchar_t* wchDest = new wchar_t[nDestSize];
	wmemset(wchDest, 0, nDestSize);
	mbstowcs(wchDest, chSrc, nDestSize);
	std::wstring wstrResult = wchDest;
	delete[]wchDest;
	setlocale(LC_ALL, strLocale.c_str());
	return wstrResult;
}

void StringUtil::StringReplace(std::string &strBase, std::string strSrc, std::string strDes)
{
	std::string::size_type pos = 0;
	std::string::size_type srcLen = strSrc.size();
	std::string::size_type desLen = strDes.size();
	pos = strBase.find(strSrc, pos);
	while ((pos != std::string::npos))
	{
		strBase.replace(pos, srcLen, strDes);
		pos = strBase.find(strSrc, (pos + desLen));
	}
}

bool FileUtil::DelDir(const QString &path)
{
	if (path.isEmpty()){
		return false;
	}
	QDir dir(path);
	if (!dir.exists()){
		return true;
	}
	dir.setFilter(QDir::AllEntries | QDir::NoDotAndDotDot); //设置过滤
	QFileInfoList fileList = dir.entryInfoList(); // 获取所有的文件信息
	foreach(QFileInfo file, fileList){ //遍历文件信息
		if (file.isFile()){ // 是文件，删除
			file.dir().remove(file.fileName());
		}
		else{ // 递归删除
			DelDir(file.absoluteFilePath());
		}
	}
	return dir.rmpath(dir.absolutePath()); // 删除文件夹
}

void FileUtil::ReMakeDir(QString dir)
{
	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

	//删文件等30秒
	while (!FileUtil::DelDir(dir))
	{
		std::cout << "pct::DelDir(pic_dir)" << std::endl;
		if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count() > 30)
			return;
	}

	//等待磁盘刷新5秒
	start = std::chrono::system_clock::now();
	while (QDir().exists(dir))
	{
		std::cout << "QDir().exists(dir)" << std::endl;
		if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count() > 5)
			return;
	}


	//创建文件夹5秒
	start = std::chrono::system_clock::now();
	while (!QDir().mkpath(dir))
	{
		std::cout << "QDir().exists(dir)" << std::endl;
		if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count() > 5)
			return;
	}
}

bool FileUtil::CopyDirectory(const QString &fromDir, const QString &toDir)
{
	QDir sourceDir(fromDir);
	QDir targetDir(toDir);

	if (!targetDir.exists()){    /**< 如果目标目录不存在，则进行创建 */
		if (!targetDir.mkpath(targetDir.absolutePath()))
			return false;
	}

	QFileInfoList fileInfoList = sourceDir.entryInfoList();
	foreach(QFileInfo fileInfo, fileInfoList){
		if (fileInfo.fileName() == "." || fileInfo.fileName() == "..")
			continue;

		if (fileInfo.isDir()){    /**< 当为目录时，递归的进行copy */
			if (!CopyDirectory(fileInfo.filePath(),
				targetDir.filePath(fileInfo.fileName())
				))
				return false;
		}
		else{            /**< 将旧文件进行删除操作 */
			if (targetDir.exists(fileInfo.fileName())){
				targetDir.remove(fileInfo.fileName());
			}

			/// 进行文件copy
			if (!QFile::copy(fileInfo.filePath(),
				targetDir.filePath(fileInfo.fileName()))){
				return false;
			}
		}
	}
	return true;
}


// 角度转弧度
double GeoUtil::rad(double d)
{
	const double PI = 3.1415926535898;
	return d * PI / 180.0;
}
// 传入两个经纬度，计算之间的大致直线距离
double GeoUtil::GetLoglatDistance(float fLati1, float fLong1, float fLati2, float fLong2)
{
	const float EARTH_RADIUS = 6378.137;

	double radLat1 = rad(fLati1);
	double radLat2 = rad(fLati2);
	double a = radLat1 - radLat2;
	double b = rad(fLong1) - rad(fLong2);
	double s = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(radLat1)*cos(radLat2)*pow(sin(b / 2), 2)));
	s = s * EARTH_RADIUS;
	s = (int)(s * 10000000) / 10000;
	return s;
}


std::map<QString, std::vector<boost::property_tree::ptree>>::iterator 
GeoUtil::GetNearImage(std::map<QString, std::vector<boost::property_tree::ptree>> &pos_images, double log, double lat, double z, double yuzhi)
{
	std::map<QString, std::vector<boost::property_tree::ptree>>::iterator it = pos_images.end();
	for (it = pos_images.begin(); it != pos_images.end(); ++it)
	{
		QStringList loglat = it->first.split(',');
		if (GeoUtil::GetLoglatDistance(loglat[0].toDouble(), loglat[1].toDouble(), log, lat) < yuzhi)
			return it;
	}
	return it;
}