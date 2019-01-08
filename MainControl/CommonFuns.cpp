#include "CommonFuns.h"
#include <windows.h>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMessageBox>
#include <chrono>
#include <iostream>
#include "md5global.h"
#include "md5.h"

#include <fstream>
#include <sstream>
#include <memory>
#include <iomanip>
#include <exception>

#include<opencv2\opencv.hpp>   
#include<opencv2\highgui\highgui.hpp>

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
	dir.setFilter(QDir::AllEntries | QDir::NoDotAndDotDot); //���ù���
	QFileInfoList fileList = dir.entryInfoList(); // ��ȡ���е��ļ���Ϣ
	foreach(QFileInfo file, fileList){ //�����ļ���Ϣ
		if (file.isFile()){ // ���ļ���ɾ��
			file.dir().remove(file.fileName());
		}
		else{ // �ݹ�ɾ��
			DelDir(file.absoluteFilePath());
		}
	}
	return dir.rmpath(dir.absolutePath()); // ɾ���ļ���
}

void FileUtil::ReMakeDir(QString dir)
{
	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
	//ɾ�ļ���30��
	while (!FileUtil::DelDir(dir))
	{
		std::cout << "pct::DelDir(pic_dir)" << std::endl;
		if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count() > 30)
			return;
	}
	//�ȴ�����ˢ��5��
	start = std::chrono::system_clock::now();
	while (QDir().exists(dir))
	{
		std::cout << "QDir().exists(dir)" << std::endl;
		if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count() > 5)
			return;
	}

	//�����ļ���5��
	start = std::chrono::system_clock::now();
	while (!QDir().mkpath(dir))
	{
		std::cout << "QDir().exists(dir)" << std::endl;
		if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count() > 5)
			return;
	}
}

std::string FileUtil::getFileMD5(const std::string& filename)
{
	std::ifstream fin(filename.c_str(), std::ifstream::in | std::ifstream::binary);
	if (fin)
	{
		MD5_CTX context;
		MD5Init(&context);

		fin.seekg(0, fin.end);
		const auto fileLength = fin.tellg();
		fin.seekg(0, fin.beg);

		const int bufferLen = 8192;
		std::unique_ptr<unsigned char[]> buffer{ new unsigned char[bufferLen] {} };
		unsigned long long totalReadCount = 0;
		decltype(fin.gcount()) readCount = 0;
		// ��ȡ�ļ����ݣ�����MD5Update()����MD5ֵ
		while (fin.read(reinterpret_cast<char*>(buffer.get()), bufferLen))
		{
			readCount = fin.gcount();
			totalReadCount += readCount;
			MD5Update(&context, buffer.get(), static_cast<unsigned int>(readCount));
		}
		// �������һ�ζ���������
		readCount = fin.gcount();
		if (readCount > 0)
		{
			totalReadCount += readCount;
			MD5Update(&context, buffer.get(), static_cast<unsigned int>(readCount));
		}
		fin.close();

		// ���������Լ��
		if (totalReadCount != fileLength)
		{
			std::ostringstream oss;
			oss << "FATAL ERROR: read " << filename << " failed!" << std::ends;
			throw std::runtime_error(oss.str());
		}

		unsigned char digest[16];
		MD5Final(digest, &context);

		// ��ȡMD5
		std::ostringstream oss;
		for (int i = 0; i < 16; ++i)
		{
			oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(digest[i]);
		}
		oss << std::ends;

		return std::move(oss.str());
	}
	else
	{
		std::ostringstream oss;
		oss << "FATAL ERROR: " << filename << " can't be opened" << std::ends;
		throw std::runtime_error(oss.str());
	}
}

bool FileUtil::CopyDirectory(const QString &fromDir, const QString &toDir)
{
	QDir sourceDir(fromDir);
	QDir targetDir(toDir);

	if (!targetDir.exists()){    /**< ���Ŀ��Ŀ¼�����ڣ�����д��� */
		if (!targetDir.mkpath(targetDir.absolutePath()))
			return false;
	}

	QFileInfoList fileInfoList = sourceDir.entryInfoList();
	foreach(QFileInfo fileInfo, fileInfoList){
		if (fileInfo.fileName() == "." || fileInfo.fileName() == "..")
			continue;

		if (fileInfo.isDir()){    /**< ��ΪĿ¼ʱ���ݹ�Ľ���copy */
			if (!CopyDirectory(fileInfo.filePath(),
				targetDir.filePath(fileInfo.fileName())
				))
				return false;
		}
		else{            /**< �����ļ�����ɾ������ */
			if (targetDir.exists(fileInfo.fileName())){
				targetDir.remove(fileInfo.fileName());
			}

			/// �����ļ�copy
			if (!QFile::copy(fileInfo.filePath(),
				targetDir.filePath(fileInfo.fileName()))){
				return false;
			}
		}
	}
	return true;
}


// �Ƕ�ת����
double GeoUtil::rad(double d)
{
	const double PI = 3.1415926535898;
	return d * PI / 180.0;
}
// ����������γ�ȣ�����֮��Ĵ���ֱ�߾���
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
		if (loglat.size() == 2 && GeoUtil::GetLoglatDistance(loglat[0].toDouble(), loglat[1].toDouble(), log, lat) < yuzhi)
			return it;
	}
	return it;
}

void ImgUtil::ReSizeImage(std::string src, std::string dst, int width)
{
	cv::Mat img = cv::imread(src);
	int w = img.cols;
	int h = img.rows;
	if (w == width)
	{
		cv::imwrite(dst, img);
		return;
	}

	double n = n = width / float(w);
	int new_h = int(h*n);
	cv::Mat res_img;
	cv::resize(img, res_img, cv::Size(width, new_h), 0, 0, cv::INTER_AREA);
	cv::imwrite(dst, res_img);
}
