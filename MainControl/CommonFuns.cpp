#include "CommonFuns.h"
#include <windows.h>
#include <QDir>
#include <QFile>
#include <QFileInfo>


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
	FileUtil::DelDir(dir);

	if (!QDir().exists(dir))
	{
		QDir().mkpath(dir);
	}
}

bool FileUtil::CopyDirectory(const QString &fromDir, const QString &toDir)
{
	QDir sourceDir(fromDir);
	QDir targetDir(toDir);
	if (!targetDir.exists()){    /**< ���Ŀ��Ŀ¼�����ڣ�����д��� */
		if (!targetDir.mkdir(targetDir.absolutePath()))
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
