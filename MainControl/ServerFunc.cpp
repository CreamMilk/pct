#include "ServerFunc.h"
#include <windows.h>
#include <QMessageBox>
#include <string>
#include <QJsonDocument>
#include <QJsonObject>

QString ServerFunc::GetLoginInfo()
{
	HANDLE m_hMapFile = NULL;
	void* m_pBaseMapFile = NULL;

	if (m_hMapFile == NULL)
	{
		m_hMapFile = OpenFileMappingA(FILE_MAP_READ, FALSE, "UseMMFBetweenProcess");
		if (m_hMapFile == NULL)
		{
			DWORD dwLastErr = GetLastError();
			QString sMsg(QStringLiteral("获得登录信息失败，错误码: 0x%1"));
			sMsg.arg(dwLastErr, 4, 16, QLatin1Char('0'));
			return QStringLiteral("");
		}

		m_pBaseMapFile = MapViewOfFile(m_hMapFile, FILE_MAP_READ, 0, 0, 0);
		if (m_pBaseMapFile == NULL)
		{
			DWORD dwLastErr = GetLastError();
			//CString sMsg;
			//sMsg.Format(_T("获得登录数据失败，错误码: 0x%X"), dwLastErr);
			//AfxMessageBox(sMsg, MB_OK|MB_ICONERROR);
			if (m_hMapFile != NULL)
			{
				// 关闭内存映射文件对象句柄
				CloseHandle(m_hMapFile);
			}
			return QStringLiteral("");
		}

	}

	int strLength = *(int*)m_pBaseMapFile;

	void* pData = (BYTE*)m_pBaseMapFile + sizeof(int);

	wchar_t* pchar = (wchar_t*)pData;
	pchar = pchar;
	wchar_t buffer[1024 * 4];
	int len;
	for (int i = 0; i < strLength; i++)
	{
		buffer[i] = *pchar;
		pchar = pchar++;
		len = i;
	}
	buffer[len + 1] = '\0';
	return QString::fromWCharArray(buffer);
}

QString ServerFunc::GetServerPort()
{
	QString info = GetLoginInfo();
	if (info.isEmpty())
		return QString();

	QString serverPort;
	QJsonParseError jsonError;//Qt5新类 
	QJsonDocument json = QJsonDocument::fromJson(info.toStdString().c_str(), &jsonError);//Qt5新类
	QJsonObject json_object = json.object();
	QJsonObject info_object = json_object[QStringLiteral("userInfo")].toObject();
	serverPort = info_object[QStringLiteral("EcpIntranetURL")].toString();
	return serverPort;
}

QString ServerFunc::GetProjectCode()
{
	QString info = GetLoginInfo();
	if (info.isEmpty())
		return QString();

	QString proj_id;
	QJsonParseError jsonError;//Qt5新类 
	QJsonDocument json = QJsonDocument::fromJson(info.toStdString().c_str(), &jsonError);//Qt5新类
	QJsonObject json_object = json.object();
	QJsonObject info_object = json_object[QStringLiteral("projInfo")].toObject();
	proj_id = info_object[QStringLiteral("ProjectCode")].toString();
	return proj_id;
}

QString ServerFunc::GetProjectId()
{
	QString info = GetLoginInfo();
	if (info.isEmpty())
		return QString();

	QString proj_id;
	QJsonParseError jsonError;//Qt5新类 
	QJsonDocument json = QJsonDocument::fromJson(info.toStdString().c_str(), &jsonError);//Qt5新类
	QJsonObject json_object = json.object();
	QJsonObject info_object = json_object[QStringLiteral("projInfo")].toObject();
	proj_id = info_object[QStringLiteral("ProjectID")].toString();
	return proj_id;
}

QString ServerFunc::GetProjectName()
{
	QString info = GetLoginInfo();
	if (info.isEmpty())
		return QString();

	QString proj_id;
	QJsonParseError jsonError;//Qt5新类 
	QJsonDocument json = QJsonDocument::fromJson(info.toStdString().c_str(), &jsonError);//Qt5新类
	QJsonObject json_object = json.object();
	QJsonObject info_object = json_object[QStringLiteral("projInfo")].toObject();
	proj_id = info_object[QStringLiteral("ProjectName")].toString();
	return proj_id;
}


ServerFunc::ServerFunc()
{
}


ServerFunc::~ServerFunc()
{
}
