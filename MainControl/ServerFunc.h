#pragma once
#include <QString>
class ServerFunc
{
public:
	QString  static GetLoginInfo();
	QString  static GetServerPort();
	QString  static GetProjectCode();
	QString  static GetProjectId();
	QString  static GetProjectName();
	ServerFunc();
	~ServerFunc();
};

