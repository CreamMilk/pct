#pragma once
#include <string>

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
