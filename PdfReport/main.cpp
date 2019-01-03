#include <stdbool.h>
#include <stdio.h>
#include <wkhtmltox/image.h>
#include <wkhtmltox/pdf.h>
#include <QMessageBox>
#include <QCoreApplication>
#include <QFile>
#include <windows.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/typeof/typeof.hpp>   
#include <boost/locale.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

#include <iostream>
#include <fstream>
#include <cassert>
#include <string>

#include <QApplication>
#include <QTextStream>
#include <QFileInfo>
#include <QTextCodec>
#include <QTimer>
#include <QDir>

boost::property_tree::ptree pt;
QString g_checktype;
QString g_jsonpath;
QString g_htmlpath;
QString g_pdfpath;
QString g_imgpath;

int errcode = 0;
#define myassert(content) \
errcode = content;\
if(1 != errcode)\
{\
    QMessageBox::critical(nullptr, QString::number(errcode) , #content);\
   return false;\
}

void GenerateCloudHtml(QString json_path, QString html_path);
void GenerateBirdHtml(QString json_path, QString html_path);

bool ParserCmdline(int argc, char *argv[]);
std::string GetExePath();
bool ExportPdf(QString html_path, QString pdf_path);
void ExportImage(QString html_path, QString img_path);
std::string GetExeName();

/* Print out loading progress information */
void progress_changed(wkhtmltopdf_converter * c, int p) {
    printf("%3d%%\r", p);
    fflush(stdout);
}

/* Print loading phase information */
void phase_changed(wkhtmltopdf_converter * c) {
    int phase = wkhtmltopdf_current_phase(c);
    printf("%s\n", wkhtmltopdf_phase_description(c, phase));
}

/* Print a message to stderr when an error occurs */
void error(wkhtmltopdf_converter * c, const char * msg) {
    fprintf(stderr, "Error: %s\n", msg);
}

/* Print a message to stderr when a warning is issued */
void warning(wkhtmltopdf_converter * c, const char * msg) {
    fprintf(stderr, "Warning: %s\n", msg);
}

void progress_changed(wkhtmltoimage_converter * c, int p) {
    printf("%3d%%\r", p);
    fflush(stdout);
}

/* Print loading phase information */
void phase_changed(wkhtmltoimage_converter * c) {
    int phase = wkhtmltoimage_current_phase(c);
    printf("%s\n", wkhtmltoimage_phase_description(c, phase));
}

/* Print a message to stderr when an error occurs */
void error(wkhtmltoimage_converter * c, const char * msg) {
    fprintf(stderr, "Error: %s\n", msg);
}

/* Print a message to stderr when a warning is issued */
void warning(wkhtmltoimage_converter * c, const char * msg) {
    fprintf(stderr, "Warning: %s\n", msg);
}

typedef std::vector<std::string> vecString;
void ReadTxtFile(std::string file, vecString& vecStr)
{
	std::ifstream infile;
	infile.open(file.data());   //���ļ����������ļ��������� 
	assert(infile.is_open());   //��ʧ��,�����������Ϣ,����ֹ�������� 

	std::string strTemp;
	while (getline(infile, strTemp))
	{
		vecStr.push_back(strTemp);
	}
	infile.close();             //�ر��ļ������� 
}

/* Main method convert pdf */
int main(int argc, char *argv[]) 
{
	QApplication a(argc, argv);
    if (false == ParserCmdline(argc, argv))
    {
        return EXIT_FAILURE;
    }
	if (g_checktype == QStringLiteral("brid"))
	{
		GenerateBirdHtml(g_jsonpath, g_htmlpath);
	}
	else
	{
		GenerateCloudHtml(g_jsonpath, g_htmlpath);
	}

	ExportPdf(g_htmlpath, g_pdfpath);
	//ExportImage(g_htmlpath, g_imgpath);

    //ExportPdf("E:/project/powerline/solution/PointCloudTool/x64/Release/testdata2/data-20170519-135919-179-1-1/data-20170519-135919-179-1-1�����.html",
    //    "E:/project/powerline/solution/PointCloudTool/x64/Release/testdata2/data-20170519-135919-179-1-1/data-20170519-135919-179-1-1�����.pdf");
}


std::string GetExePath()
{
    std::string exe_path = "";
    //��ȡӦ�ó���Ŀ¼
    char szapipath[MAX_PATH] = { 0 };//��D:\Documents\Downloads\TEST.exe��
    GetModuleFileNameA(NULL, szapipath, MAX_PATH);
    //replace(szapipath, "\\", "/");
    exe_path = szapipath;
    return exe_path;
}

QString GetfilePath(QString qpath)
{
    qpath.replace("\\", "/");
    return qpath.left(qpath.lastIndexOf('/'));
}

std::string GetExeName()
{
    std::string exe_path = "";
    //��ȡӦ�ó���Ŀ¼
    char szapipath[MAX_PATH] = { 0 };//��D:\Documents\Downloads\TEST.exe��
    GetModuleFileNameA(NULL, szapipath, MAX_PATH);
    //��ȡӦ�ó�������
    char szExe[MAX_PATH] = { 0 };//��TEST.exe��
    char *pbuf = nullptr;
    char* szLine = strtok_s(szapipath, "\\", &pbuf);
    while (NULL != szLine)
    {
        strcpy_s(szExe, szLine);
        szLine = strtok_s(NULL, "\\", &pbuf);
    }
    exe_path = szExe;
    return exe_path;
}

bool ParserCmdline(int argc, char *argv[])
{
    std::string exe_name = GetExeName();
    // ����������
    boost::program_options::variables_map vm;
    boost::program_options::options_description opts("pdfReport");

    opts.add_options()
        ("jsonpath", boost::program_options::value<std::string>(), "")
        ("htmlpath", boost::program_options::value<std::string>(), "")
        ("pdfpath", boost::program_options::value<std::string>(), "")
        ("imgpath", boost::program_options::value<std::string>(), "")
		("checktype", boost::program_options::value<std::string>(), "")
        ("help", "����");

    try{
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opts), vm);
    }
    catch (...){
        std::cout << "����Ĳ����д���δ�����ѡ�\n";
        return false;
    }

    // ��鲢��ֵ
    if (vm.count("help")){//����������helpѡ��
        std::cout << opts << std::endl;
        return false;
    }
    else
	{
		if (vm.count("checktype"))
		{
			g_checktype = QString::fromLocal8Bit(vm["checktype"].as<std::string>().c_str());
		}
        if (vm.count("jsonpath"))
        {
            g_jsonpath = QString::fromLocal8Bit(vm["jsonpath"].as<std::string>().c_str());
            if (!QFileInfo(g_jsonpath).isFile())
            {
                QMessageBox::information(nullptr, "", "", 0);
				return false;
            }
               
        }
        else
		{ 
			return false;
		}
           

        if (vm.count("htmlpath"))
        {
            g_htmlpath = QString::fromLocal8Bit(vm["htmlpath"].as<std::string>().c_str());
        }
        else
        {
            g_htmlpath = g_jsonpath;
            g_htmlpath = g_htmlpath.replace(QStringLiteral("json"), QStringLiteral("html"));
        }
           
        if (vm.count("pdfpath"))
        {
            g_pdfpath = QString::fromLocal8Bit(vm["pdfpath"].as<std::string>().c_str());
        }
        else
        {
            g_pdfpath = g_jsonpath;
            g_pdfpath = g_pdfpath.replace(QStringLiteral("json"), QStringLiteral("pdf"));
            g_pdfpath = g_pdfpath.replace('\\', '/');
        }

        if (vm.count("imgpath"))
        {
            g_imgpath = QString::fromLocal8Bit(vm["imgpath"].as<std::string>().c_str());
        }
        else
        {
            g_imgpath = g_jsonpath;
            g_imgpath = g_imgpath.replace(QStringLiteral("json"), QStringLiteral("jpeg"));
            g_imgpath = g_imgpath.replace('\\', '/');
        }
    }
	//std::string s = g_imgpath.toLocal8Bit().data();
    return true;
}


std::string to_utf8(const wchar_t* buffer, int len)
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

std::string to_utf8(const std::wstring& str)
{
    return to_utf8(str.c_str(), (int)str.size());
}

std::string WString2String(const std::wstring& ws)
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
// string => wstring
std::wstring String2WString(const std::string& s)
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

std::string & StringReplace(std::string &strBase, std::string strSrc, std::string strDes)
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
    return strBase;
}

std::string U2A(std::string src)
{
    QString mid = QString::fromUtf8(src.c_str());
    return std::string(mid.toLocal8Bit().data());
}


void GenerateCloudHtml(QString json_path, QString html_path)
{
	// ***_front����.html�ļ��в���Ҫ�Ķ��Ĳ��֣�***_template����.html�ļ�����Ҫ��ֵ�Ķ��Ĳ���
    QFile::remove(html_path);
	// ��·��Ϣ
	QString xianluxinxi_front;
	QString xianluxinxi_template;
	// ������
	QString jiancejielun_front;
	QString jiancejielun_template;
	// ���й淶
    QString yunxingguifan_front;
    QString yunxingguifan_template;
	// ͼ���ܱ�
    QString tulizongbiao_front;
    QString tulizongbiao_template;
	// ������ͼ
    QString yinhuanzongtu_fronttemplate;
	// ������ϸ
    QString yinhuanmingxi_front;
    QString yinhuanmingxi_template;
	QString yinhuanmingxi_other;
	// ��������
    QString yinhuanxiangqing_front;
    QString yinhuanxiangqing_template;
	// ����
    QString last_Section;

    // utf8��ʽ��
	QFile inputFile(QStringLiteral(":/PdfReport/Resources/Cloudtemplate.html"));
    QString json_dir = GetfilePath(json_path);

	// ��ȡ.htmlģ��ȫ������
	QStringList strList;
	QTextCodec *codec(QTextCodec::codecForName("UTF-8"));
	if (inputFile.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		while (!inputFile.atEnd())
		{
			QString strLine = codec->toUnicode(inputFile.readLine());
			strList.append(strLine);
		}

		inputFile.close();
	}

	for (int nLine = 0; nLine < strList.size(); nLine++)
	{
		QString line = strList.at(nLine);
		if (nLine <= 15)
			xianluxinxi_front += (line);
		else if (nLine <= 26)
			xianluxinxi_template += (line);
		else if (nLine <= 28) //
			jiancejielun_front += (line);
		else if (nLine <= 30)
			jiancejielun_template += (line);
		else if (nLine <= 39)
			yunxingguifan_front += (line);
		else if (nLine <= 44)
			yunxingguifan_template += (line);
		else if (nLine <= 55)
			tulizongbiao_front += (line);
		else if (nLine <= 60)
			tulizongbiao_template += (line);
		else if (nLine <= 65)
			yinhuanzongtu_fronttemplate += (line);
		else if (nLine <= 80)
			yinhuanmingxi_front += (line);
		else if (nLine <= 91)
			yinhuanmingxi_template += (line);
		else if (nLine <= 105)
			yinhuanmingxi_other += (line);
		else if (nLine <= 118)
			yinhuanxiangqing_front += (line);
		else if (nLine <= 137)
			yinhuanxiangqing_template += (line);
		else
			last_Section += (line);
	}
    
    // ��ȡ.json�����ļ�  utf8��ʽ��
    
    std::ifstream is;
    is.open(json_path.toLocal8Bit().data(), std::ios::in);
    if (is.is_open())
    {
        try {
            read_json(is, pt);          //parse json
            is.close();
        }
        catch (...) {
            return;
        }
        is.close();
    }
    else
    {
        QMessageBox::information(nullptr, "��ҵ��ʾ", "�������.json��ʧ�ܣ�", 0);
        return;
    }
    
	boost::property_tree::ptree juliyaoqiu_pt = pt.get_child(to_utf8(String2WString("����Ҫ��")));
	boost::property_tree::ptree tuliyanse_pt = pt.get_child(to_utf8(String2WString("ͼ����ɫ")));
	boost::property_tree::ptree yinhuanliebiao_pt = pt.get_child(to_utf8(String2WString("�����б�")));


	QString html;
	html += xianluxinxi_front;

	// ��ȡtxt�ļ�ֵ
	QString txt_dir = QFileInfo(json_path).absoluteDir().absolutePath();
	txt_dir += QStringLiteral("/��·��Ϣ.txt");
	std::string strTxt = txt_dir.toLocal8Bit().data();
	vecString vecStr;
	ReadTxtFile(strTxt, vecStr);
	std::string strNum1 = vecStr.at(0).substr(vecStr.at(0).find("=") + 1, vecStr.at(0).length() - vecStr.at(0).find("="));
	std::string strNum2 = vecStr.at(1).substr(vecStr.at(1).find("=") + 1, vecStr.at(1).length() - vecStr.at(1).find("="));
	std::string strNum3 = vecStr.at(2).substr(vecStr.at(2).find("=") + 1, vecStr.at(2).length() - vecStr.at(2).find("="));
	std::string strNum4 = vecStr.at(3).substr(vecStr.at(3).find("=") + 1, vecStr.at(3).length() - vecStr.at(3).find("="));;
	QString strQNum1 = QString::fromLocal8Bit(strNum1.c_str());
	QString	strQNum2 = QString::fromLocal8Bit(strNum2.c_str());
	QString strQNum3 = QString::fromLocal8Bit(strNum3.c_str());
	QString strQNum4 = QString::fromLocal8Bit(strNum4.c_str());

	QString ceshitu = QString::fromUtf8(pt.get<std::string>(to_utf8(String2WString("�߳���ɫ����ͼ·��"))).c_str());
	QString fushitu = QString::fromUtf8(pt.get<std::string>(to_utf8(String2WString("�߳���ɫ����ͼ·��"))).c_str());

	html += xianluxinxi_template.arg(strQNum1).arg(strQNum2).arg(strQNum3).arg(strQNum4).arg(ceshitu).arg(fushitu);
	
	html += jiancejielun_front;
	html += jiancejielun_template.arg(yinhuanliebiao_pt.size()).arg(yinhuanliebiao_pt.size() == 0 ? QStringLiteral("�ϸ�") : QStringLiteral("���ϸ�"));



	// ����ȡ��.json��ֵ����.html��Ӧ������
    QString zongtu = QString::fromUtf8(pt.get<std::string>(to_utf8(String2WString("xyƽ��ͼ·��"))).c_str());

    html += yunxingguifan_front;
	for (boost::property_tree::ptree::iterator it = juliyaoqiu_pt.begin(); it != juliyaoqiu_pt.end(); ++it)
    {
        //������������
        QString key = QString::fromUtf8(it->first.c_str());
        QStringList val = QString::fromUtf8(it->second.get_value<std::string>().c_str()).remove(" ").split(",");

        html += yunxingguifan_template.arg(key).arg(val[0]).arg(val[1]);
    }

    html += tulizongbiao_front;
    int serial = 0;
	for (boost::property_tree::ptree::iterator it = tuliyanse_pt.begin(); it != tuliyanse_pt.end(); ++it)
    {
        //������������
        QString key = QString::fromUtf8(it->first.c_str());
        QString val = QString::fromUtf8(it->second.get_value<std::string>().c_str());
        QString scolor = val;
        scolor.remove(' ');
        QStringList scolor_list = scolor.split(',');
        if (scolor_list.size() == 3)
        {
            int r = scolor_list[0].toInt() << 16;
            int g = scolor_list[1].toInt() << 8;
            int b = scolor_list[2].toInt();
            char hexcol[32] = { 0 };
            sprintf_s(hexcol, "%6x", r | g | b);

            html += tulizongbiao_template
                .arg(++serial)
                .arg(key)
                .arg(QString::fromLocal8Bit(hexcol));
        }
    }

      html += yinhuanzongtu_fronttemplate.arg(zongtu);
  
      html += yinhuanmingxi_front;
	  BOOST_FOREACH(boost::property_tree::ptree::value_type &cvt, yinhuanliebiao_pt)
      {
          QString yinhuanzuobiao = QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��������"))).c_str()).remove(' ').replace(',', "<br/>");
          QString taganqujian = QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��������"))).c_str()).remove(' ').replace('-', "<br/>-<br/>");
  
          html += yinhuanmingxi_template
              .arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("���"))).c_str()))
              .arg(taganqujian)
              .arg(yinhuanzuobiao)
              .arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("�����뾶"))).c_str()))
              .arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��������"))).c_str()))
              .arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��������"))).c_str()))
              .arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("ˮƽ����"))).c_str()))
              .arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��ֱ����"))).c_str()))
              .arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("������"))).c_str()));
      }
	  html += yinhuanmingxi_other;

	  BOOST_FOREACH(boost::property_tree::ptree::value_type &cvt, yinhuanliebiao_pt)
	  {
		  html += yinhuanxiangqing_front;
		  QString yinhuanzuobiao = QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��������"))).c_str()).remove(' ').replace(',', "<br/>");
		  QString taganqujian = QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��������"))).c_str()).remove(' ').replace('-', "<br/>-<br/>");
		  QString jubuzongtu = QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��ͼ"))).c_str());
		  QString xijietu = QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("ϸ��ͼ"))).c_str());


		  html += yinhuanxiangqing_template
			  .arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("���"))).c_str()))
			  .arg(taganqujian)
			  .arg(yinhuanzuobiao)
			  .arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("�����뾶"))).c_str()))
			  .arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��������"))).c_str()))
			  .arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��������"))).c_str()))
			  .arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("ˮƽ����"))).c_str()))
			  .arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��ֱ����"))).c_str()))
			  .arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("������"))).c_str()))
			  .arg(jubuzongtu)
			  .arg(xijietu);
	  }

    html += last_Section;

	// ��ȡ��ֵ����������.html�ļ�
    QFile htmlfile(html_path);
    //char bom[] = { (byte)0xEF, (byte)0xBB, (byte)0xBF };
    if (htmlfile.open(QIODevice::ReadWrite | QIODevice::Text | QIODevice::Truncate))
    {
        QTextStream inStream(&htmlfile);
		inStream.setCodec("UTF-8"); //��ע������
		inStream << html;
        htmlfile.close();
    }
}

void GenerateBirdHtml(QString json_path, QString html_path)
{
	// ***_front����.html�ļ��в���Ҫ�Ķ��Ĳ��֣�***_template����.html�ļ�����Ҫ��ֵ�Ķ��Ĳ���
	QFile::remove(html_path);
	// ��·��Ϣ
	QString BirdLineInfo_front;
	QString BirdLineInfo_template;

	// �������ϸ��
	QString BirdMingXi_front;
	QString BirdMingXi_template;

	// ���������
	QString BirdXiangQing_front;
	QString BirdXiangQing_template;

	// ����
	QString last_Section;

	// utf8��ʽ��
	QFile inputFile(QStringLiteral(":/PdfReport/Resources/Birdtemplate.html"));
	QString json_dir = GetfilePath(json_path);

	// ��ȡ.htmlģ��ȫ������
	QStringList strList;
	QTextCodec *codec(QTextCodec::codecForName("UTF-8"));
	if (inputFile.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		while (!inputFile.atEnd())
		{
			QString strLine = codec->toUnicode(inputFile.readLine());
			strList.append(strLine);
		}

		inputFile.close();
	}

	for (int nLine = 0; nLine < strList.size(); nLine++)
	{
		QString line = strList.at(nLine);
		if (nLine <= 15)
			BirdLineInfo_front += (line);
		else if (nLine <= 20)
			BirdLineInfo_template += (line);
		else if (nLine <= 30)
			BirdMingXi_front += (line);
		else if (nLine <= 37)
			BirdMingXi_template += (line);
		else if (nLine <= 40)
			BirdXiangQing_front += (line);
		else if (nLine <= 85)
			BirdXiangQing_template += (line);
		else
			last_Section += (line);
	}

	// ��ȡ.json�����ļ�  utf8��ʽ��

	std::ifstream is;
	is.open(json_path.toLocal8Bit().data(), std::ios::in);
	if (is.is_open())
	{
		try {
			read_json(is, pt);          //parse json
			is.close();
		}
		catch (...) {
			return;
		}
		is.close();
	}
	else
	{
		QMessageBox::information(nullptr, "json_path", "open fail��", 0);
		return;
	}

	QString html;
	html += BirdLineInfo_front;

	// ��ȡtxt�ļ�ֵ
	QString txt_dir = QFileInfo(json_path).absoluteDir().absolutePath();
	txt_dir += QStringLiteral("/��·��Ϣ.txt");
	std::string strTxt = txt_dir.toLocal8Bit().data();
	vecString vecStr;
	ReadTxtFile(strTxt, vecStr);
	std::string strNum1 = vecStr.at(0).substr(vecStr.at(0).find("=") + 1, vecStr.at(0).length() - vecStr.at(0).find("="));
	std::string strNum2 = vecStr.at(1).substr(vecStr.at(1).find("=") + 1, vecStr.at(1).length() - vecStr.at(1).find("="));
	std::string strNum3 = vecStr.at(2).substr(vecStr.at(2).find("=") + 1, vecStr.at(2).length() - vecStr.at(2).find("="));
	std::string strNum4 = vecStr.at(3).substr(vecStr.at(3).find("=") + 1, vecStr.at(3).length() - vecStr.at(3).find("="));;
	QString strQNum1 = QString::fromLocal8Bit(strNum1.c_str());
	QString	strQNum2 = QString::fromLocal8Bit(strNum2.c_str());
	QString strQNum3 = QString::fromLocal8Bit(strNum3.c_str());
	QString strQNum4 = QString::fromLocal8Bit(strNum4.c_str());



	html += BirdLineInfo_template.arg(strQNum1).arg(strQNum2).arg(strQNum3).arg(strQNum4);

	// ����ȡ��.json��ֵ����.html��Ӧ������
	html += BirdMingXi_front;
	boost::property_tree::ptree traversee_pt = pt.get_child(to_utf8(String2WString("������б�")));
	BOOST_FOREACH(boost::property_tree::ptree::value_type &cvt, traversee_pt)
	{
		html += BirdMingXi_template
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("���"))).c_str()))
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��·����"))).c_str()))
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("�������"))).c_str()))
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("ȱ������"))).c_str()))
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��ע"))).c_str()));
	}

	html += BirdXiangQing_front;
	BOOST_FOREACH(boost::property_tree::ptree::value_type &cvt, traversee_pt)
	{
		html += BirdXiangQing_template
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("���"))).c_str()))
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��·����"))).c_str()))
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("�������"))).c_str()))
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("ȱ������"))).c_str()))
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��ע"))).c_str()))
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��ͼ"))).c_str()))
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("��������"))).c_str()))
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("�ļ�����"))).c_str()))
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("����"))).c_str()))
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("γ��"))).c_str()))
			.arg(QString::fromUtf8(cvt.second.get<std::string>(to_utf8(String2WString("����λ��"))).c_str()));
	}

	html += last_Section;

	// ��ȡ��ֵ����������.html�ļ�
	QFile htmlfile(html_path);
	//char bom[] = { (byte)0xEF, (byte)0xBB, (byte)0xBF };
	if (htmlfile.open(QIODevice::ReadWrite | QIODevice::Text | QIODevice::Truncate))
	{
		QTextStream inStream(&htmlfile);
		inStream.setCodec("UTF-8"); //��ע������
		inStream << html;
		htmlfile.close();
	}
}

void ExportImage(QString html_path, QString img_path) {
    wkhtmltoimage_global_settings * gs;
    wkhtmltoimage_converter * c;
    const unsigned char * data;
    long len;

    /* Init wkhtmltoimage in graphics less mode */
    wkhtmltoimage_init(false);

    /*
    * Create a global settings object used to store options that are not
    * related to input objects, note that control of this object is parsed to
    * the converter later, which is then responsible for freeing it
    */
    gs = wkhtmltoimage_create_global_settings();

    /* We want to convert the qstring documentation page */
    wkhtmltoimage_set_global_setting(gs, "in", html_path.toUtf8().data());
    wkhtmltoimage_set_global_setting(gs, "out", img_path.toUtf8().data());
    wkhtmltoimage_set_global_setting(gs, "fmt", "jpeg");

    /* Create the actual converter object used to convert the pages */
    c = wkhtmltoimage_create_converter(gs, NULL);

    /* Call the progress_changed function when progress changes */
    wkhtmltoimage_set_progress_changed_callback(c, progress_changed);

    /* Call the phase _changed function when the phase changes */
    wkhtmltoimage_set_phase_changed_callback(c, phase_changed);

    /* Call the error function when an error occurs */
    wkhtmltoimage_set_error_callback(c, error);

    /* Call the warning function when a warning is issued */
    wkhtmltoimage_set_warning_callback(c, warning);

    /* Perform the actual conversion */
    if (!wkhtmltoimage_convert(c))
        fprintf(stderr, "Conversion failed!");

    /* Output possible http error code encountered */
    printf("httpErrorCode: %d\n", wkhtmltoimage_http_error_code(c));

    len = wkhtmltoimage_get_output(c, &data);
    printf("%ld len\n", len);


    /* Destroy the converter object since we are done with it */
    wkhtmltoimage_destroy_converter(c);

    /* We will no longer be needing wkhtmltoimage funcionality */
    wkhtmltoimage_deinit();
}


bool ExportPdf(QString html_path, QString pdf_path)
{
    QFile::remove(pdf_path);

    int result = 0;
    wkhtmltopdf_global_settings * gs;
    wkhtmltopdf_object_settings * os;
    wkhtmltopdf_converter * c;

    /* Init wkhtmltopdf in graphics less mode */
    wkhtmltopdf_init(false);

    /*
    * Create a global settings object used to store options that are not
    * related to input objects, note that control of this object is parsed to
    * the converter later, which is then responsible for freeing it
    */
    gs = wkhtmltopdf_create_global_settings();
    myassert(wkhtmltopdf_set_global_setting(gs, "collate", "true"));
    myassert(wkhtmltopdf_set_global_setting(gs, "outline", "true"));
    myassert(wkhtmltopdf_set_global_setting(gs, "margin.top", "2cm"));
   


    myassert(wkhtmltopdf_set_global_setting(gs, "out", pdf_path.toUtf8().data()));
    myassert( wkhtmltopdf_set_global_setting(gs, "load.cookieJar", "myjar.jar"));
    /*
    * Create a input object settings object that is used to store settings
    * related to a input object, note again that control of this object is parsed to
    * the converter later, which is then responsible for freeing it
    */
    os = wkhtmltopdf_create_object_settings();
    /* We want to convert to convert the qstring documentation page */
    myassert(wkhtmltopdf_set_object_setting(os, "page", html_path.toUtf8().data()));
    myassert(wkhtmltopdf_set_object_setting(os, "toc.indentation", "2em"));
    myassert(wkhtmltopdf_set_object_setting(os, "useExternalLinks", "true"));
    myassert(wkhtmltopdf_set_object_setting(os, "web.loadImages", "true"));

    myassert(wkhtmltopdf_set_object_setting(os, "web.background", "true"));
    myassert(wkhtmltopdf_set_object_setting(os, "web.defaultEncoding", "utf-8"));

    /* Create the actual converter object used to convert the pages */
    c = wkhtmltopdf_create_converter(gs);

    /* Call the progress_changed function when progress changes */
    wkhtmltopdf_set_progress_changed_callback(c, progress_changed);

    /* Call the phase _changed function when the phase changes */
    wkhtmltopdf_set_phase_changed_callback(c, phase_changed);

    /* Call the error function when an error occurs */
    wkhtmltopdf_set_error_callback(c, error);

    /* Call the warning function when a warning is issued */
    wkhtmltopdf_set_warning_callback(c, warning);

    /*
    * Add the the settings object describing the qstring documentation page
    * to the list of pages to convert. Objects are converted in the order in which
    * they are added
    */
    wkhtmltopdf_add_object(c, os, NULL);
    
    /* Perform the actual conversion */
    myassert(wkhtmltopdf_convert(c));

    /* Output possible http error code encountered */
    errcode = wkhtmltopdf_http_error_code(c);
    printf("httpErrorCode: %d\n", errcode);
    //QMessageBox::critical(nullptr, QString::number(errcode),
    //    QString("httpErrorCode: %1").arg(QString::number(errcode)));

    /* Destroy the converter object since we are done with it */
    wkhtmltopdf_destroy_converter(c);

    /* We will no longer be needing wkhtmltopdf funcionality */
    myassert(wkhtmltopdf_deinit());


	return true;
}
