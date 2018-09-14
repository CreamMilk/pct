#include <stdbool.h>
#include <stdio.h>
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

#include <QTextStream>
#include <QFileInfo>
#include <QTextCodec>

boost::property_tree::ptree pt;
QString g_jsonpath;
QString g_htmlpath;
QString g_pdfpath;

int errcode;
#define myassert(content) \
errcode = content;\
if(1 != errcode)\
{\
    QMessageBox::critical(nullptr, QString::number(errcode) , #content);\
   QCoreApplication::exit(0);\
}

void GenerateHtml(QString json_path, QString html_path);
bool ParserCmdline(int argc, char *argv[]);
std::string GetExePath();
void ExportPdf(QString html_path, QString pdf_path);
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

/* Main method convert pdf */
int main(int argc, char *argv[]) {
    if (false == ParserCmdline(argc, argv))
    {
        return EXIT_FAILURE;
    }



    GenerateHtml(g_jsonpath, g_htmlpath);


    ExportPdf(g_htmlpath, g_pdfpath);
    return 0;
}


std::string GetExePath()
{
    std::string exe_path = "";
    //获取应用程序目录
    char szapipath[MAX_PATH] = { 0 };//（D:\Documents\Downloads\TEST.exe）
    GetModuleFileNameA(NULL, szapipath, MAX_PATH);
    //replace(szapipath, "\\", "/");
    exe_path = szapipath;
    return exe_path;
}

std::string GetExeName()
{
    std::string exe_path = "";
    //获取应用程序目录
    char szapipath[MAX_PATH] = { 0 };//（D:\Documents\Downloads\TEST.exe）
    GetModuleFileNameA(NULL, szapipath, MAX_PATH);
    //获取应用程序名称
    char szExe[MAX_PATH] = { 0 };//（TEST.exe）
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
    // 解析命令行
    boost::program_options::variables_map vm;
    boost::program_options::options_description opts("pdfReport");

    opts.add_options()
        ("jsonpath", boost::program_options::value<std::string>(), "")
        ("htmlpath", boost::program_options::value<std::string>(), "")
        ("pdfpath", boost::program_options::value<std::string>(), "")
        ("help", "帮助");

    try{
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opts), vm);
    }
    catch (...){
        std::cout << "输入的参数中存在未定义的选项！\n";
        return false;
    }

    // 检查并赋值
    if (vm.count("help")){//若参数中有help选项
        std::cout << opts << std::endl;
        return false;
    }
    else
    {
        if (vm.count("jsonpath"))
        {
            g_jsonpath = QString::fromLocal8Bit(vm["jsonpath"].as<std::string>().c_str());
            if (!QFileInfo(g_jsonpath).isFile())
            {
                QMessageBox::information(nullptr, "", "", 0);
                QCoreApplication::exit(0);
            }
               
        }
        else
            return false;

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
        }
    }
    
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

void GenerateHtml(QString json_path, QString html_path)
{
    QString yunxingguifan_front;
    QString yunxingguifan_template;
    QString tulizongbiao_front;
    QString tulizongbiao_template;
    QString yinhuanmingxi_front;
    QString yinhuanmingxi_template;
    QString last;
    QFile inputFile(":/PdfReport/Resources/template.html");

    int read_step = 0;
    QTextCodec *codec(QTextCodec::codecForName("GBK"));
    if (inputFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        while (!inputFile.atEnd())
        {
            QString line = codec->toUnicode(inputFile.readLine());
            if (read_step <= 18)
                yunxingguifan_front += (line /*+ QStringLiteral("\r\n")*/);
            else if (read_step <= 22)
                yunxingguifan_template += (line /*+ QStringLiteral("\r\n")*/);
            else if (read_step <= 33)
                tulizongbiao_front += (line /*+ QStringLiteral("\r\n")*/);
            else if (read_step <= 38)
                tulizongbiao_template += (line /*+ QStringLiteral("\r\n")*/);
            else if (read_step <= 54)
                yinhuanmingxi_front += (line /*+ QStringLiteral("\r\n")*/);
            else if (read_step <= 64)
                yinhuanmingxi_template += (line /*+ QStringLiteral("\r\n")*/);
            else
                last += (line /*+ QStringLiteral("\r\n")*/);
            read_step++;
        }
        inputFile.close();
    }
    
    // 读取配置文件
    
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
        QMessageBox::information(nullptr, "json_path", "open fail！", 0);
        QCoreApplication::exit(0);
    }
    
    /*
        QString yunxingguifan_front;
        QString yunxingguifan_template;
        QString tulizongbiao_front;
        QString tulizongbiao_template;
        QString yinhuanmingxi_front;
        QString yinhuanmingxi_template;
        QString last;
    */
    QString html;
    html += yunxingguifan_front;
    boost::property_tree::ptree traversee_pt = pt.get_child(to_utf8(String2WString("距离要求")));
    for (boost::property_tree::ptree::iterator it = traversee_pt.begin(); it != traversee_pt.end(); ++it)
    {
        //遍历读出数据
        std::string key = it->first;
        std::string val = it->second.get_value<std::string>();
        html += QString().sprintf(yunxingguifan_template.toUtf8().data(),
            key, val);
    }

    html += tulizongbiao_front;
    traversee_pt = pt.get_child(to_utf8(String2WString("图例颜色")));
    int serial = 0;
    for (boost::property_tree::ptree::iterator it = traversee_pt.begin(); it != traversee_pt.end(); ++it)
    {
        //遍历读出数据
        std::string key = it->first;
        std::string val = it->second.get_value<std::string>();
        QString scolor = QString::fromLocal8Bit(val.c_str());
        scolor.remove(' ');
        QStringList scolor_list = scolor.split(',');
        if (scolor_list.size() == 3)
        {
            int r = scolor_list[0].toInt() << 16;
            int g = scolor_list[1].toInt() << 8;
            int b = scolor_list[2].toInt();
            char hexcol[32] = { 0 };
            sprintf_s(hexcol, "%6x", r | g | b);
            html += QString().sprintf(tulizongbiao_template.toUtf8().data(),
                QString::number(++serial).toUtf8().data(), key, hexcol);
        }
    }

    html += yinhuanmingxi_front;
    traversee_pt = pt.get_child(to_utf8(String2WString("隐患列表")));
    BOOST_FOREACH(boost::property_tree::ptree::value_type &cvt, traversee_pt)
    {
        html += QString().sprintf(yinhuanmingxi_template.toUtf8().data()
            , cvt.second.get<std::string>(to_utf8(String2WString("序号")))
            , cvt.second.get<std::string>(to_utf8(String2WString("塔杆区间")))
            , cvt.second.get<std::string>(to_utf8(String2WString("隐患坐标")))
            , cvt.second.get<std::string>(to_utf8(String2WString("隐患半径")))
            , cvt.second.get<std::string>(to_utf8(String2WString("隐患类型")))
            , cvt.second.get<std::string>(to_utf8(String2WString("隐患距离")))
            , cvt.second.get<std::string>(to_utf8(String2WString("对地距离")))
            , cvt.second.get<std::string>(to_utf8(String2WString("超限率")))
            );
    }

    html += last;
    QFile htmlfile(html_path);
    if (htmlfile.open(QIODevice::ReadWrite | QIODevice::Text | QIODevice::Truncate))
    {
        QTextStream in(&htmlfile);
        in << html;
        htmlfile.close();
    }
}




void ExportPdf(QString html_path, QString pdf_path)
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


    myassert(wkhtmltopdf_set_global_setting(gs, "out", pdf_path.toLocal8Bit().data()));
    myassert( wkhtmltopdf_set_global_setting(gs, "load.cookieJar", "myjar.jar"));
    /*
    * Create a input object settings object that is used to store settings
    * related to a input object, note again that control of this object is parsed to
    * the converter later, which is then responsible for freeing it
    */
    os = wkhtmltopdf_create_object_settings();
    /* We want to convert to convert the qstring documentation page */
    myassert(wkhtmltopdf_set_object_setting(os, "page", html_path.toLocal8Bit().data()));
    myassert(wkhtmltopdf_set_object_setting(os, "toc.indentation", "2em"));
    myassert(wkhtmltopdf_set_object_setting(os, "useExternalLinks", "true"));

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
    wkhtmltopdf_deinit();
}
