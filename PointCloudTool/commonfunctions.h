#pragma once
#include <vector>
#include <string>

#define setsettingitem(itemname, itemtpye, must, def) \
    itemtpye itemname;\
if (vm.count(#itemname)){ \
    std::cout << "[" << #itemname << "]\t" << vm[#itemname].as<itemtpye>() << std::endl;  \
    itemname = vm[#itemname].as<itemtpye>();                        \
}else {         \
    if (must) {\
          std::cout << "����Ĳ�����δ�ҵ�" << #itemname << "ѡ�" << std::endl;           \
          return false;\
                                                    }else{\
            itemname = def;\
                                                    }\
 }\
setting.itemname = itemname;

namespace pct
{

    bool combineTrainXmlFiles(std::vector<std::string> xmls, std::string dst_xml);
}




