#pragma once

#include <string>
#ifdef MY_DLL_API

#else

#define MY_DLL_API  _declspec(dllimport)

#endif


class MY_DLL_API Las2Pnts
{
public:
	Las2Pnts();
	~Las2Pnts();

	bool run(std::string input_las, std::string output_pnts);
};

