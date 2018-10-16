@echo off

set classifexe=PointCloudTool.exe
set traindir=classdir
set classifdirs=testdata

if not exist %traindir%\config.xml (
	@echo 没有找到文件：%traindir%\config.xml，在执行分类前，请先训练样本文件！
	exit
)

for %%d in (%classifdirs%) do (
    @echo %%d

	for  %%f in (%%d\*.las) do (
		 @echo %%f
		 %classifexe% --cmdtype distancecheck --inputfile %%f --classdir %traindir%
		 
		 @echo %%f end
	)
	
)


pause