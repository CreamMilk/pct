@echo off

set classifexe=PointCloudTool.exe
set traindir=classdir
set classifdirs=testdata
set exceldir = testdata\铁塔纠偏数据

if not exist %traindir%\config.xml (
	@echo 没有找到文件：%traindir%\config.xml，在执行分类前，请先训练样本文件！
	exit
)

for %%d in (%classifdirs%) do (
    @echo %%d

	for  %%f in (%%d\*.las) do (
		 @echo %%f
		 %classifexe%  --cmdtype poscorrect --inputfile %%f --classdir %traindir% --method 2 --exceldir %exceldir% --overrideExcel false --stampcorrectcell true
		 @echo %%f end
	)
	
)


pause