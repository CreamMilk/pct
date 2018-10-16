@echo off

set classifexe=PointCloudTool.exe
set traindir=classdir
set classifdirs=testdata

if not exist %traindir%\config.xml (
	@echo û���ҵ��ļ���%traindir%\config.xml����ִ�з���ǰ������ѵ�������ļ���
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