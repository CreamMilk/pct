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
		 %classifexe% --inputfile %%f --classdir %traindir% --retrain false --method 2 --dbscaneps 2 --dbscanmin 700
		 
		 @echo %%f end
	)
	
)


pause