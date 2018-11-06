@echo off

set classifexe=PointCloudTool.exe
set traindir=classdir
set classifdirs=testdata
set exceldir = testdata\������ƫ����

if not exist %traindir%\config.xml (
	@echo û���ҵ��ļ���%traindir%\config.xml����ִ�з���ǰ������ѵ�������ļ���
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