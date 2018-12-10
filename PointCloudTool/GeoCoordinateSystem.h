#pragma once
//��������ϵ����������������
class CGeoCoordinateSystem
{
public:
	CGeoCoordinateSystem();

	//dSemimajorAxis ������
	//dInverseFlattening ���ʵ���
	CGeoCoordinateSystem(double dSemimajorAxis, double dInverseFlattening);
	void set(double dSemimajorAxis, double dInverseFlattening);

	//�ɾ�γ��(��λ��)�õ���������(��λm)
	static void LonLat2ECI(double dLon, double dLat, double dHeight, double& x, double& y, double& z, const CGeoCoordinateSystem& gcs = WGS84);

	//�ɵ�������(��λm)�õ���γ��(��λ��)
	static void ECI2LonLat(double x, double y, double z, double& dLon, double& dLat, double& dHeight, const CGeoCoordinateSystem& gcs = WGS84);

	// Summary: 
	//   �õ�ĳ�ο��㴦����������ox,oy,oz��ƽ�����굽��������ı任����
	// Parameters:
	//   ox  [IN] �ο���x����(��������ϵ)
	//   oy  [IN] �ο���y����(��������ϵ)
	//   oz  [IN] �ο���z����(��������ϵ)
	//   bWithPoint [IN] ���صı任�����У��Ƿ��������(ox,oy,oz)��ƽ�ơ����������������任�󣬻�Ҫ����(ox,oy,oz)���������ĵ������ꡣ
	//   matEntry  [OUT] ���ر任����
	//   gcs  [IN] ��������ϵ����������������
	// Remarks:
	//       Ӧ�ó�������ƽ����������������ת�������У�Ϊ�˼��٣�ͨ������������е㶼ִ�С�ƽ������<->��γ��<->�������ꡱ�任�����ǰ�һС�����ݵ���һ��
	//   ���壬ֻ��������ĵ�������任��Ȼ���øú�������任������������Ա任���󼴿ɡ���Ϊ��Χͨ����̫�󣨼���m��������ǿ��Խ��ܵġ�
	//       ���ؾ��������󣬼��òο��㴦���������굽ƽ������ı任����
	static void EastNorthUpToFixedFrame(double ox, double oy, double oz, bool bWithPoint, double matEntry[4][4], const CGeoCoordinateSystem& gcs = WGS84);

public:
	static CGeoCoordinateSystem Beijing54;
	static CGeoCoordinateSystem XiAn80;
	static CGeoCoordinateSystem WGS84;

	double a;  //������
	double b;  //�̰���
	double f;  //���ʵ��� (a/(a-b))
	double e;  //��һƫ���� sqrt(1-pow(b/a,2))
	double ep;  //�ڶ�ƫ���� sqrt(pow(a/b,2)-1)

	//���������Ϊ���������
public:
	double aSquare;      //a*a
	double bSquare;      //b*b
	double e1;        //(1-b/a)/(1+b/a)
	double jDenominator;  //(a(1-pow(e,2)/4-3pow(e,4)/64-5pow(e,6)/256)
	double BfCoefficient1;  //(3*e1/2-27pow(e1,3)/32)
	double BfCoefficient2;  //(21pow(e1,2)/16-55pow(e1,4)/32)
	double BfCoefficient3;  //(151pow(e1,3)/96)
	double MCoefficient1;  //(1-pow(e,2)/4-3pow(e,4)/64-5pow(e,6)/256)
	double MCoefficient2;  //(3pow(e,2)/8+3pow(e,4)/32+45pow(e,6)/1024)
	double MCoefficient3;  //(15pow(e,4)/256+45pow(e,6)/1024)
	double MCoefficient4;  //35*pow(e,6)/3072
};

//ͶӰ����ϵ������ͶӰ����
class CProCoordinateSystem : public CGeoCoordinateSystem
{
public:
	CProCoordinateSystem();

	//gcs ��������ϵ
	//LC ����������
	//FE α��
	//FN α��
	//k0 ���������߱���ϵ��
	CProCoordinateSystem(const CGeoCoordinateSystem& gcs, double LC, double FE = 500000, double FN = 0, double k0 = 1);
	void set(const CGeoCoordinateSystem& gcs, double LC, double FE, double FN, double k0);

	//��˹����ϵX���ϱ���Y��������ת��������ϵ��γ��
	static void GaussToGeo(double y, double x, double &L, double &B, const CProCoordinateSystem& gcs);

	//��������ϵ��γ��ת��˹����ϵX���ϱ���Y��������
	static void GeoToGauss(double L, double B, double &y, double &x, const CProCoordinateSystem& gcs);

public:
	double LC;  //����������
	double FE;  //��γƫ�ƣ�α����
	double FN;  //��γƫ�ƣ�α����
	double k0;  //��������
};