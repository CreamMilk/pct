#pragma once
//地理坐标系，定义地球椭球参数
class CGeoCoordinateSystem
{
public:
	CGeoCoordinateSystem();

	//dSemimajorAxis 长半轴
	//dInverseFlattening 扁率倒数
	CGeoCoordinateSystem(double dSemimajorAxis, double dInverseFlattening);
	void set(double dSemimajorAxis, double dInverseFlattening);

	//由经纬度(单位度)得到地心坐标(单位m)
	static void LonLat2ECI(double dLon, double dLat, double dHeight, double& x, double& y, double& z, const CGeoCoordinateSystem& gcs = WGS84);

	//由地心坐标(单位m)得到经纬度(单位度)
	static void ECI2LonLat(double x, double y, double z, double& dLon, double& dLat, double& dHeight, const CGeoCoordinateSystem& gcs = WGS84);

	// Summary: 
	//   得到某参考点处（地心坐标ox,oy,oz）平面坐标到地心坐标的变换矩阵
	// Parameters:
	//   ox  [IN] 参考点x坐标(地心坐标系)
	//   oy  [IN] 参考点y坐标(地心坐标系)
	//   oz  [IN] 参考点z坐标(地心坐标系)
	//   bWithPoint [IN] 返回的变换矩阵中，是否包含到点(ox,oy,oz)的平移。如果不包含，矩阵变换后，还要加上(ox,oy,oz)才是真正的地心坐标。
	//   matEntry  [OUT] 返回变换矩阵
	//   gcs  [IN] 地理坐标系，定义地球椭球参数
	// Remarks:
	//       应用场景：在平面坐标与地心坐标的转换过程中，为了加速，通常不是针对所有点都执行“平面坐标<->经纬度<->地心坐标”变换，而是把一小块数据当作一个
	//   整体，只针对其中心点做这个变换。然后用该函数求出变换矩阵，其它点乘以变换矩阵即可。因为范围通常不太大（几百m），误差是可以接受的。
	//       返回矩阵的逆矩阵，即该参考点处，地心坐标到平面坐标的变换矩阵。
	static void EastNorthUpToFixedFrame(double ox, double oy, double oz, bool bWithPoint, double matEntry[4][4], const CGeoCoordinateSystem& gcs = WGS84);

public:
	static CGeoCoordinateSystem Beijing54;
	static CGeoCoordinateSystem XiAn80;
	static CGeoCoordinateSystem WGS84;

	double a;  //长半轴
	double b;  //短半轴
	double f;  //扁率倒数 (a/(a-b))
	double e;  //第一偏心率 sqrt(1-pow(b/a,2))
	double ep;  //第二偏心率 sqrt(pow(a/b,2)-1)

	//下面各变量为计算加速用
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

//投影坐标系，定义投影参数
class CProCoordinateSystem : public CGeoCoordinateSystem
{
public:
	CProCoordinateSystem();

	//gcs 地理坐标系
	//LC 中央子午线
	//FE 伪东
	//FN 伪北
	//k0 中央子午线比例系数
	CProCoordinateSystem(const CGeoCoordinateSystem& gcs, double LC, double FE = 500000, double FN = 0, double k0 = 1);
	void set(const CGeoCoordinateSystem& gcs, double LC, double FE, double FN, double k0);

	//高斯坐标系X（南北）Y（东西）转地理坐标系经纬度
	static void GaussToGeo(double y, double x, double &L, double &B, const CProCoordinateSystem& gcs);

	//地理坐标系经纬度转高斯坐标系X（南北）Y（东西）
	static void GeoToGauss(double L, double B, double &y, double &x, const CProCoordinateSystem& gcs);

public:
	double LC;  //中央子午线
	double FE;  //东纬偏移（伪东）
	double FN;  //北纬偏移（伪北）
	double k0;  //比例因子
};