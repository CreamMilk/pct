#include <math.h>
#include "GeoCoordinateSystem.h"
#include <memory.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HyGeVector3d
class HyGeVector3d
{
public:
	HyGeVector3d() {}
	HyGeVector3d(double xx, double yy, double zz) { x = xx; y = yy; z = zz; }
	void set(double xx, double yy, double zz) { x = xx; y = yy; z = zz; }

	double length() const { return sqrt(x * x + y * y + z * z); }

	double dotProduct(const HyGeVector3d& v) const { return x * v.x + y * v.y + z * v.z; }

	HyGeVector3d& normalize()
	{
		double dLen = length();
		x /= dLen;
		y /= dLen;
		z /= dLen;
		return *this;
	}

	HyGeVector3d crossProduct(const HyGeVector3d& v) const
	{
		return HyGeVector3d(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
	}

	HyGeVector3d operator * (const HyGeVector3d& v) const
	{
		return HyGeVector3d(x*v.x, y*v.y, z*v.z);
	}

	void operator /= (double v)
	{
		x /= v; y /= v; z /= v;
	}

	void operator *= (double v)
	{
		x *= v; y *= v; z *= v;
	}

public:
	double         x, y, z;
};

namespace MyMathUtils
{
	//角度转弧度
	double DEG2RAD(double dDeg);

	//弧度转角度
	double RAD2DEG(double dRad);

	//来自Cesium 代码，看不懂原理
	HyGeVector3d ScaleToGeodeticSurface(const HyGeVector3d& cartesian, const HyGeVector3d& oneOverRadii, const HyGeVector3d& oneOverRadiiSquared,
		double centerToleranceSquared);

	//来自Cesium 代码，看不懂原理
	HyGeVector3d geodeticSurfaceNormal(const HyGeVector3d& ptCenter, const CGeoCoordinateSystem& gcs);
};

double MyMathUtils::DEG2RAD(double dDeg)
{
	return (dDeg * 3.1415926535897932 / 180.0);
}

double MyMathUtils::RAD2DEG(double dRad)
{
	return (dRad * 180.0 / 3.1415926535897932);
}

HyGeVector3d MyMathUtils::ScaleToGeodeticSurface(const HyGeVector3d& cartesian, const HyGeVector3d& oneOverRadii, const HyGeVector3d& oneOverRadiiSquared,
	double centerToleranceSquared)
{
	double positionX = cartesian.x;
	double positionY = cartesian.y;
	double positionZ = cartesian.z;

	double oneOverRadiiX = oneOverRadii.x;
	double oneOverRadiiY = oneOverRadii.y;
	double oneOverRadiiZ = oneOverRadii.z;

	double x2 = positionX * positionX * oneOverRadiiX * oneOverRadiiX;
	double y2 = positionY * positionY * oneOverRadiiY * oneOverRadiiY;
	double z2 = positionZ * positionZ * oneOverRadiiZ * oneOverRadiiZ;

	// Compute the squared ellipsoid norm.
	double squaredNorm = x2 + y2 + z2;
	double ratio = sqrt(1.0 / squaredNorm);

	// As an initial approximation, assume that the radial intersection is the projection point.
	HyGeVector3d intersection = cartesian;
	intersection *= ratio;

	// If the position is near the center, the iteration will not converge.
	if (squaredNorm < centerToleranceSquared) {
		return intersection;//!isFinite(ratio) ? undefined : Cartesian3.clone(intersection, result);
	}

	double oneOverRadiiSquaredX = oneOverRadiiSquared.x;
	double oneOverRadiiSquaredY = oneOverRadiiSquared.y;
	double oneOverRadiiSquaredZ = oneOverRadiiSquared.z;

	// Use the gradient at the intersection point in place of the true unit normal.
	// The difference in magnitude will be absorbed in the multiplier.
	HyGeVector3d gradient;// = scaleToGeodeticSurfaceGradient;
	gradient.x = intersection.x * oneOverRadiiSquaredX * 2.0;
	gradient.y = intersection.y * oneOverRadiiSquaredY * 2.0;
	gradient.z = intersection.z * oneOverRadiiSquaredZ * 2.0;

	// Compute the initial guess at the normal vector multiplier, lambda.
	//var lambda = (1.0 - ratio) * Cartesian3.magnitude(cartesian) / (0.5 * Cartesian3.magnitude(gradient));
	double lambda = (1.0 - ratio) * cartesian.length() / (0.5 * gradient.length());
	double correction = 0.0;

	double func;
	double denominator;
	double xMultiplier;
	double yMultiplier;
	double zMultiplier;
	double xMultiplier2;
	double yMultiplier2;
	double zMultiplier2;
	double xMultiplier3;
	double yMultiplier3;
	double zMultiplier3;

	do {
		lambda -= correction;

		xMultiplier = 1.0 / (1.0 + lambda * oneOverRadiiSquaredX);
		yMultiplier = 1.0 / (1.0 + lambda * oneOverRadiiSquaredY);
		zMultiplier = 1.0 / (1.0 + lambda * oneOverRadiiSquaredZ);

		xMultiplier2 = xMultiplier * xMultiplier;
		yMultiplier2 = yMultiplier * yMultiplier;
		zMultiplier2 = zMultiplier * zMultiplier;

		xMultiplier3 = xMultiplier2 * xMultiplier;
		yMultiplier3 = yMultiplier2 * yMultiplier;
		zMultiplier3 = zMultiplier2 * zMultiplier;

		func = x2 * xMultiplier2 + y2 * yMultiplier2 + z2 * zMultiplier2 - 1.0;

		// "denominator" here refers to the use of this expression in the velocity and acceleration
		// computations in the sections to follow.
		denominator = x2 * xMultiplier3 * oneOverRadiiSquaredX + y2 * yMultiplier3 * oneOverRadiiSquaredY + z2 * zMultiplier3 * oneOverRadiiSquaredZ;

		double derivative = -2.0 * denominator;

		correction = func / derivative;
	} while (fabs(func) > 1e-12);

	return HyGeVector3d(positionX * xMultiplier, positionY * yMultiplier, positionZ * zMultiplier);
}

HyGeVector3d MyMathUtils::geodeticSurfaceNormal(const HyGeVector3d& ptCenter, const CGeoCoordinateSystem& gcs)
{
	HyGeVector3d oneOverRadiiSquared(1.0 / gcs.aSquare, 1.0 / gcs.aSquare, 1.0 / gcs.bSquare);
	HyGeVector3d ret = ptCenter * oneOverRadiiSquared;
	ret.normalize();

	return ret;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CGeoCoordinateSystem

CGeoCoordinateSystem CGeoCoordinateSystem::Beijing54(6378245, 298.3);
CGeoCoordinateSystem CGeoCoordinateSystem::XiAn80(6378140, 298.257);
CGeoCoordinateSystem CGeoCoordinateSystem::WGS84(6378137, 298.257);

CGeoCoordinateSystem::CGeoCoordinateSystem()
{
	*this = Beijing54;
}

CGeoCoordinateSystem::CGeoCoordinateSystem(double dSemimajorAxis, double dInverseFlattening)
{
	set(dSemimajorAxis, dInverseFlattening);
}

void CGeoCoordinateSystem::set(double dSemimajorAxis, double dInverseFlattening)
{
	a = dSemimajorAxis;
	f = dInverseFlattening;
	b = a*(1 - 1 / f);
	e = sqrt(1 - pow(b / a, 2));
	ep = sqrt(pow(a / b, 2) - 1);

	aSquare = a*a;
	bSquare = b*b;

	e1 = (1 - b / a) / (1 + b / a);
	jDenominator = a*(1 - pow(e, 2) / 4 - 3 * pow(e, 4) / 64 - 5 * pow(e, 6) / 256);
	BfCoefficient1 = 3 * e1 / 2 - 27 * pow(e1, 3) / 32;
	BfCoefficient2 = 21 * pow(e1, 2) / 16 - 55 * pow(e1, 4) / 32;
	BfCoefficient3 = 151 * pow(e1, 3) / 96;
	MCoefficient1 = 1 - pow(e, 2) / 4 - 3 * pow(e, 4) / 64 - 5 * pow(e, 6) / 256;
	MCoefficient2 = 3 * pow(e, 2) / 8 + 3 * pow(e, 4) / 32 + 45 * pow(e, 6) / 1024;
	MCoefficient3 = 15 * pow(e, 4) / 256 + 45 * pow(e, 6) / 1024;
	MCoefficient4 = 35 * pow(e, 6) / 3072;
}

//由经纬度(单位度)得到地心坐标(单位m)。代码来自Cesium js，原理看不明白
void CGeoCoordinateSystem::LonLat2ECI(double dLon, double dLat, double dHeight, double& x, double& y, double& z, const CGeoCoordinateSystem& gcs)
{
	HyGeVector3d radiiSquared(gcs.aSquare, gcs.aSquare, gcs.bSquare);
	dLon = MyMathUtils::DEG2RAD(dLon);
	dLat = MyMathUtils::DEG2RAD(dLat);

	double cosLatitude = cos(dLat);

	//地心指向该经纬度的方向单位向量
	HyGeVector3d scratchN;
	scratchN.x = cosLatitude * cos(dLon);
	scratchN.y = cosLatitude * sin(dLon);
	scratchN.z = sin(dLat);
	scratchN.normalize();

	HyGeVector3d scratchK = radiiSquared * scratchN;
	double gamma = sqrt(scratchN.dotProduct(scratchK));
	scratchK /= gamma;
	scratchN *= dHeight;	//高程的影响

	x = scratchK.x + scratchN.x;
	y = scratchK.y + scratchN.y;
	z = scratchK.z + scratchN.z;
}

void LonLat2ECI1(double dLon, double dLat, double dHeight, double& x, double& y, double& z, const CGeoCoordinateSystem& gcs)
{
	HyGeVector3d radiiSquared(gcs.aSquare, gcs.aSquare, gcs.bSquare);
	dLon = MyMathUtils::DEG2RAD(dLon);
	dLat = MyMathUtils::DEG2RAD(dLat);

	double cosLatitude = cos(dLat);

	//地心指向该经纬度的方向单位向量
	HyGeVector3d scratchN;
	scratchN.x = cosLatitude * cos(dLon);
	scratchN.y = cosLatitude * sin(dLon);
	scratchN.z = sin(dLat);
	scratchN.normalize();

	HyGeVector3d scratchK = radiiSquared * scratchN;
	double gamma = sqrt(scratchN.dotProduct(scratchK));
	scratchK /= gamma;
	scratchN *= dHeight;	//高程的影响

	x = scratchK.x + scratchN.x;
	y = scratchK.y + scratchN.y;
	z = scratchK.z + scratchN.z;
}

//由地心坐标(单位m)得到经纬度(单位度)。代码来自Cesium js，原理看不明白
void CGeoCoordinateSystem::ECI2LonLat(double x, double y, double z, double& dLon, double& dLat, double& dHeight, const CGeoCoordinateSystem& gcs)
{
	HyGeVector3d oneOverRadii(1.0 / gcs.a, 1.0 / gcs.a, 1.0 / gcs.b);
	HyGeVector3d oneOverRadiiSquared(1.0 / gcs.aSquare, 1.0 / gcs.aSquare, 1.0 / gcs.bSquare);
	double centerToleranceSquared = 0.00001;//CesiumMath.EPSILON1;

	HyGeVector3d cartesian(x, y, z);

	//`cartesian is required.` is thrown from scaleToGeodeticSurface
	HyGeVector3d p = MyMathUtils::ScaleToGeodeticSurface(cartesian, oneOverRadii, oneOverRadiiSquared, centerToleranceSquared);

	//Cartesian3.multiplyComponents(cartesian, oneOverRadiiSquared, cartesianToCartographicN);
	HyGeVector3d n = cartesian * oneOverRadiiSquared;
	n.normalize();

	HyGeVector3d h(cartesian.x - p.x, cartesian.y - p.y, cartesian.z - p.z);// = Cartesian3.subtract(cartesian, p, cartesianToCartographicH);

	dLon = atan2(n.y, n.x);
	dLat = asin(n.z);
	dHeight = (h.dotProduct(cartesian) > 0 ? 1 : -1) * h.length(); //sign(Cartesian3.dot(h, cartesian)) * Cartesian3.magnitude(h);

	dLon = MyMathUtils::RAD2DEG(dLon);
	dLat = MyMathUtils::RAD2DEG(dLat);
}

//得到某点处（地心坐标）平面坐标到地心坐标的变换矩阵。代码来自Cesium js，原理看不明白
void CGeoCoordinateSystem::EastNorthUpToFixedFrame(double ox, double oy, double oz, bool bWithPoint, double matEntry[4][4], const CGeoCoordinateSystem& gcs)
{
	HyGeVector3d center(ox, oy, oz);
	HyGeVector3d normal = MyMathUtils::geodeticSurfaceNormal(center, gcs);

	HyGeVector3d tangent;
	tangent.x = -center.y;
	tangent.y = center.x;
	tangent.z = 0.0;
	tangent.normalize();

	HyGeVector3d bitangent = normal.crossProduct(tangent);

	if (bWithPoint)
	{
		double entry[4][4] = {
			tangent.x, bitangent.x, normal.x, center.x, \
			tangent.y, bitangent.y, normal.y, center.y, \
			tangent.z, bitangent.z, normal.z, center.z, \
			0, 0, 0, 1 };

		memcpy(matEntry, entry, 16 * sizeof(double));
	}
	else
	{
		double entry[4][4] = {
			tangent.x, bitangent.x, normal.x, 0, \
			tangent.y, bitangent.y, normal.y, 0, \
			tangent.z, bitangent.z, normal.z, 0, \
			0, 0, 0, 1 };

		memcpy(matEntry, entry, 16 * sizeof(double));
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CProCoordinateSystem
CProCoordinateSystem::CProCoordinateSystem()
{
	set(CGeoCoordinateSystem::Beijing54, 120, 500000, 0, 1);
}

CProCoordinateSystem::CProCoordinateSystem(const CGeoCoordinateSystem& gcs, double LC, double FE, double FN, double k0)
{
	set(gcs, LC, FE, FN, k0);
}

void CProCoordinateSystem::set(const CGeoCoordinateSystem& gcs, double LC, double FE, double FN, double k0)
{
	CGeoCoordinateSystem* p = (CGeoCoordinateSystem*)this;
	*p = gcs;

	this->LC = LC;
	this->FE = FE;
	this->FN = FN;
	this->k0 = k0;
}

// y:     高斯坐标的横坐标，以米为单位，不带500000偏移
// x:	  高斯坐标的纵坐标，以米为单位
// L:     经度，以度为单位
// B:     纬度，以度为单位
//计算公式:
//B = Bf - Nf*tgBf/Rf * (pow(D,2)/2-(5+3Tf+Cf-9TfCf))pow(D,4)/24 + (61+90Tf+45pow(Tf,2))pow(D,6)/720)
//L = LC+(D-(1+2Tf+Cf)pow(D,3)/6+(5+28Tf+6Cf+8TfCf+24pow(Tf,2))pow(D,5)/120)/cosBf
//a = 椭球体长半轴
//b = 椭球体短半轴
//f = 扁率倒数(a/(a-b))
//e = 第一偏心率 sqrt(1-pow(b/a,2))
//ep= 第二偏心率 sqrt(pow(a/b,2)-1)
//Nf = a/sqrt(1-pow(e,2)*pow(sinBf,2))
//Rf = a(1-pow(e,2))/pow(1-pow(e,2)*pow(sinBf,2), 1.5)
//Bf = j+(3*e1/2-27pow(e1,3)/32)sin(2j)+(21pow(e1,2)/16-55pow(e1,4)/32)sin(4j)+(151pow(e1,3)/96)sin(6j)
//e1 = (1-b/a)/(1+b/a)
//j  = ((x-FN)/k0)/(a(1-pow(e,2)/4-3pow(e,4)/64-5pow(e,6)/256)
//Tf = pow(tgBf, 2)
//Cf = pow(ep,2)*pow(cosBf,2)
//D  = (y-FE)/(k0*Nf)
//北纬偏移 FN北半球= 0，FN南半球= 10000000米
void CProCoordinateSystem::GaussToGeo(double y, double x, double &L, double &B, const CProCoordinateSystem& pcs)
{
	double PI = 3.14159265358979;

	double j = ((x - pcs.FN) / pcs.k0) / pcs.jDenominator;
	double Bf = j + pcs.BfCoefficient1 * sin(2 * j);
	Bf += pcs.BfCoefficient2 * sin(4 * j);
	Bf += pcs.BfCoefficient3 * sin(6 * j);

	double Tf = pow(tan(Bf), 2);
	double Cf = pow(pcs.ep, 2)*pow(cos(Bf), 2);
	double Rf = pcs.a*(1 - pow(pcs.e, 2)) / pow(1 - pow(pcs.e, 2)*pow(sin(Bf), 2), 1.5);
	double Nf = pcs.a / sqrt(1 - pow(pcs.e, 2)*pow(sin(Bf), 2));
	double D = (y - pcs.FE) / (pcs.k0*Nf);

	B = Bf * 180 / PI - Nf*tan(Bf) / Rf * (pow(D, 2) / 2 - (5 + 3 * Tf + Cf - 9 * Tf*Cf)*pow(D, 4) / 24 + (61 + 90 * Tf + 45 * pow(Tf, 2))*pow(D, 6) / 720) * 180 / PI;
	L = pcs.LC + (D - (1 + 2 * Tf + Cf)*pow(D, 3) / 6 + (5 + 28 * Tf + 6 * Cf + 8 * Tf*Cf + 24 * pow(Tf, 2))*pow(D, 5) / 120) / cos(Bf) * 180 / PI;
}

/* 功能说明：
（1）将地理坐标(wd,jd)转换成绝对的高斯坐标(y,x)
（2）本函数支持基于六度带（或三度带）、克拉索夫斯基椭球进行转换                             */
// L:		地理坐标的经度，以度为单位
// B:		地理坐标的纬度，以度为单位
// y:		横坐标，单位m
// x:		纵坐标，单位m
//计算公式:
//X = k0(M+NtgB(pow(A,2)/2+(5-T+9C+4pow(C,2))pow(A,4)/24)+(61-58T+pow(T,2)+270C-330TC)pow(A,6)/720)
//Y = FE+k0*N(A+(1-T+C)pow(A,3)/6+(5-18T+pow(T,2)+14C-58TC)pow(A,5)/120)
//a = 椭球体长半轴
//b = 椭球体短半轴
//f = 扁率倒数(a/(a-b))
//e = 第一偏心率 sqrt(1-pow(b/a,2))
//ep= 第二偏心率 sqrt(pow(a/b,2)-1)
//T = pow(tgB,2)
//C = pow(ep,2)pow(cosB,2)
//A = (L-LC)cosB
//M = a(  (1-pow(e,2)/4-3pow(e,4)/64-5pow(e,6)/256)B
//		 -(3pow(e,2)/8+3pow(e,4)/32+45pow(e,6)/1024)sin2B
//		 +(15pow(e,4)/256+45pow(e,6)/1024)sin4B
//		 -35sin6B*pow(e,6)/3072 )
//N = a/sqrt(1-pow(e,2)pow(sinB,2))
void CProCoordinateSystem::GeoToGauss(double L, double B, double& y, double& x, const CProCoordinateSystem& pcs)
{
	double PI = 3.14159265358979;

	double T = pow(tan(B*PI / 180), 2);
	double C = pow(pcs.ep*cos(B*PI / 180), 2);
	double A = (L - pcs.LC)*cos(B*PI / 180)*PI / 180;

	double M = pcs.MCoefficient1 * B * PI / 180;
	M -= pcs.MCoefficient2 * sin(2 * B*PI / 180);
	M += pcs.MCoefficient3 * sin(4 * B*PI / 180);
	M -= pcs.MCoefficient4 * sin(6 * B*PI / 180);
	M *= pcs.a;
	double N = pcs.a / sqrt(1 - pow(pcs.e, 2)*pow(sin(B*PI / 180), 2));

	x = M + N*tan(B*PI / 180)*(pow(A, 2) / 2 + (5 - T + 9 * C + 4 * pow(C, 2))*pow(A, 4) / 24);
	x += (61 - 58 * T + pow(T, 2) + 270 * C - 330 * T*C)*pow(A, 6) / 720;
	x *= pcs.k0;
	x += pcs.FN;

	y = pcs.FE + pcs.k0*N*(A + (1 - T + C)*pow(A, 3) / 6 + (5 - 18 * T + pow(T, 2) + 14 * C - 58 * T*C)*pow(A, 5) / 120);
}