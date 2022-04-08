/*! @file rx_matrix.h
	
	@brief ベクトル・行列ライブラリ - 行列クラス
 
	@author 
	@date  
*/

#ifndef _MATRIX_H_
#define _MATRIX_H_

//-----------------------------------------------------------------------------
// インクルードファイル
//-----------------------------------------------------------------------------
#include <memory.h>
#include "rx_utility.h"

//-----------------------------------------------------------------------------
// 定義
//-----------------------------------------------------------------------------
#define real double 

#define QUATERNION_NORMALIZATION_THRESHOLD  64

#ifndef RX_INV_PI
#define RX_INV_PI 0.318309886
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG  (real)(57.2957795130823208767981548141052)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD  (real)(0.0174532925199432957692369076848861)
#endif

#ifndef RX_EPSILON
#define RX_EPSILON     (real)(10e-6)
#endif

#ifndef RX_PI
#define RX_PI          (real)(3.1415926535897932384626433832795)    
#endif

#define equivalent(a,b)     (((a < b + RX_EPSILON) && (a > b - RX_EPSILON)) ? true : false)

//-----------------------------------------------------------------------------
// MARK:4x4行列クラス
//-----------------------------------------------------------------------------
class Mat4x4
{
protected:
	real m[16];

public:
	//! コンストラクタ
	Mat4x4(){ MakeIdentity(); }
	Mat4x4(real r){ SetValue(r); }
	Mat4x4(real *m){ SetValue(m); }
	Mat4x4(real a00, real a01, real a02, real a03,
			real a10, real a11, real a12, real a13,
			real a20, real a21, real a22, real a23,
			real a30, real a31, real a32, real a33)
	{
		element(0,0) = a00;
		element(0,1) = a01;
		element(0,2) = a02;
		element(0,3) = a03;
		
		element(1,0) = a10;
		element(1,1) = a11;
		element(1,2) = a12;
		element(1,3) = a13;
		
		element(2,0) = a20;
		element(2,1) = a21;
		element(2,2) = a22;
		element(2,3) = a23;
		
		element(3,0) = a30;
		element(3,1) = a31;
		element(3,2) = a32;
		element(3,3) = a33;
	}

	//! 行列をmpに入れる
	void GetValue(real *mp) const
	{
		int c = 0;
		for(int j = 0; j < 4; ++j){
			for(int i = 0; i < 4; ++i){
				mp[c++] = element(i,j);
			}
		}
	}

	//! 行列をmpに入れる
	template<typename T> 
	void GetValueT(T *mp) const
	{
		int c = 0;
		for(int j = 0; j < 4; ++j){
			for(int i = 0; i < 4; ++i){
				mp[c++] = (T)element(i,j);
			}
		}
	}

	//! 行列をポインタで取得
	real* GetValue(){ return m; }

	//! mpから行列に値を代入
	void SetValue(real *mp)
	{
		int c = 0;
		for(int j = 0; j < 4; ++j){
			for(int i = 0; i < 4; ++i){
				element(i,j) = mp[c++];
			}
		}
	}

	//! mpから行列に値を代入
	template<typename T> 
	void SetValueT(T *mp)
	{
		int c = 0;
		for(int j = 0; j < 4; ++j){
			for(int i = 0; i < 4; ++i){
				element(i,j) = (T)mp[c++];
			}
		}
	}

	//! 行列のすべての値をrにする
	void SetValue(real r)
	{
		for(int i = 0; i < 4; ++i){
			for(int j = 0; j < 4; ++j){
				element(i,j) = r;
			}
		}
	}

	//! 単位行列生成
	void MakeIdentity()
	{
		element(0,0) = 1.0;
		element(0,1) = 0.0;
		element(0,2) = 0.0; 
		element(0,3) = 0.0;
		
		element(1,0) = 0.0;
		element(1,1) = 1.0; 
		element(1,2) = 0.0;
		element(1,3) = 0.0;
		
		element(2,0) = 0.0;
		element(2,1) = 0.0;
		element(2,2) = 1.0;
		element(2,3) = 0.0;
		
		element(3,0) = 0.0; 
		element(3,1) = 0.0; 
		element(3,2) = 0.0;
		element(3,3) = 1.0;
	}

	//! 単位行列を返す
	static Mat4x4 Identity()
	{
		static Mat4x4 mident(
			1.0, 0.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0);

		return mident;
	}

	//! スケール(対角成分)を設定	    
	void SetScale(real s)
	{
		element(0,0) = s;
		element(1,1) = s;
		element(2,2) = s;
	}

	//! スケール(対角成分)を設定	    
	void SetScale(const Vec3 &s)
	{
		element(0,0) = s[0];
		element(1,1) = s[1];
		element(2,2) = s[2];
	}

	//! 平行移動成分(0,3)-(0,2)を設定
	void SetTranslate(const Vec3 &t)
	{
		element(0,3) = t[0];
		element(1,3) = t[1];
		element(2,3) = t[2];
	}

	//! r行成分を設定
	void SetRow(int r, const Vec4 &t)
	{
		element(r,0) = t[0];
		element(r,1) = t[1];
		element(r,2) = t[2];
		element(r,3) = t[3];
	}

	//! c列成分を設定
	void SetColumn(int c, const Vec4&t)
	{
		element(0,c) = t[0];
		element(1,c) = t[1];
		element(2,c) = t[2];
		element(3,c) = t[3];
	}

	//! r行成分取得
	void GetRow(int r, Vec4&t) const
	{
		t[0] = element(r,0);
		t[1] = element(r,1);
		t[2] = element(r,2);
		t[3] = element(r,3);
	}

	//! r行成分取得
	Vec4 GetRow(int r) const
	{
		Vec4 v;
		GetRow(r, v);
		return v;
	}

	//! c列成分取得
	void GetColumn(int c, Vec4&t) const
	{
		t[0] = element(0,c);
		t[1] = element(1,c);
		t[2] = element(2,c);
		t[3] = element(3,c);
	}

	//! c列成分取得
	Vec4 GetColumn(int c) const
	{
		Vec4 v;
		GetColumn(c, v);
		return v;
	}

	//! 逆行列取得
	Mat4x4 Inverse() const
	{
		Mat4x4 minv;
		
		real r1[8], r2[8], r3[8], r4[8];
		real *s[4], *tmprow;
		
		s[0] = &r1[0];
		s[1] = &r2[0];
		s[2] = &r3[0];
		s[3] = &r4[0];
		
		int i, j, p, jj;
		for(i = 0; i < 4; ++i){
			for(j = 0; j < 4; ++j){
				s[i][j] = element(i, j);

				if(i==j){
					s[i][j+4] = 1.0;
				}
				else{
					s[i][j+4] = 0.0;
				}
			}
		}

		real scp[4];
		for(i = 0; i < 4; ++i){
			scp[i] = real(fabs(s[i][0]));
			for(j=1; j < 4; ++j)
				if(real(fabs(s[i][j])) > scp[i]) scp[i] = real(fabs(s[i][j]));
				if(scp[i] == 0.0) return minv; // singular matrix!
		}
		
		int pivot_to;
		real scp_max;
		for(i = 0; i < 4; ++i){
			// select pivot row
			pivot_to = i;
			scp_max = real(fabs(s[i][i]/scp[i]));
			// find out which row should be on top
			for(p = i+1; p < 4; ++p)
				if(real(fabs(s[p][i]/scp[p])) > scp_max){
					scp_max = real(fabs(s[p][i]/scp[p])); pivot_to = p;
				}
				// Pivot if necessary
				if(pivot_to != i)
				{
					tmprow = s[i];
					s[i] = s[pivot_to];
					s[pivot_to] = tmprow;
					real tmpscp;
					tmpscp = scp[i];
					scp[i] = scp[pivot_to];
					scp[pivot_to] = tmpscp;
				}
				
				real mji;
				// perform gaussian elimination
				for(j = i+1; j < 4; ++j)
				{
					mji = s[j][i]/s[i][i];
					s[j][i] = 0.0;
					for(jj=i+1; jj<8; jj++)
						s[j][jj] -= mji*s[i][jj];
				}
		}

		if(s[3][3] == 0.0) return minv; // singular matrix!
		
		//
		// Now we have an upper triangular matrix.
		//
		//  x x x x | y y y y
		//  0 x x x | y y y y 
		//  0 0 x x | y y y y
		//  0 0 0 x | y y y y
		//
		//  we'll back substitute to Get the inverse
		//
		//  1 0 0 0 | z z z z
		//  0 1 0 0 | z z z z
		//  0 0 1 0 | z z z z
		//  0 0 0 1 | z z z z 
		//
		
		real mij;
		for(i = 3; i > 0; --i){
			for(j = i-1; j > -1; --j){
				mij = s[j][i]/s[i][i];
				for(jj = j+1; jj < 8; ++jj){
					s[j][jj] -= mij*s[i][jj];
				}
			}
		}
		
		for(i = 0; i < 4; ++i){
			for(j = 0; j < 4; ++j){
				minv(i,j) = s[i][j+4]/s[i][i];
			}
		}
			
		return minv;
	}

	//! 転置行列取得
	Mat4x4 Transpose() const
	{
		Mat4x4 mtrans;
		
		for(int i = 0; i < 4; ++i){
			for(int j = 0; j < 4; ++j){
				mtrans(i,j) = element(j,i);
			}
		}

		return mtrans;
	}

	//! 行列を右側から掛ける
	Mat4x4 &multRight(const Mat4x4 &b)
	{
		Mat4x4 mt(*this);
		SetValue(real(0));

		for(int i = 0; i < 4; ++i){
			for(int j = 0; j < 4; ++j){
				for(int c = 0; c < 4; ++c){
					element(i,j) += mt(i,c)*b(c,j);
				}
			}
		}

		return *this;
	}    

	//! 行列を左側から掛ける
	Mat4x4 &multLeft(const Mat4x4 &b)
	{
		Mat4x4 mt(*this);
		SetValue(real(0));

		for(int i = 0; i < 4; ++i){
			for(int j = 0; j < 4; ++j){
				for(int c = 0; c < 4; ++c){
					element(i,j) += b(i,c)*mt(c,j);
				}
			}
		}

		return *this;
	}

	//! dst = M*src
	void multMatrixVec(const Vec3 &src, Vec3 &dst) const
	{
		real w = (src[0]*element(3,0)+src[1]*element(3,1)+src[2]*element(3,2)+element(3,3));
		
		assert(w != 0.0);

		dst[0] = (src[0]*element(0,0)+src[1]*element(0,1)+src[2]*element(0,2)+element(0,3))/w;
		dst[1] = (src[0]*element(1,0)+src[1]*element(1,1)+src[2]*element(1,2)+element(1,3))/w;
		dst[2] = (src[0]*element(2,0)+src[1]*element(2,1)+src[2]*element(2,2)+element(2,3))/w;
	}

	void multMatrixVec(Vec3 & src_and_dst) const
	{ 
		multMatrixVec(Vec3(src_and_dst), src_and_dst); 
	}


	//! dst = src*M
	void multVecMatrix(const Vec3 &src, Vec3 &dst) const
	{
		real w = (src[0]*element(0,3)+src[1]*element(1,3)+src[2]*element(2,3)+element(3,3));
		
		assert(w != 0.0);

		dst[0] = (src[0]*element(0,0)+src[1]*element(1,0)+src[2]*element(2,0)+element(3,0))/w;
		dst[1] = (src[0]*element(0,1)+src[1]*element(1,1)+src[2]*element(2,1)+element(3,1))/w;
		dst[2] = (src[0]*element(0,2)+src[1]*element(1,2)+src[2]*element(2,2)+element(3,2))/w;
	}
		

	void multVecMatrix(Vec3 & src_and_dst) const
	{ 
		multVecMatrix(Vec3(src_and_dst), src_and_dst);
	}

	//! dst = M*src
	void multMatrixVec(const Vec4 &src, Vec4 &dst) const
	{
		dst[0] = (src[0]*element(0,0)+src[1]*element(0,1)+src[2]*element(0,2)+src[3]*element(0,3));
		dst[1] = (src[0]*element(1,0)+src[1]*element(1,1)+src[2]*element(1,2)+src[3]*element(1,3));
		dst[2] = (src[0]*element(2,0)+src[1]*element(2,1)+src[2]*element(2,2)+src[3]*element(2,3));
		dst[3] = (src[0]*element(3,0)+src[1]*element(3,1)+src[2]*element(3,2)+src[3]*element(3,3));
	}

	void multMatrixVec(Vec4 &src_and_dst) const
	{
		multMatrixVec(Vec4(src_and_dst), src_and_dst);
	}


	//! dst = src*M
	void multVecMatrix(const Vec4 &src, Vec4 &dst) const
	{
		dst[0] = (src[0]*element(0,0)+src[1]*element(1,0)+src[2]*element(2,0)+src[3]*element(3,0));
		dst[1] = (src[0]*element(0,1)+src[1]*element(1,1)+src[2]*element(2,1)+src[3]*element(3,1));
		dst[2] = (src[0]*element(0,2)+src[1]*element(1,2)+src[2]*element(2,2)+src[3]*element(3,2));
		dst[3] = (src[0]*element(0,3)+src[1]*element(1,3)+src[2]*element(2,3)+src[3]*element(3,3));
	}
		

	void multVecMatrix(Vec4 & src_and_dst) const
	{
		multVecMatrix(Vec4(src_and_dst), src_and_dst);
	}


	//! dst = M*src
	void multMatrixDir(const Vec3 &src, Vec3 &dst) const
	{
		dst[0] = (src[0]*element(0,0)+src[1]*element(0,1)+src[2]*element(0,2)) ;
		dst[1] = (src[0]*element(1,0)+src[1]*element(1,1)+src[2]*element(1,2)) ;
		dst[2] = (src[0]*element(2,0)+src[1]*element(2,1)+src[2]*element(2,2)) ;
	}
		

	void multMatrixDir(Vec3 & src_and_dst) const
	{
		multMatrixDir(Vec3(src_and_dst), src_and_dst);
	}


	//! dst = src*M
	void multDirMatrix(const Vec3 &src, Vec3 &dst) const
	{
		dst[0] = (src[0]*element(0,0)+
				  src[1]*element(1,0)+
				  src[2]*element(2,0)) ;
		dst[1] = (src[0]*element(0,1)+
				  src[1]*element(1,1)+
				  src[2]*element(2,1)) ;
		dst[2] = (src[0]*element(0,2)+
				  src[1]*element(1,2)+
				  src[2]*element(2,2)) ;
	}

	void multDirMatrix(Vec3 &src_and_dst) const
	{ 
		multDirMatrix(Vec3(src_and_dst), src_and_dst);
	}

	//! 行列の要素を返す
	real& element(int row, int col){ return m[row | (col<<2)]; }
	const real& element(int row, int col) const { return m[row | (col<<2)]; }


	//! オペレータ
	real& operator()(int row, int col){ return element(row,col); }
	const real& operator()(int row, int col) const	{ return element(row,col); }

	Mat4x4& operator*=(const Mat4x4 &mat)
	{
		multRight(mat);
		return *this;
	}

	Mat4x4& operator*=(const real & r)
	{
		for(int i = 0; i < 4; ++i){
			element(0,i) *= r;
			element(1,i) *= r;
			element(2,i) *= r;
			element(3,i) *= r;
		}

		return *this;
	}

	Mat4x4& operator+=(const Mat4x4 &mat)
	{
		for(int i = 0; i < 4; ++i){
			element(0,i) += mat.element(0,i);
			element(1,i) += mat.element(1,i);
			element(2,i) += mat.element(2,i);
			element(3,i) += mat.element(3,i);
		}

		return *this;
	}

	friend Mat4x4 operator*(const Mat4x4 &m1, const Mat4x4 &m2);
	friend Mat4x4 operator*(const Mat4x4 &m, const real &r);
	friend Mat4x4 operator*(const real &r, const Mat4x4 &m);
	friend Vec3 operator*(const Mat4x4 &m, const Vec3 &v);
	friend bool operator==(const Mat4x4 &m1, const Mat4x4 &m2);
	friend bool operator!=(const Mat4x4 &m1, const Mat4x4 &m2);
};

inline Mat4x4 operator*(const Mat4x4 &m1, const Mat4x4 &m2)
{
	Mat4x4 product;
	
	product = m1;
	product.multRight(m2);
	
	return product;
}

inline Mat4x4 operator*(const Mat4x4 &m1, const real &r)
{
	Mat4x4 product;

	product = m1;
	product *= r;

	return product;
}
inline Mat4x4 operator*(const real &r, const Mat4x4 &m1){ return m1*r; }

inline Vec3 operator*(const Mat4x4 &m, const Vec3 &v)
{
	Vec3 product;
	
	m.multMatrixVec(v, product);
	
	return product;
}

inline Vec4 operator*(const Mat4x4 &m, const Vec4 &v)
{
	Vec4 product;
	
	m.multMatrixVec(v, product);
	
	return product;
}

inline bool operator==(const Mat4x4 &m1, const Mat4x4 &m2)
{
	return (m1(0,0) == m2(0,0) &&
			m1(0,1) == m2(0,1) &&
			m1(0,2) == m2(0,2) &&
			m1(0,3) == m2(0,3) &&
			m1(1,0) == m2(1,0) &&
			m1(1,1) == m2(1,1) &&
			m1(1,2) == m2(1,2) &&
			m1(1,3) == m2(1,3) &&
			m1(2,0) == m2(2,0) &&
			m1(2,1) == m2(2,1) &&
			m1(2,2) == m2(2,2) &&
			m1(2,3) == m2(2,3) &&
			m1(3,0) == m2(3,0) &&
			m1(3,1) == m2(3,1) &&
			m1(3,2) == m2(3,2) &&
			m1(3,3) == m2(3,3));
}

inline bool operator!=(const Mat4x4 &m1, const Mat4x4 &m2)
{
	return !(m1 == m2);
}


inline real* operator&(Mat4x4 &m)
{
	return m.GetValue();
}




//-----------------------------------------------------------------------------
// MARK:3x3行列クラス
//-----------------------------------------------------------------------------
class Mat3x3
{
protected:
	real m[9];

public:
	//! コンストラクタ
	Mat3x3(){ makeIdentity(); }
	Mat3x3(real r){ SetValue(r); }
	Mat3x3(real *m){ SetValue(m); }
	Mat3x3(real a00, real a01, real a02,
			  real a10, real a11, real a12,
			  real a20, real a21, real a22)
	{
		element(0,0) = a00;
		element(0,1) = a01;
		element(0,2) = a02;
		
		element(1,0) = a10;
		element(1,1) = a11;
		element(1,2) = a12;
		
		element(2,0) = a20;
		element(2,1) = a21;
		element(2,2) = a22;
	}

	//! 行列をmpに入れる
	void GetValue(real *mp) const
	{
		int c = 0;
		for(int j = 0; j < 3; ++j){
			for(int i = 0; i < 3; ++i){
				mp[c++] = element(i,j);
			}
		}
	}

	//! 行列をmpに入れる
	void GetValue4x4(real *mp) const
	{
		mp[0]  = element(0,0);
		mp[1]  = element(1,0);
		mp[2]  = element(2,0);
		mp[3]  = 0.0;	  
						  
		mp[4]  = element(0,1);
		mp[5]  = element(1,1);
		mp[6]  = element(2,1);
		mp[7]  = 0.0;	  
						  
		mp[8]  = element(0,2);
		mp[9]  = element(1,2);
		mp[10] = element(2,2);
		mp[11] = 0.0;
	}

	//! 行列をポインタで取得
	real* GetValue(){ return m; }

	//! mpから行列に値を代入
	void SetValue(real *mp)
	{
		int c = 0;
		for(int j = 0; j < 3; ++j){
			for(int i = 0; i < 3; ++i){
				element(i,j) = mp[c++];
			}
		}
	}

	//! mpから行列に値を代入
	void SetValue4x4(real *mp)
	{
		element(0,0) = mp[0] ;
		element(1,0) = mp[1] ;
		element(2,0) = mp[2] ;
				 
		element(0,1)  =mp[4] ;
		element(1,1) = mp[5] ;
		element(2,1) = mp[6] ;

		element(0,2) = mp[8] ;
		element(1,2) = mp[9] ;
		element(2,2) = mp[10];
	}					  
						  
	//! 行列のすべての値をにする
	void SetValue(real r)  
	{					  
		for(int i = 0; i < 3; ++i){
			for(int j = 0;j < 3; ++j){
				element(i,j) = r;
			}			  
		}				  
	}
	void SetValue(real a00, real a01, real a02,
				  real a10, real a11, real a12,
				  real a20, real a21, real a22)
	{
		element(0,0) = a00;
		element(0,1) = a01;
		element(0,2) = a02;
		
		element(1,0) = a10;
		element(1,1) = a11;
		element(1,2) = a12;
		
		element(2,0) = a20;
		element(2,1) = a21;
		element(2,2) = a22;
	}

	//! オイラー角から回転行列を設定
	void SetEuler(const double& yaw, const double& pitch, const double& roll) 
	{
		// yaw is CW around y-axis, pitch is CCW around x-axis, and roll is CW around z-axis
		double cy = cos(yaw); 
		double sy = sin(yaw); 
		double cp = cos(pitch); 
		double sp = sin(pitch); 
		double cr = cos(roll);
		double sr = sin(roll);

		double cc = cy*cr; 
		double cs = cy*sr; 
		double sc = sy*cr; 
		double ss = sy*sr;
		SetValue(cc+sp*ss, cs-sp*sc, -sy*cp,
				 -cp*sr,   cp*cr,    -sp,
				 sc-sp*cs, ss+sp*cc, cy*cp);
					
	}

	//! 単位行列生成
	void makeIdentity()
	{
		element(0,0) = 1.0;
		element(0,1) = 0.0;
		element(0,2) = 0.0; 
		
		element(1,0) = 0.0;
		element(1,1) = 1.0; 
		element(1,2) = 0.0;
		
		element(2,0) = 0.0;
		element(2,1) = 0.0;
		element(2,2) = 1.0;
	}

	//! 単位行列を返す
	static Mat3x3 Identity()
	{
		static Mat3x3 mident(
			1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0);

		return mident;
	}

	//! スケール(対角成分)を設定	    
	void SetScale(real s)
	{
		element(0,0) = s;
		element(1,1) = s;
		element(2,2) = s;
	}

	//! スケール(対角成分)を設定	    
	void SetScale(const Vec3 &s)
	{
		element(0,0) = s[0];
		element(1,1) = s[1];
		element(2,2) = s[2];
	}

	//! 平行移動成分(0,3)-(0,2)を設定
	void SetTranslate(const Vec3 &t)
	{
//		element(0,3) = t[0];
//		element(1,3) = t[1];
//		element(2,3) = t[2];
	}

	//! r行成分を設定
	void SetRow(int r, const Vec3 &t)
	{
		element(r,0) = t[0];
		element(r,1) = t[1];
		element(r,2) = t[2];
	}

	//! c列成分を設定
	void SetColumn(int c, const Vec3 &t)
	{
		element(0,c) = t[0];
		element(1,c) = t[1];
		element(2,c) = t[2];
	}

	//! r行成分取得
	void GetRow(int r, Vec3 &t) const
	{
		t[0] = element(r,0);
		t[1] = element(r,1);
		t[2] = element(r,2);
	}

	//! r行成分取得
	Vec3 GetRow(int r) const
	{
		Vec3 v;
		GetRow(r, v);
		return v;
	}

	//! c列成分取得
	void GetColumn(int c, Vec3 &t) const
	{
		t[0] = element(0,c);
		t[1] = element(1,c);
		t[2] = element(2,c);
	}

	//! c列成分取得
	Vec3 GetColumn(int c) const
	{
		Vec3 v;
		GetColumn(c, v);
		return v;
	}

	//! 逆行列取得
	Mat3x3 Inverse() const
	{
		real d = element(0, 0)*element(1, 1)*element(2, 2)- 
				 element(0, 0)*element(2, 1)*element(1, 2)+ 
				 element(1, 0)*element(2, 1)*element(0, 2)- 
				 element(1, 0)*element(0, 1)*element(2, 2)+ 
				 element(2, 0)*element(0, 1)*element(1, 2)- 
				 element(2, 0)*element(1, 1)*element(0, 2);

		if(d == 0) d = 1;

		return	Mat3x3( (element(1, 1)*element(2, 2)-element(1, 2)*element(2, 1))/d,
						  -(element(0, 1)*element(2, 2)-element(0, 2)*element(2, 1))/d,
						   (element(0, 1)*element(1, 2)-element(0, 2)*element(1, 1))/d,
						  -(element(1, 0)*element(2, 2)-element(1, 2)*element(2, 0))/d,
						   (element(0, 0)*element(2, 2)-element(0, 2)*element(2, 0))/d,
						  -(element(0, 0)*element(1, 2)-element(0, 2)*element(1, 0))/d,
						   (element(1, 0)*element(2, 1)-element(1, 1)*element(2, 0))/d,
						  -(element(0, 0)*element(2, 1)-element(0, 1)*element(2, 0))/d,
						   (element(0, 0)*element(1, 1)-element(0, 1)*element(1, 0))/d);	
	}

	//! 行列式
	double Determinant(void) const 
	{
		return element(0, 0)*element(1, 1)*element(2, 2)- 
			   element(0, 0)*element(2, 1)*element(1, 2)+ 
			   element(1, 0)*element(2, 1)*element(0, 2)- 
			   element(1, 0)*element(0, 1)*element(2, 2)+ 
			   element(2, 0)*element(0, 1)*element(1, 2)- 
			   element(2, 0)*element(1, 1)*element(0, 2);
	}


	//! 転置行列取得
	Mat3x3 Transpose() const
	{
		Mat3x3 mtrans;
		
		for(int i = 0; i < 3; ++i){
			for(int j = 0; j < 3; ++j){
				mtrans(i, j) = element(j, i);
			}
		}

		return mtrans;
	}
		
		
	Mat3x3 Scaled(const Vec3& s) const
	{
		return Mat3x3(element(0, 0)*s[0], element(0, 1)*s[1], element(0, 2)*s[2],
						 element(1, 0)*s[0], element(1, 1)*s[1], element(1, 2)*s[2],
						 element(2, 0)*s[0], element(2, 1)*s[1], element(2, 2)*s[2]);
	}

	//! 行列を右側から掛ける
	Mat3x3 &multRight(const Mat3x3 &b)
	{
		Mat3x3 mt(*this);
		SetValue(real(0));

		for(int i = 0; i < 3; ++i){
			for(int j = 0; j < 3; ++j){
				for(int c = 0; c < 3; ++c){
					element(i, j) += mt(i, c)*b(c, j);
				}
			}
		}

		return *this;
	}    

	//! 行列を左側から掛ける
	Mat3x3 &multLeft(const Mat3x3 &b)
	{
		Mat3x3 mt(*this);
		SetValue(real(0));

		for(int i = 0; i < 3; ++i){
			for(int j = 0; j < 3; ++j){
				for(int c = 0; c < 3; ++c){
					element(i, j) += b(i, c)*mt(c, j);
				}
			}
		}

		return *this;
	}

	//! dst = M*src
	void multMatrixVec(const Vec3 &src, Vec3 &dst) const
	{
		dst[0] = src[0]*element(0, 0)+src[1]*element(0, 1)+src[2]*element(0, 2);
		dst[1] = src[0]*element(1, 0)+src[1]*element(1, 1)+src[2]*element(1, 2);
		dst[2] = src[0]*element(2, 0)+src[1]*element(2, 1)+src[2]*element(2, 2);
	}

	void multMatrixVec(Vec3 &src_and_dst) const
	{
		multMatrixVec(Vec3(src_and_dst), src_and_dst);
	}


	//! dst = src*M
	void multVecMatrix(const Vec3 &src, Vec3 &dst) const
	{
		dst[0] = src[0]*element(0, 0)+src[1]*element(1, 0)+src[2]*element(2, 0);
		dst[1] = src[0]*element(0, 1)+src[1]*element(1, 1)+src[2]*element(2, 1);
		dst[2] = src[0]*element(0, 2)+src[1]*element(1, 2)+src[2]*element(2, 2);
	}
		

	void multVecMatrix(Vec3 & src_and_dst) const
	{
		multVecMatrix(Vec3(src_and_dst), src_and_dst);
	}


	//! 行列の要素を返す
	real& element(int row, int col){ return m[3*row+col]; }
	const real& element(int row, int col) const { return m[3*row+col]; }


	//! オペレータ
	real& operator()(int row, int col){ return element(row, col); }
	const real& operator()(int row, int col) const	{ return element(row, col); }

	Mat3x3& operator*=(const Mat3x3 &mat)
	{
		multRight(mat);
		return *this;
	}

	Mat3x3& operator*=(const real &r)
	{
		for(int i = 0; i < 3; ++i){
			element(0,i) *= r;
			element(1,i) *= r;
			element(2,i) *= r;
		}

		return *this;
	}

	Mat3x3& operator+=(const Mat3x3 &mat)
	{
		for(int i = 0; i < 3; ++i){
			element(0,i) += mat.element(0,i);
			element(1,i) += mat.element(1,i);
			element(2,i) += mat.element(2,i);
		}

		return *this;
	}

	Mat3x3& operator-=(const Mat3x3 &mat)
	{
		for(int i = 0; i < 3; ++i){
			element(0,i) -= mat.element(0,i);
			element(1,i) -= mat.element(1,i);
			element(2,i) -= mat.element(2,i);
		}

		return *this;
	}

	friend Mat3x3 operator*(const Mat3x3 &m1, const Mat3x3 &m2);
	friend Mat3x3 operator+(const Mat3x3 &m1, const Mat3x3 &m2);
	friend Mat3x3 operator-(const Mat3x3 &m1, const Mat3x3 &m2);
	friend Mat3x3 operator-(const Mat3x3 &m);
	friend Vec3 operator*(const Mat3x3 &m, const Vec3 &v);
	friend Vec3 operator*(const Vec3 &v, const Mat3x3 &m);
	friend Mat3x3 operator*(const Mat3x3 &m, const real &s);
	friend Mat3x3 operator*(const real &s, const Mat3x3 &m);
	friend bool operator==(const Mat3x3 &m1, const Mat3x3 &m2);
	friend bool operator!=(const Mat3x3 &m1, const Mat3x3 &m2);
};

inline Mat3x3 operator*(const Mat3x3 &m1, const Mat3x3 &m2)
{
	Mat3x3 product;
	
	product = m1;
	product.multRight(m2);
	
	return product;
}
inline Mat3x3 operator+(const Mat3x3 &m1, const Mat3x3 &m2)
{
	Mat3x3 res = m1;
	res += m2;
	return res;
}
inline Mat3x3 operator-(const Mat3x3 &m1, const Mat3x3 &m2)
{
	Mat3x3 res = m1;
	res -= m2;
	return res;
}
inline Mat3x3 operator-(const Mat3x3 &m)
{
	return Mat3x3(-m.m[0], -m.m[1], -m.m[2], -m.m[3], -m.m[4], -m.m[5], -m.m[6], -m.m[7], -m.m[8]);
}

inline Vec3 operator*(const Mat3x3 &m, const Vec3 &v)
{
	Vec3 product;
	
	m.multMatrixVec(v, product);
	
	return product;
}
inline Vec3 operator*(const Vec3 &v, const Mat3x3 &m)
{
	Vec3 product;
	
	m.multVecMatrix(v, product);
	
	return product;
}

inline Mat3x3 operator*(const Mat3x3 &m, const real &s)
{
	Mat3x3 ms = m;
	ms *= s;
	return ms;
}
inline Mat3x3 operator*(const real &s, const Mat3x3 &m)
{
	Mat3x3 ms = m;
	ms *= s;
	return ms;
}

inline bool operator==(const Mat3x3 &m1, const Mat3x3 &m2)
{
	return (m1(0,0) == m2(0,0) &&
			m1(0,1) == m2(0,1) &&
			m1(0,2) == m2(0,2) &&
			m1(1,0) == m2(1,0) &&
			m1(1,1) == m2(1,1) &&
			m1(1,2) == m2(1,2) &&
			m1(2,0) == m2(2,0) &&
			m1(2,1) == m2(2,1) &&
			m1(2,2) == m2(2,2));
}

inline bool operator!=(const Mat3x3 &m1, const Mat3x3 &m2)
{
	return !(m1 == m2);
}  

inline real* operator&(Mat3x3 &m)
{
	return m.GetValue();
}


//-----------------------------------------------------------------------------
// MARK:2x2行列クラス
//-----------------------------------------------------------------------------
class Mat2x2
{
protected:
	real m[4];

public:
	//! コンストラクタ
	Mat2x2(){ MakeIdentity(); }
	Mat2x2(real r){ SetValue(r); }
	Mat2x2(real *m){ SetValue(m); }
	Mat2x2(real a00, real a01, 
			  real a10, real a11)
	{
		element(0,0) = a00;
		element(0,1) = a01;
	
		element(1,0) = a10;
		element(1,1) = a11;
	}

	//! 行列をmpに入れる
	void GetValue(real *mp) const
	{
		int c = 0;
		for(int j = 0; j < 2; ++j){
			for(int i = 0; i < 2; ++i){
				mp[c++] = element(i,j);
			}
		}
	}

	//! 行列をmpに入れる
	void GetValue3x3(real *mp) const
	{
		mp[0] = element(0,0);
		mp[1] = element(1,0);
		mp[2] = 0.0;
					  
		mp[3] = element(0,1);
		mp[4] = element(1,1);
		mp[5] = 0.0;	  
						  
		mp[6] = 0.0;
		mp[7] = 0.0;
		mp[8] = 0.0;
	}

	//! 行列をポインタで取得
	real* GetValue(){ return m; }

	//! mpから行列に値を代入
	void SetValue(real *mp)
	{
		int c = 0;
		for(int j = 0; j < 2; ++j){
			for(int i = 0; i < 2; ++i){
				element(i,j) = mp[c++];
			}
		}
	}

	//! mpから行列に値を代入
	void SetValue3x3(real *mp)
	{
		element(0,0) = mp[0];
		element(1,0) = mp[1];
				 
		element(0,1)  =mp[3];
		element(1,1) = mp[4];
	}					  
						  
	//! 行列のすべての値をにする
	void SetValue(real r)  
	{					  
		for(int i = 0; i < 2; ++i){
			for(int j = 0;j < 2; ++j){
				element(i,j) = r;
			}			  
		}				  
	}
	void SetValue(real a00, real a01, 
				  real a10, real a11)
	{
		element(0,0) = a00;
		element(0,1) = a01;
	
		element(1,0) = a10;
		element(1,1) = a11;
	}

	//! 角から回転行列を設定
	void SetEuler(const double& angle) 
	{
		// yaw is CW around y-axis, pitch is CCW around x-axis, and roll is CW around z-axis
		SetValue(cos(angle), -sin(angle), 
				 sin(angle),  cos(angle));
					
	}

	//! 単位行列生成
	void MakeIdentity()
	{
		element(0,0) = 1.0;
		element(0,1) = 0.0;
		
		element(1,0) = 0.0;
		element(1,1) = 1.0; 
	}

	//! 単位行列を返す
	static Mat2x2 Identity()
	{
		static Mat2x2 mident(1.0, 0.0, 
								0.0, 1.0);

		return mident;
	}

	//! スケール(対角成分)を設定	    
	void SetScale(real s)
	{
		element(0,0) = s;
		element(1,1) = s;
	}

	//! スケール(対角成分)を設定	    
	void SetScale(const Vec2 &s)
	{
		element(0,0) = s[0];
		element(1,1) = s[1];
	}

	//! r行成分を設定
	void SetRow(int r, const Vec2 &t)
	{
		element(r,0) = t[0];
		element(r,1) = t[1];
	}

	//! c列成分を設定
	void SetColumn(int c, const Vec2 &t)
	{
		element(0,c) = t[0];
		element(1,c) = t[1];
	}

	//! r行成分取得
	void GetRow(int r, Vec2 &t) const
	{
		t[0] = element(r,0);
		t[1] = element(r,1);
	}

	//! r行成分取得
	Vec2 GetRow(int r) const
	{
		Vec2 v;
		GetRow(r, v);
		return v;
	}

	//! c列成分取得
	void GetColumn(int c, Vec2 &t) const
	{
		t[0] = element(0,c);
		t[1] = element(1,c);
	}

	//! c列成分取得
	Vec2 GetColumn(int c) const
	{
		Vec2 v;
		GetColumn(c, v);
		return v;
	}

	real Determinant(void) const
	{
		return element(0, 0)*element(1, 1)-element(1, 0)*element(0, 1);
	}

	//! 逆行列取得
	Mat2x2 Inverse() const
	{
		real d = element(0, 0)*element(1, 1)-element(1, 0)*element(0, 1);

		if(d == 0) d = 1;

		d = (real)1.0/d;

		return Mat2x2 (element(1, 1)*d, -element(0, 1)*d, 
						 -element(1, 0)*d,  element(0, 0)*d);
	}

	//! 転置行列取得
	Mat2x2 Transpose() const
	{
		Mat2x2 mtrans;
		
		for(int i = 0; i < 2; ++i){
			for(int j = 0; j < 2; ++j){
				mtrans(i, j) = element(j, i);
			}
		}

		return mtrans;
	}
		
	void GramSchmidt(void)
	{
		Vec2 c0, c1;
		GetColumn(0, c0); normalize(c0);
		c1[0] = -c0[1];
		c1[1] =  c0[0];
		SetColumn(0, c0);
		SetColumn(1, c1);
	}

	void Rot(real angle)
	{
		real cosAngle = cos(angle);
		real sinAngle = sin(angle);
		element(0, 0) = element(1, 1) = cosAngle;
		element(0, 1) = -sinAngle;
		element(1, 0) =  sinAngle;
	}

	Mat2x2 Scaled(const Vec2& s) const
	{
		return Mat2x2(element(0, 0)*s[0], element(0, 1)*s[1], 
						 element(1, 0)*s[0], element(1, 1)*s[1]);
	}

	//! 行列を右側から掛ける
	Mat2x2 &MultRight(const Mat2x2 &b)
	{
		Mat2x2 mt(*this);
		SetValue(real(0));

		for(int i = 0; i < 2; ++i){
			for(int j = 0; j < 2; ++j){
				for(int c = 0; c < 2; ++c){
					element(i, j) += mt(i, c)*b(c, j);
				}
			}
		}

		return *this;
	}    

	//! 行列を左側から掛ける
	Mat2x2 &MultLeft(const Mat2x2 &b)
	{
		Mat2x2 mt(*this);
		SetValue(real(0));

		for(int i = 0; i < 2; ++i){
			for(int j = 0; j < 2; ++j){
				for(int c = 0; c < 2; ++c){
					element(i, j) += b(i, c)*mt(c, j);
				}
			}
		}

		return *this;
	}

	//! dst = M*src
	void MultMatrixVec(const Vec2 &src, Vec2 &dst) const
	{
		dst[0] = src[0]*element(0, 0)+src[1]*element(0, 1);
		dst[1] = src[0]*element(1, 0)+src[1]*element(1, 1);
	}

	void MultMatrixVec(Vec2 &src_and_dst) const
	{
		MultMatrixVec(Vec2(src_and_dst), src_and_dst);
	}


	//! dst = src*M
	void MultVecMatrix(const Vec2 &src, Vec2 &dst) const
	{
		dst[0] = src[0]*element(0, 0)+src[1]*element(1, 0);
		dst[1] = src[0]*element(0, 1)+src[1]*element(1, 1);
	}
		

	void MultVecMatrix(Vec2 &src_and_dst) const
	{
		MultVecMatrix(Vec2(src_and_dst), src_and_dst);
	}



	//! オペレータ
	real& operator()(int row, int col){ return element(row, col); }
	const real& operator()(int row, int col) const	{ return element(row, col); }

	Mat2x2& operator*=(const Mat2x2 &mat)
	{
		MultRight(mat);
		return *this;
	}

	Mat2x2& operator*=(const real &r)
	{
		for(int i = 0; i < 2; ++i){
			element(0, i) *= r;
			element(1, i) *= r;
		}

		return *this;
	}

	Mat2x2& operator+=(const Mat2x2 &mat)
	{
		for(int i = 0; i < 2; ++i){
			element(0, i) += mat.element(0, i);
			element(1, i) += mat.element(1, i);
		}

		return *this;
	}

	Mat2x2& operator-=(const Mat2x2 &mat)
	{
		for(int i = 0; i < 2; ++i){
			element(0, i) -= mat.element(0, i);
			element(1, i) -= mat.element(1, i);
		}

		return *this;
	}

protected:
	//! 行列の要素を返す
	real& element(int row, int col){ return m[2*row+col]; }
	const real& element(int row, int col) const { return m[2*row+col]; }


public:
	friend Mat2x2 operator*(const Mat2x2 &m1, const Mat2x2 &m2);
	friend Mat2x2 operator+(const Mat2x2 &m1, const Mat2x2 &m2);
	friend Mat2x2 operator-(const Mat2x2 &m1, const Mat2x2 &m2);
	friend Mat2x2 operator-(const Mat2x2 &m);
	friend Vec2 operator*(const Mat2x2 &m, const Vec2 &v);
	friend Vec2 operator*(const Vec2 &v, const Mat2x2 &m);
	friend Mat2x2 operator*(const Mat2x2 &m, const real &s);
	friend Mat2x2 operator*(const real &s, const Mat2x2 &m);
	friend bool operator==(const Mat2x2 &m1, const Mat2x2 &m2);
	friend bool operator!=(const Mat2x2 &m1, const Mat2x2 &m2);
	friend real* operator&(const Mat2x2 &m);
};

inline Mat2x2 operator*(const Mat2x2 &m1, const Mat2x2 &m2)
{
	Mat2x2 product;
	
	product = m1;
	product.MultRight(m2);
	
	return product;
}

inline Mat2x2 operator+(const Mat2x2 &m1, const Mat2x2 &m2)
{
	Mat2x2 res = m1;
	res += m2;
	return res;
}

inline Mat2x2 operator-(const Mat2x2 &m1, const Mat2x2 &m2)
{
	Mat2x2 res = m1;
	res -= m2;
	return res;
}

inline Mat2x2 operator-(const Mat2x2 &m)
{
	return Mat2x2(-m.m[0], -m.m[1], -m.m[2], -m.m[3]);
}


inline Vec2 operator*(const Mat2x2 &m, const Vec2 &v)
{
	Vec2 product;
	
	m.MultMatrixVec(v, product);
	
	return product;
}

inline Vec2 operator*(const Vec2 &v, const Mat2x2 &m)
{
	Vec2 product;
	
	m.MultVecMatrix(v, product);
	
	return product;
}

inline Mat2x2 operator*(const Mat2x2 &m, const real &s)
{
	Mat2x2 ms = m;
	ms *= s;
	return ms;
}

inline Mat2x2 operator*(const real &s, const Mat2x2 &m)
{
	Mat2x2 ms = m;
	ms *= s;
	return ms;
}

inline bool operator==(const Mat2x2 &m1, const Mat2x2 &m2)
{
	return (m1(0,0) == m2(0,0) &&
			m1(0,1) == m2(0,1) &&
			m1(1,0) == m2(1,0) &&
			m1(1,1) == m2(1,1));
}

inline bool operator!=(const Mat2x2 &m1, const Mat2x2 &m2)
{
	return !(m1 == m2);
}  


inline real* operator&(Mat2x2 &m)
{
	return m.GetValue();
}




//-----------------------------------------------------------------------------
// MARK:NxN行列クラス
//-----------------------------------------------------------------------------
template <class T, int N>
class MatNxN
{
protected:
	real m_vElements[N*N];

public:
	//! コンストラクタ
	MatNxN(){ MakeIdentity(); }
	MatNxN(real r){ SetValue(r); }
	MatNxN(real *m){ SetValue(m); }


	//! 行列をmpに入れる
	void GetValue(real *mp) const
	{
		int c = 0;
		for(int j = 0; j < N; ++j){
			for(int i = 0; i < N; ++i){
				mp[c++] = element(i,j);
			}
		}
	}


	//! 行列をポインタで取得
	real* GetValue(){ return &m_vElements[0]; }

	//! mpから行列に値を代入
	void SetValue(real *mp)
	{
		int c = 0;
		for(int j = 0; j < N; ++j){
			for(int i = 0; i < N; ++i){
				element(i,j) = mp[c++];
			}
		}
	}
						  
	//! 行列のすべての値をrにする
	void SetValue(real r)  
	{					  
		for(int i = 0; i < N; ++i){
			for(int j = 0;j < N; ++j){
				element(i,j) = r;
			}			  
		}				  
	}

	//! 単位行列生成
	void MakeIdentity()
	{
		for(int i = 0; i < N; ++i){
			for(int j = 0; j < N; ++j){
				element(i,j) = (i == j) ? 1.0 : 0.0;
			}			  
		}				  
	}

	//! r行成分を設定
	void SetRow(int r, const double *t)
	{
		for(int j = 0; j < N; ++j) element(r,j) = t[j];
	}
	//! c列成分を設定
	void SetColumn(int c, const Vec3 &t)
	{
		for(int i = 0; i < N; ++i) element(i,c) = t[i];
	}

	//! r行成分取得
	void GetRow(int r, double *t) const
	{
		for(int j = 0; j < N; ++j) t[j] = element(r,j);
	}
	//! c列成分取得
	void GetColumn(int c, double *t) const
	{
		for(int i = 0; i < N; ++i) t[i] = element(i,c);
	}

	//! 転置行列取得
	MatNxN Transpose() const
	{
		MatNxN mtrans(N);
		for(int i = 0; i < N; ++i){
			for(int j = 0; j < N; ++j){
				mtrans(i,j) = element(j,i);
			}
		}
		return mtrans;
	}
		
	//! 行列を右側から掛ける
	MatNxN& MultRight(const MatNxN &b)
	{
		MatNxN mt(*this);
		SetValue(real(0));
		for(int i = 0; i < N; ++i){
			for(int j = 0; j < N; ++j){
				for(int c = 0; c < N; ++c){
					element(i,j) += mt(i,c)*b(c,j);
				}
			}
		}
		return *this;
	}    

	//! 行列を左側から掛ける
	MatNxN& MultLeft(const MatNxN &b)
	{
		MatNxN mt(*this);
		SetValue(real(0));
		for(int i = 0; i < N; ++i){
			for(int j = 0; j < N; ++j){
				for(int c = 0; c < N; ++c){
					element(i,j) += b(i,c)*mt(c,j);
				}
			}
		}
		return *this;
	}

	//! dst = M*src
	void MultMatrixVec(const real *src, real *dst) const
	{
		for(int i = 0; i < N; ++i){
			dst[i] = 0.0;
			for(int j = 0; j < N; ++j){
				dst[i] += element(i,j)*src[j];
			}
		}
	}
	//! dst = src*M
	void MultVecMatrix(const real *src, real *dst) const
	{
		for(int j = 0; j < N; ++j){
			dst[j] = 0.0;
			for(int i = 0; i < N; ++i){
				dst[j] += src[i]*element(i,j);
			}
		}
	}


	//! オペレータ
	real& operator()(int row, int col){ return element(row, col); }
	const real& operator()(int row, int col) const	{ return element(row, col); }

	MatNxN& operator*=(const MatNxN &mat)
	{
		MultRight(mat);
		return *this;
	}

	MatNxN& operator*=(const real &r)
	{
		for(int i = 0; i < N; ++i){
			for(int j = 0; j < N; ++j){
				element(i,j) *= r;
			}
		}
		return *this;
	}

	MatNxN& operator+=(const MatNxN &mat)
	{
		for(int i = 0; i < N; ++i){
			for(int j = 0; j < N; ++j){
				element(i,j) += mat(i,j);
			}
		}
		return *this;
	}

	MatNxN& operator-=(const MatNxN &mat)
	{
		for(int i = 0; i < N; ++i){
			for(int j = 0; j < N; ++j){
				element(i,j) -= mat(i,j);
			}
		}
		return *this;
	}

	bool IsSymmetric(void)
	{
		for(int i = 0; i < N-1; ++i){
			for(int j = i+1; j < N; ++j){
				if(!RX_FEQ(element(i,j), element(j,i))){
					return false;
				}
			}
		}
		return true;
	}

	// LU分解による逆行列算出(対称行列の場合のみ有効)
	bool LUDecomp(MatNxN<T,N> &a, std::vector<int> &indx, double &d);
	bool LUbksb(const MatNxN<T,N> &a, const std::vector<int> &indx, std::vector<double> &b);
	void Invert(void);

protected:
	//! 行列の要素を返す
	real& element(int row, int col){ return m_vElements[N*row+col]; }
	const real& element(int row, int col) const { return m_vElements[N*row+col]; }
};



/*!
 * LU分解:行列a(n×n)をLU分解して返す
 * @param[inout] a n×n正値対称行列
 */
template <class T, int N> 
bool MatNxN<T,N>::LUDecomp(MatNxN<T,N> &a, std::vector<int> &indx, double &d)
{
	int i, imax = 0, j, k;
	float big, dum, sum, temp;
	std::vector<double> vv;

	vv.resize(N);
	d = 1.0;
		
	for(i = 0; i < N; i++){
		big = 0.0;
		for(j = 0; j < N; j++)
			if((temp = fabs(a(i,j))) > big) big = temp;

		if(big  ==  0.0) return false;
			
		vv[i] = 1.0f/big;
	}

	for(j = 0; j < N; j++){
		for(i = 0; i < j; i++){
			sum = a(i,j);
			for(k = 0; k < i; k++) sum -= a(i,k)*a(k,j);
			a(i,j) = sum;
		}

		big = 0.0;
		for(i = j; i < N; i++){
			sum = a(i,j);
			for(k = 0; k < j; k++) sum -= a(i,k)*a(k,j);
			a(i,j) = sum;
			if((dum = vv[i]*fabs(sum)) >= big){
				big = dum;
				imax = i;
			}
		}

		if(j != imax){
			for(k = 0; k < N; k++){
				dum = a(imax,k);
				a(imax,k) = a(j,k);
				a(j,k) = dum;
			}
			d = -d;
			vv[imax] = vv[j];
		}

		indx[j] = imax;
		if(a(j,j) == 0.0) a(j,j) = (double)RX_FEQ_EPS;
		if(j != N-1){
			dum = 1.0f/(a(j,j));
			for(i = j+1; i < N; i++) a(i,j) *= dum;
		}
	}

	return true;
}

/*!
 * LU分解した行列a(n×n)から前進代入・後退代入によりA・x=bを解く
 * @param[in] a LU分解された行列
 * @param[in] indx LUDecompルーチンが吐き出したピボット選択による行交換の履歴
 * @param[inout] b   右辺ベクトル
 * @param[in] n 
 * @return 
 */
template <class T, int N> 
bool MatNxN<T,N>::LUbksb(const MatNxN<T,N> &a, const std::vector<int> &indx, std::vector<double> &b)
{
	int i, ii=0, ip, j;
	double sum;

	// 前進代入
	for(i = 0; i < N; ++i){
		ip = indx[i];
		sum = b[ip];
		b[ip] = b[i];
		if(ii != 0){
			for(j = ii-1; j < i; ++j){
				sum -= a(i,j)*b[j];
			}
		}
		else if(sum != 0.0){
			ii = i+1;
		}

		b[i] = sum;
	}

	// 後退代入
	for(i = N-1; i >= 0; --i){
		sum = b[i];
		for(j = i+1; j < N; ++j){
			sum -= a(i,j)*b[j];
		}

		b[i] = sum/a(i,i);
	}

	return true;
}

/*!
 * LU分解で逆行列を計算
 * @param[in]  mat 計算したい行列
 * @param[out] inv 逆行列
 * @return 行列の大きさが0のときにfalseを返す
 */
template <class T, int N> 
void MatNxN<T,N>::Invert(void)
{
	if(!IsSymmetric()) return;

	double d;
	std::vector<int> indx;
	indx.resize(N);

	MatNxN<T,N> mat(*this);

	// 行列を1回だけLU分解
	LUDecomp(mat, indx, d);

	std::vector<double> col;
	col.resize(N);

	// 行ごとに逆行列を算出
	for(int j = 0; j < N; ++j){
		for(int i = 0; i < N; ++i){
			col[i] = 0.0;
		}
		col[j] = 1.0;

		LUbksb(mat, indx, col);
		for(int i = 0; i < N; ++i){
			element(i,j) = col[i];
		}
	}
}


//! 行列同士のかけ算
template <class T, int N> 
inline MatNxN<T,N> operator*(const MatNxN<T,N> &m1, const MatNxN<T,N> &m2)
{
	MatNxN<T,N> product;
	product = m1;
	product.MultRight(m2);
	return product;
}
//! 行列同士の足し算
template <class T, int N> 
inline MatNxN<T,N> operator+(const MatNxN<T,N> &m1, const MatNxN<T,N> &m2)
{
	MatNxN<T,N> res = m1;
	res += m2;
	return res;
}
//! 行列同士の引き算
template <class T, int N> 
inline MatNxN<T,N> operator-(const MatNxN<T,N> &m1, const MatNxN<T,N> &m2)
{
	MatNxN<T,N> res = m1;
	res -= m2;
	return res;
}
//! 行列の符号反転
template <class T, int N> 
inline MatNxN<T,N> operator-(const MatNxN<T,N> &m)
{
	MatNxN<T,N> res(0.0);
	res -= m;
	return res;
}

//! 行列とスカラー値のかけ算
template <class T, int N> 
inline MatNxN<T,N> operator*(const MatNxN<T,N> &m, const real &s)
{
	MatNxN<T,N> ms = m;
	ms *= s;
	return ms;
}
template <class T, int N> 
inline MatNxN<T,N> operator*(const real &s, const MatNxN<T,N> &m)
{
	MatNxN<T,N> ms = m;
	ms *= s;
	return ms;
}

//! 行列同士の比較
template <class T, int N> 
inline bool operator==(const MatNxN<T,N> &m1, const MatNxN<T,N> &m2)
{
	for(int i = 0; i < N; ++i){
		for(int j = 0; j < N; ++j){
			if(m1(i,j) != m2(i,j)){
				return false;
			}
		}
	}
	return true;
}
template <class T, int N> 
inline bool operator!=(const MatNxN<T,N> &m1, const MatNxN<T,N> &m2)
{
	return !(m1 == m2);
}  

//! 要素参照
template <class T, int N> 
inline real* operator&(MatNxN<T,N> &m)
{
	return m.GetValue();
}

//! 画面出力オペレータ
template<class T, int N> 
inline std::ostream &operator<<(std::ostream &out, const MatNxN<T,N> &a)
{
	for(int i = 0; i < N; ++i){
		out << "| ";
		for(int j = 0; j < N; ++j){
			out << a(i,j) << " ";
		}
		out << "|" << std::endl;
	}
	return out;
}

inline std::ostream &operator<<(std::ostream &out, const Mat4x4 &a)
{
	for(int i = 0; i < 4; ++i){
		out << "| ";
		for(int j = 0; j < 4; ++j){
			out << a(i,j) << " ";
		}
		out << "|" << std::endl;
	}
	return out;
}

inline std::ostream &operator<<(std::ostream &out, const Mat3x3 &a)
{
	for(int i = 0; i < 3; ++i){
		out << "| ";
		for(int j = 0; j < 3; ++j){
			out << a(i,j) << " ";
		}
		out << "|" << std::endl;
	}
	return out;
}

inline std::ostream &operator<<(std::ostream &out, const Mat2x2 &a)
{
	for(int i = 0; i < 2; ++i){
		out << "| ";
		for(int j = 0; j < 2; ++j){
			out << a(i,j) << " ";
		}
		out << "|" << std::endl;
	}
	return out;
}


//inline std::istream &operator>>(std::istream &in, Vec3& a)
//{
//	return in >> a[0] >> a[1] >> a[2] ;
//}



#endif // _MATRIX_H_