/*
 * lut.h
 *
 *  Created on: 30 Oct 2017
 *      Author: nusuav
 */

#ifndef MODULES_MC_ATT_CONTROL_NL_LUT_H_
#define MODULES_MC_ATT_CONTROL_NL_LUT_H_

struct FM_PARAM {
	unsigned int n;	///< rotor angular speed
	float Va;		///< wind velocity
	float thetab;	///< angle
};

class FMLut {
public:
	FMLut();
	~FMLut();

public:
	static const float lut_f_m_rpm_va_thetab[5][17][10][6];		///< FM clock-wise lut
	static const float lut_f_m_c_rpm_va_thetab[5][17][10][6];	///< FM counter clock-wise lut
	static const float lut_pfmpn_rpm_va_thetab[5][17][10][6];	///< FM partial differential lut for clock-wise
	static const float lut_pfmpn_c_rpm_va_thetab[5][17][10][6];	///< FM partial differential lut for counter clock-wise
	static const float lut_rpmi[17][9][5];						///< FM clock wise rpm lut
	static const float lut_rpmic[17][9][5];						///< FM counter clock-wise rpm lut
	static const float lut_prpmipi[17][9][5];					///< FM partial differential lut for clock-wise
	static const float lut_prpmicpi[17][9][5];					///< FM partial differential lut for counter clock-wise
	static const float lut_rpm[10];		///< Lut for rotor rpm
	static const float lut_Va[5];		///< Lut for wind velocity
	static const float lut_thetab[17];	///< Lut for thetab
	static const float lut_di[9];		///< Lut for control input: 0.1 ~ 0.9

private:
	FM_PARAM	m_fmParam;

public:
	float getByIndex(const int& i, const int& j, const int& k, const int& m);
	math::Vector<6> interpolation3(const FM_PARAM& param, const float lut_f_m[5][17][10][6], const float lut_rpm[10],
			const float lut_Va[5], const float lut_thetab[17]);
	float interpolation3(const math::Vector<3> param, const float lut_rpmi[17][9][5], const float lut_di[9],
			const float lut_Va[5], const float lut_thetab[17]);

};




#endif /* MODULES_MC_ATT_CONTROL_NL_LUT_H_ */
