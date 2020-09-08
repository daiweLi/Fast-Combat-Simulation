// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           radar.h
*   @brief          雷达及全向雷达告警器功能仿真。
*   @details        雷达及全向雷达告警器功能仿真。
*   @author         LiDaiwei
*   @date           20191102
*   @version        1.0.0.2
*   @par Copyright
*                   LiDaiwei
*   @par History
*                   1.0.0.1: LiDaiwei, 20191102, 首次创建
*                   1.0.0.2: LiDaiwei, 20191113, 删除接收机噪声环节，免去随机干扰

*/



#ifndef RADAR_H_INCLUDED
#define RADAR_H_INCLUDED


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          初始化雷达功能函数。
*   @details        初始化雷达功能函数。
*   @param[out]      Lambda               雷达工作波长
*   @param[out]      Pgt                  雷达发射功率
*   @param[out]      nG                   天线效率
*   @param[out]      Gmax                 天线最大增益
*   @param[out]      theta_t              主瓣波束宽度 //默认值1.0
*   @param[out]      theta_b              单程半功率点波束宽度 //默认值3.14
*   @param[out]      Lr                   雷达综合损耗Lr
*   @param[out]      Br                   雷达工作的瞬时带宽 //默认值5兆
*   @param[out]      Nf                   雷达接收机噪声系数
*   @param[out]      azimuth_radar_width  雷达扫描方位角范围
*   @param[out]      pitch_radar_width    雷达扫描俯仰角范围
*   @retval          0               正常
*   @retval          1               错误
*/
int Radar_Initial(
	double* lambda,
	double* Pgt,
	double* nG,
	double* Gmax,
	double* theta_t,
	double* theta_b,
	double* Lr,
//	double* Br,
//	double* Nf,
	double* azimuth_radar_width,
	double* pitch_radar_width);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          初始化全向警告器参数。
*   @details        初始化全向警告器参数。
*   @param[out]      P                    接收机灵敏度
*   @param[out]      G                    接收机天线增益
*   @param[out]      Ltx                  接收机传输损耗
*   @param[out]      Lp                   极化损失
*   @param[out]      f_min                告警器接收机频率最小值
*   @param[out]      f_max                告警器接收机频率最大值
*   @param[out]      azimuth_alarm_width  接收机方位角范围
*   @param[out]      pitch_alarm_width    接收机俯仰角范围
*   @retval         0               正常
*   @retval         1               错误
*/
int Alarm_Initial(
	double* P,
	double* G,
	double* Ltx,
	double* Lp,
	double* f_min,
	double* f_max,
	double* azimuth_alarm_width,
	double* pitch_alarm_width);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          计算两个飞机之间的距离。
*   @details        计算两个飞机之间的距离。
*   @param[out]     distance               两飞机的距离
*   @param[in]      positionA[3]           A飞机的三维位置坐标(x,y,z)
*   @param[in]      positionB[3]           B飞机的三维位置坐标(x,y,z)
*   @retval         0               正常
*   @retval         1               错误
*/
int getDistance (
	double* distance,
	const double positionA[3],
	const double positionB[3]);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          计算B机相对A机的探测方位角及俯仰角。
*   @details        计算B机相对A机的探测方位角及俯仰角。
*   @param[out]     azimuth                B机相对A机的探测方位角
*   @param[out]     Pitch                  B机相对A机的探测俯仰角
*   @param[in]      positionA[3]           A飞机的三维位置坐标(x,y,z)
*   @param[in]      attitudeA[3]           A飞机的三个欧拉角姿态(x,y,z-滚转，偏航，俯仰)
*   @param[in]      positionB[3]           B飞机的三维位置坐标(x,y,z)
*   @retval         0               正常
*   @retval         1               错误
*/
int getTargetAzimuthPitch(
	double* azimuth,
	double* pitch,
	const double positionA[3],
	const double attitudeA[3],///*弧度*/
	const double positionB[3]);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          计算两个飞机之间的雷达视距。
*   @details        计算两个飞机之间的雷达视距。
*   @param[out]     sight           两个飞机之间的雷达视距
*   @param[in]      h1              A飞机的高度
*   @param[in]      h2              B飞机的高度
*   @retval         0               正常
*   @retval         1               错误
*/
int getRadarSight (
	double* sight,
	const double h1,
	const double h2);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          计算目标RCS。
*   @details        计算目标RCS。
*   @param[out]     RCS                             雷达方向向目标探测时的RCS
*   @param[in]      azimuth_RadarToTarget           雷达相对目标的探测方位角
*   @param[in]      pitch_RadarToTarget             雷达相对目标的探测俯仰角
*   @param[in]      SigimaType[5]                   目标的5个RCS典型值(+x,-x,+y,-z,z)
*   @retval         0               正常
*   @retval         1               错误
*/
int getTargetRCS (
	double* RCS,
	const double azimuth_RadarToTarget,
	const double pitch_RadarToTarget,
	const double SigimaType[5]);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          计算雷达发射天线在目标方位的增益。
*   @details        计算雷达发射天线在目标方位的增益。
*   @param[out]     Gain                            天线增益系数
*   @param[in]      azimuth_TargetToRadar           目标相对雷达的探测方位角
*   @param[in]      pitch_TargetToRadar             目标相对雷达的探测俯仰角
*   @param[in]      nG                              天线效率
*   @param[in]      theta_t                         主瓣波束宽度 //默认值1.0
*   @param[in]      theta_b                         单程半功率点波束宽度 //默认值3.14
*   @param[in]      Gmax                            天线最大增益
*   @param[in]      azimuth_radar_width             雷达扫描方位角范围
*   @param[in]      pitch_radar_width               雷达扫描俯仰角范围
*   @retval         0               正常
*   @retval         1               错误
*/
int getTargetAntennaGain(
	double* Gain,
	const double azimuth_TargetToRadar,
	const double pitch_TargetToRadar,
	const double nG,
	const double theta_t,
	const double theta_b,
	const double Gmax,
	const double azimuth_radar_width,
	const double pitch_radar_width);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          计算目标回波功率。
*   @details        计算目标回波功率。
*   @param[out]     EchoPower       目标回波功率
*   @param[in]      Pgt             雷达发射功率
*   @param[in]      Ggt             发射天线增益
*   @param[in]      Sigima          目标RCS（雷达反射截面积）
*   @param[in]      Lambda          雷达工作波长
*   @param[in]      R               雷达与目标距离
*   @param[in]      Lr              雷达综合损耗Lr
*   @retval         0               正常
*   @retval         1               错误
*/
int getTargetEchoPower(
	double* EchoPower,
	const double Pgt,
	const double Ggt,
	const double Sigima,
	const double Lambda,
	const double R,
	const double Lr);

//// --------------------------------------------------------------------------------------------------------------------------------
///**
//*   @brief          计算接收机噪声。
//*   @details        计算接收机噪声。
//*   @param[out]     noise           噪声瞬时功率
//*   @param[in]      Br              雷达工作的瞬时带宽 //默认值5兆
//*   @param[in]      Nf              雷达接收机噪声系数
//*   @retval         0               正常
//*   @retval         1               错误
//*/
//int getReceivingNoise(
//	double* noise,
//	const double Br,
//	const double Nf);
//
//
//// --------------------------------------------------------------------------------------------------------------------------------
///**
//*   @brief          计算雷达接收机综合信噪比。
//*   @details        计算雷达接收机综合信噪比。
//*   @param[out]     SNt             雷达接收机综合信噪比
//*   @param[in]      Pgr             目标回波功率
//*   @param[in]      Pin             噪声功率
//*   @param[in]      Pcs             杂波回波功率
//*   @param[in]      Pj              雷达受到的有源干扰功率
//*   @retval         0               正常
//*   @retval         1               错误
//*/
//int getSignalNoiseRatio(
//	double* SNt,
//	const double Pgr,
//	const double Pin,
//	const double Pcs,
//	const double Pj);
//
//// --------------------------------------------------------------------------------------------------------------------------------
///**
//*   @brief          计算探测概率。
//*   @details        计算探测概率（取虚警概率为10^-6）。
//*   @param[out]     Pd_out              探测到目标的概率
//*   @param[in]      SNt             雷达接收机综合信噪比
//*   @retval         0               正常
//*   @retval         1               错误
//*/
//int getPd(
//	double* Pd_out,
//	const double SNt);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          判断并确认是否发现目标。
*   @details        判断并确认是否发现目标。
*   @param[out]     finded          是or否发现目标
*   @param[in]      Pgr             目标回波功率
*   @param[in]      n               前3次扫描中发现目标的次数
*   @retval         0               正常
*   @retval         1               错误
*/
int confirmTarget(
	bool* finded,
	const double Pgr);


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          计算雷达告警器接收机探测到某辐射源的最大距离。
*   @details        计算雷达告警器接收机探测到某辐射源的最大距离。
*   @param[out]     Rmax            雷达告警器接收机能探测到某辐射源的最大距离
*   @param[in]      P               接收机灵敏度
*   @param[in]      Pt              辐射源发射功率
*   @param[in]      Lambda          波长
*   @param[in]      Gt              辐射源发射机天线增益
*   @param[in]      G               接收机天线增益
*   @param[in]      Ltx             接收机传输损耗
*   @param[in]      Lp              极化损失
*   @retval         0               正常
*   @retval         1               错误
*/
int getAlarmMaxDistance(
	double* Rmax,
	const double P,
	const double Pt,
	const double Lambda,
	const double Gt,
	const double G,
	const double Ltx,
	const double Lp);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          判断雷达全向告警器能否探测到威胁雷达。
*   @details        判断雷达全向告警器能否探测到威胁雷达。
*   @param[out]     alarm             雷达告警器是or否探测到雷达
*   @param[in]      f                 雷达辐射源频率
*   @param[in]      f_min             告警器接收机频率最小值
*   @param[in]      f_max             告警器接收机频率最大值
*   @param[in]      pitch_alarm       辐射源相对接收机方位角
*   @param[in]      pitch_alarm_min   收机方位角范围最小值
*   @param[in]      pitch_alarm_max   收机方位角范围最大值
*   @param[in]      pitch_alarm       辐射源相对接收机俯仰角
*   @param[in]      pitch_alarm_min   接收机俯仰角范围最小值
*   @param[in]      pitch_alarm_max   接收机俯仰角范围最大值
*   @param[in]      R                 接收机和辐射源距离
*   @param[in]      Rmax              接收机最大探测距离
*   @param[in]      azimuth_radar     接收机相对辐射源方位角
*   @param[in]      azimuth_radar_min 辐射源扫描方位角范围最小值
*   @param[in]      azimuth_radar_max 辐射源扫描方位角范围最大值
*   @param[in]      pitch_radar       接收机相对辐射源俯仰角
*   @param[in]      pitch_radar_min   辐射源扫描俯仰角范围最小值
*   @param[in]      pitch_radar_max   辐射源扫描俯仰角范围最大值
*   @retval         0               正常
*   @retval         1               错误
*/
int confirmAlarm(
	bool* alarm,
	const double f,
	const double f_min,
	const double f_max,
	const double azimuth_alarm,
	const double azimuth_alarm_min,
	const double azimuth_alarm_max,
	const double pitch_alarm,
	const double pitch_alarm_min,
	const double pitch_alarm_max,
	const double R,
	const double Rmax,
	const double azimuth_radar,
	const double azimuth_radar_min,
	const double azimuth_radar_max,
	const double pitch_radar,
	const double pitch_radar_min,
	const double pitch_radar_max);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          雷达功能综合函数。
*   @details        雷达功能综合函数。
*   @param[out]     findedTarget         是or否发现目标
*   @param[out]     Pgr                  目标回波功率
*   @param[in]      positionRadar[3]     雷达机的三维位置坐标(x,y,z)
*   @param[in]      attitudeRadar[3]     雷达机的三个欧拉角姿态(x,y,z-滚转，偏航，俯仰)
*   @param[in]      positionTarget[3]    目标机的三维位置坐标(x,y,z)
*   @param[in]      attitudeTarget[3]    目标机的三个欧拉角姿态(x,y,z-滚转，偏航，俯仰)
*   @param[in]      SigimaType[5]        目标的5个RCS典型值(+x,-x,+y,-z,z)
*   @param[in]      Lambda               雷达工作波长
*   @param[in]      Pgt                  雷达发射功率
*   @param[in]      nG                   天线效率
*   @param[in]      Gmax                 天线最大增益
*   @param[in]      theta_t              主瓣波束宽度
*   @param[in]      theta_b              单程半功率点波束宽度
*   @param[in]      Lr                   雷达综合损耗Lr
*   @param[in]      Br                   雷达工作的瞬时带宽 //默认值5兆
*   @param[in]      Nf                   雷达接收机噪声系数
*   @param[in]      azimuth_radar_width  雷达扫描方位角范围
*   @param[in]      pitch_radar_width    雷达扫描俯仰角范围
*   @retval         0               正常
*   @retval         1               错误
*/
int Radar(
	bool* findedTarget,
	double* Pgr,
	const double positionRadar[3],
	const double attitudeRadar[3],
	const double positionTarget[3],
	const double attitudeTarget[3],
	const double SigimaType[5]/*目标典型RCS*/,
	const double Lambda/*雷达波长*/,
	const double Pgt/*雷达发射功率*/,
	const double nG/*天线效率*/,
	const double Gmax/*天线最大增益*/,
	const double theta_t,
	const double theta_b,
	const double Lr/*雷达综合损耗*/,
//	const double Br/*雷达工作时的瞬时带宽*/,
//	const double Nf/*雷达接收机噪声系数*/,
	const double azimuth_radar_width,
	const double pitch_radar_width);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          全向告警器综合功能函数。
*   @details        全向告警器综合功能函数。
*   @param[out]     alarm_on             雷达告警器是or否探测到雷达
*   @param[in]      positionRadar[3]     雷达机的三维位置坐标(x,y,z)
*   @param[in]      attitudeRadar[3]     雷达机的三个欧拉角姿态(x,y,z-滚转，偏航，俯仰)
*   @param[in]      positionTarget[3]    目标机的三维位置坐标(x,y,z)
*   @param[in]      attitudeTarget[3]    目标机的三个欧拉角姿态(x,y,z-滚转，偏航，俯仰)
*   @param[in]      Lambda               雷达工作波长
*   @param[in]      Pgt                  雷达发射功率
*   @param[in]      nG                   天线效率
*   @param[in]      Gmax                 天线最大增益
*   @param[in]      theta_t              主瓣波束宽度
*   @param[in]      theta_b              单程半功率点波束宽度
*   @param[in]      P                    接收机灵敏度
*   @param[in]      G                    接收机天线增益
*   @param[in]      Ltx                  接收机传输损耗
*   @param[in]      Lp                   极化损失
*   @param[in]      f_min                告警器接收机频率最小值
*   @param[in]      f_max                告警器接收机频率最大值
*   @param[in]      azimuth_alarm_width  接收机方位角范围
*   @param[in]      pitch_alarm_width    接收机俯仰角范围
*   @param[in]      Rmax                 接收机最大探测距离
*   @param[in]      azimuth_radar_width  辐射源扫描方位角范围
*   @param[in]      pitch_radar_width    辐射源扫描俯仰角范围
*   @retval         0               正常
*   @retval         1               错误
*/
int Alarm(
	bool* alarm_on,
	const double positionRadar[3],
	const double attitudeRadar[3],
	const double positionTarget[3],
	const double attitudeTarget[3],
	const double Lambda/*雷达波长*/,
	const double Pgt/*雷达发射功率*/,
	const double nG/*发射天线效率*/,
	const double Gmax/*发射天线最大增益*/,
	const double theta_t,
	const double theta_b,
	const double P/*接收机灵敏度*/,
	const double G/*接收机天线增益*/,
	const double Ltx/*接收机传输损耗*/,
	const double Lp/*极化损失*/,
	const double f_min,
	const double f_max,
	const double azimuth_alarm_width,
	const double pitch_alarm_width,
	const double azimuth_radar_width,
	const double pitch_radar_width);

#endif // RADAR_H_INCLUDED
