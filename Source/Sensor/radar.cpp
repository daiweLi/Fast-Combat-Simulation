// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           radar.cpp
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

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @name           头文件。
*   @{
*/
#include "radar.h"
#include "tool_function.h"
//#include <stdlib.h>
//#include <time.h>

/** @}  */



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
	double* pitch_radar_width)
{
//	srand((unsigned)time(NULL));//设定随机数种子
	//设置各个参数：
	*lambda=0.03;
	*Pgt=20000;
	*nG=0.8;
	*Gmax=25000;
	*theta_t=1.0;
	*theta_b=3.14;
	*Lr=0.9;
//	*Br=5E6;
//	*Nf=0.001;
	*azimuth_radar_width=2.5;
	*pitch_radar_width=2.5;

	return 0;
}


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
	double* pitch_alarm_width)
{
//	srand((unsigned)time(NULL));//设定随机数种子
	//设置各个参数：
	*P=0.000003;
	*G=20000;
	*Ltx=20.0;
	*Lp=10.0;
	*f_min=2000000000;
	*f_max=18000000000;
	*azimuth_alarm_width=2*M_PI;
	*pitch_alarm_width=2*M_PI;
	return 0;
}


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
	const double positionB[3])
{
	double dx2,dy2,dz2;//x,y,z的差的平方

	dx2=pow(positionA[0]-positionB[0],2);
	dy2=pow(positionA[1]-positionB[1],2);
	dz2=pow(positionA[2]-positionB[2],2);

	*distance = sqrt(dx2+dy2+dz2);

	return 0;
}

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
	const double positionB[3])
{
	//在导航坐标系（n系）内A到B的方向向量
	double Vban[3]={positionB[0]-positionA[0],positionB[1]-positionA[1],positionB[2]-positionA[2]};

	//方向余弦矩阵的算子
	double c1=cos(attitudeA[1]),s1=sin(attitudeA[1]);
	double c2=cos(attitudeA[2]),s2=sin(attitudeA[2]);
	double c3=cos(attitudeA[0]),s3=sin(attitudeA[0]);
	//方向余弦矩阵
	double Cnb[3][3];
	Cnb[0][0]=c1*c2;	Cnb[0][1]=s1*s3-c1*c3*s2;	Cnb[0][2]=c3*s1+c1*s2*s3;
	Cnb[1][0]=s2;		Cnb[1][1]=c2*c3;		Cnb[1][2]=-c2*s3;
	Cnb[2][0]=-c2*s1;	Cnb[2][1]=c1*s3+c3*s1*s2;	Cnb[2][2]=c1*c3-s1*s2*s3;

	//在A机载体坐标系（b系）内A到B的方向向量
	double Vbab[3];
	Vbab[0]=Cnb[0][0]*Vban[0]+Cnb[0][1]*Vban[1]+Cnb[0][2]*Vban[2];
	Vbab[1]=Cnb[1][0]*Vban[0]+Cnb[1][1]*Vban[1]+Cnb[1][2]*Vban[2];
	Vbab[2]=Cnb[2][0]*Vban[0]+Cnb[2][1]*Vban[1]+Cnb[2][2]*Vban[2];

	double Azimuth=acos(Vbab[0]/sqrt(pow(Vbab[0],2)+pow(Vbab[2],2)));
	double temp;
	deg2rad(&temp,360.0);
	*azimuth=(Vbab[2]<0)?Azimuth:(temp-Azimuth);
	//*pitch=atan(Vbab[1]/sqrt(pow(Vbab[0],2)+pow(Vbab[1],2)+pow(Vbab[2],2)));
	*pitch=atan(Vbab[1]/sqrt(pow(Vbab[0],2)+pow(Vbab[2],2)));
	return 0;
}


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
	const double h2)
{
	*sight = 4120*(sqrt(h1)+sqrt(h2));
	return 0;
}

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
	const double SigimaType[5])
{
	//从目标正前方、正后方、正上方、正下方、正侧方探测时的典型RCS
	double Sxp=SigimaType[0],Sxn=SigimaType[1],Syp=SigimaType[2],Syn=SigimaType[3],Sz=SigimaType[4];
	//雷达相对目标的探测方位、俯仰角
	double azimuth=azimuth_RadarToTarget,pitch=pitch_RadarToTarget;

	double S1,S2,S3;
	double temp1,temp2;
	deg2rad(&temp1,90);
	deg2rad(&temp2,270);
	S1=(azimuth>temp1 && azimuth<temp2)?Sxn:Sxp;
	S2=(pitch<0)?Syn:Syp;
	S3=Sz;

	*RCS = S1*(1-sqrt(abs(sin(pitch))))*(1-sqrt(abs(sin(azimuth))))+S2*abs(sin(pitch))+
		S3*(1-sqrt(abs(sin(pitch))))*abs(sin(azimuth));

	return 0;
}

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          计算水平面某一个角度的天线增益。
*   @details        计算水平面某一个角度的天线增益。
*   @param[out]     f_surface                    平面某一个角度的天线增益系数
*   @param[in]      theta                        目标相对雷达的探测方位角
*   @param[in]      theta_t                      主瓣波束宽度 //默认值1.0
*   @param[in]      theta_b                      单程半功率点波束宽度 //默认值3.14
*   @retval         0               正常
*   @retval         1               错误
*/
double G_AntennaGain_surface_P(
	double* f_surface,
	const double theta,
	const double theta_t,
	const double theta_b)
{

	//主瓣
	double k_main=4*log(sqrt(2)/(theta_t*theta_t));
	double f_main=exp(-k_main*theta*theta);
	//主瓣以外
	double k_other=1.3916/(sin(0.5*theta_b));
	double f_other=0;
	if(sin(theta)==0)
		f_other=0;
	else

		f_other=((1+cos(theta))/2)*(sin(k_other*sin(theta))/(k_other*sin(theta)));

	double f=(f_main>=f_other)?f_main:f_other;
	*f_surface = f;



	return 0;
}

//// --------------------------------------------------------------------------------------------------------------------------------
///**
//*   @brief          计算竖直面某一个角度的天线增益。
//*   @details        计算竖直面某一个角度的天线增益(参数待修改)。
//*   @param[out]     f_surface                    平面某一个角度的天线增益系数
//*   @param[in]      theta                           目标相对雷达的探测俯仰角
//*   @param[in]      theta_t                         主瓣波束宽度 //默认值0.15
//*   @param[in]      theta_b                         单程半功率点波束宽度 //默认值1.2
//*   @retval         0               正常
//*   @retval         1               错误
//*/
//double G_AntennaGain_surface_V(
//	double* f_surface,
//	const double theta,
//	const double theta_t,
//	const double theta_b)
//{
//	//主瓣
////	double k_main=4*log(sqrt(2)/(theta_t*theta_t));
//	double f_main=sin(0.5*2.4+theta)/sqrt(2)*1.1;;
//	//主瓣以外
//	double k_other=1.3916/(sin(0.5*theta_b));
//	double f_other=0;
//	if(sin(theta)==0)
//		f_other=0;
//	else
//
//		f_other=1.4*((1+cos(theta))/2)*(sin(k_other*sin(theta))/(k_other*sin(theta)));
//
//	double f=(f_main>=f_other)?f_main:f_other;
////	double G=nG*Gmax*f;
//
//	*f_surface = f;
//	return 0;
//}

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
	const double pitch_radar_width)
{
	//天线增益的计算公式取值范围是-180度~180度，探测方位角的范围是0-360度，需要进行转换
	double f_azimuth=0;
	double new_azimuth_TargetToRadar=0;
	double temp1,temp2;
	deg2rad(&temp1,180);
	deg2rad(&temp2,360);
	if(azimuth_TargetToRadar>temp1)
		new_azimuth_TargetToRadar=-(temp2-azimuth_TargetToRadar);
	else
		new_azimuth_TargetToRadar=azimuth_TargetToRadar;
	//

	if(new_azimuth_TargetToRadar>=-azimuth_radar_width/2.0 && new_azimuth_TargetToRadar<=azimuth_radar_width/2.0)
		G_AntennaGain_surface_P(&f_azimuth, new_azimuth_TargetToRadar,  theta_t,  theta_b);
	else
		f_azimuth=0;
	//

	double f_pitch=0;
	if(pitch_TargetToRadar>=-pitch_radar_width/2.0 && pitch_TargetToRadar<=pitch_radar_width/2.0)
		G_AntennaGain_surface_P(&f_pitch, pitch_TargetToRadar, theta_t,  theta_b);
	else
		f_pitch=0;
	//

//	*Gain = sqrt(G_azimuth*G_azimuth + G_pitch*G_pitch);
	*Gain = nG*Gmax*sqrt(f_azimuth*f_pitch);
	return 0;
}

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
	const double Lr)
{
	*EchoPower = (Pgt*Ggt*Ggt*Sigima*Lambda*Lambda/(pow(4*M_PI,3)*pow(R,4)))*Lr;
	return 0;
}

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
//	const double Nf)
//{
//	double k=1.380649E-23 ;//玻尔兹曼常数
//	double T=290;
//	double Sn2=k*T*Br*Nf;
//	double x,y;
//	gaussrand(&x);
//	gaussrand(&y);
//
//	double Pin=0.5*Sn2*(x*x+y*y);
//
//	*noise = Pin;
//	return 0;
//}


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
//	const double Pj)
//{
//
//	*SNt = 10*log10(Pgr/(Pin+Pcs+Pj));
//	return 0;
//}



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
//	const double SNt)
//{
//	const int list_size=8;
//	double SNt_list[list_size+1]={10.2,11.2,12.5,13.8,15.0,16.6,19.8,29.9,9999.9};
//	double Pd_list[list_size+1]={0.3,0.4,0.5,0.6,0.7,0.8,0.9,0.98,1};
//	double Pd=0;
//
//	for(int i=0;i<list_size;i++){
//		if(SNt>=SNt_list[i] && SNt<SNt_list[i+1]){//将表格线性化求连续的Pd值
//			Pd=(SNt-SNt_list[i])/(SNt_list[i+1]-SNt_list[i])*(Pd_list[i+1]-Pd_list[i])+Pd_list[i];
//			break;
//		}
//	}
//	if(SNt<SNt_list[0])
//		Pd=SNt/SNt_list[0]*Pd_list[0];
//	if(SNt<0)
//		Pd=0;
//
//	*Pd_out = Pd;
//	return 0;
//}


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
	const double Pgr)
{
//	double Pd=0;
//	getPd(&Pd,SNt);
//	double alpha=(double)rand() / RAND_MAX;//生成一个0-1均匀分布的随机数
//	if(alpha < Pd && n>=2)
//		*finded = true;
//	else
//		if(n>=3)
//			*finded = true;
//		else
//			*finded = false;

	if (Pgr<2e-016)//
		*finded=false;
	else
		*finded=true;

	return 0;
}



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
	const double Lp)
{
	*Rmax=sqrt(Pt*Gt*G*Lambda*Lambda/(pow(4*M_PI,2)*P*Ltx*Lp));
	return 0;
}

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
	const double pitch_radar_max)
{
	if( f>=f_min && f<=f_max && (azimuth_alarm>=azimuth_alarm_min || azimuth_alarm<=azimuth_alarm_max) && (pitch_alarm>=pitch_alarm_min && pitch_alarm<=pitch_alarm_max) && R<=Rmax &&
	    (azimuth_radar>=azimuth_radar_min || azimuth_radar<=azimuth_radar_max) && (pitch_radar>=pitch_radar_min && pitch_radar<=pitch_radar_max))
		*alarm = true;
	else
		*alarm = false;

	return 0;
}

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
	const double pitch_radar_width)
{
	//重定义坐标和姿态角的坐标轴方向：
	//定义导航坐标xyz轴分别对应北天东方向，定义机体坐标系xyz轴分别对应前上右方向
	//如果和输入量定义不同在下方更改
	double RadarPosition[3];
	RadarPosition[0]=positionRadar[0];
	RadarPosition[1]=positionRadar[1];
	RadarPosition[2]=positionRadar[2];

	double RadarAttitude[3];
	RadarAttitude[0]=attitudeRadar[0];
	RadarAttitude[2]=attitudeRadar[1];
	RadarAttitude[3]=attitudeRadar[2];

	double TargetPosition[3];
	TargetPosition[0]=positionTarget[0];
	TargetPosition[1]=positionTarget[1];
	TargetPosition[2]=positionTarget[2];

	double TargetAttitude[3];
	TargetAttitude[0]=attitudeTarget[0];
	TargetAttitude[1]=attitudeTarget[2];
	TargetAttitude[2]=attitudeTarget[1];


	//计算两机距离
	double distance=0;
	getDistance(&distance,RadarPosition,TargetPosition);

	//计算目标相对雷达的探测角
	double azimuth_target_to_radar=0,pitch_target_to_radar=0;
	getTargetAzimuthPitch(&azimuth_target_to_radar,&pitch_target_to_radar,RadarPosition,RadarAttitude,TargetPosition);
//	cout<<rad2deg(azimuth_target_to_radar)<<"____"<<rad2deg(pitch_target_to_radar)<<"___"<<endl;
	//计算雷达相对目标的探测角
	double azimuth_radar_to_target=0,pitch_radar_to_target=0;
	getTargetAzimuthPitch(&azimuth_radar_to_target,&pitch_radar_to_target,TargetPosition,TargetAttitude,RadarPosition);

	//判断视距
	double sight=0;
	getRadarSight(&sight,RadarPosition[1],TargetPosition[1]);
	if(sight<distance)
		*findedTarget = false;
	else{
		//计算天线增益
		double AntennaGain=0;
		getTargetAntennaGain(&AntennaGain,azimuth_target_to_radar,pitch_target_to_radar,
					nG,theta_t,theta_b,Gmax,azimuth_radar_width,pitch_radar_width);

		//计算目标RCS
		double TargetRCS=0;
		getTargetRCS(&TargetRCS,azimuth_radar_to_target,pitch_radar_to_target,SigimaType);

		//计算雷达回波功率
		double EchoPower=0;
		getTargetEchoPower(&EchoPower,Pgt,AntennaGain,TargetRCS,Lambda,distance,Lr);
		*Pgr = EchoPower;
//		//计算接收机噪声
//		double Noise=0;
//		getReceivingNoise(&Noise,Br,Nf);

//		//计算接收机综合信噪比
//		double SNt=0;
//		getSignalNoiseRatio(&SNt,EchoPower,Noise,0,0);
//
//		getPd(Pd,SNt);
		confirmTarget(findedTarget,EchoPower);
	}

	return 0;
}


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
	const double pitch_radar_width)
{
	//重定义坐标和姿态角的坐标轴方向：
	//定义导航坐标xyz轴分别对应北天东方向，定义机体坐标系xyz轴分别对应前上右方向
	//如果和输入量定义不同在下方更改
	double RadarPosition[3];
	RadarPosition[0]=positionRadar[0];
	RadarPosition[1]=positionRadar[1];
	RadarPosition[2]=positionRadar[2];

	double RadarAttitude[3];
	RadarAttitude[0]=attitudeRadar[0];
	RadarAttitude[2]=attitudeRadar[1];
	RadarAttitude[3]=attitudeRadar[2];

	double TargetPosition[3];
	TargetPosition[0]=positionTarget[0];
	TargetPosition[1]=positionTarget[1];
	TargetPosition[2]=positionTarget[2];

	double TargetAttitude[3];
	TargetAttitude[0]=attitudeTarget[0];
	TargetAttitude[1]=attitudeTarget[2];
	TargetAttitude[2]=attitudeTarget[1];

	//计算两机距离
	double distance=0;
	getDistance(&distance,RadarPosition,TargetPosition);


	//计算目标相对雷达的探测角
	double azimuth_target_to_radar=0,pitch_target_to_radar=0;
	getTargetAzimuthPitch(&azimuth_target_to_radar,&pitch_target_to_radar,RadarPosition,RadarAttitude,TargetPosition);

	//计算雷达相对目标的探测角
	double azimuth_radar_to_target=0,pitch_radar_to_target=0;
	getTargetAzimuthPitch(&azimuth_radar_to_target,&pitch_radar_to_target,TargetPosition,TargetAttitude,RadarPosition);

	//判断视距
	double sight=0;
	getRadarSight(&sight,RadarPosition[1],TargetPosition[1]);
	if(sight<distance)
		*alarm_on = false;
	else{
		//计算天线增益
		double AntennaGain=0;
		getTargetAntennaGain(&AntennaGain,azimuth_target_to_radar,pitch_target_to_radar,
					nG,theta_t,theta_b,Gmax,azimuth_radar_width,pitch_radar_width);

		//计算雷达告警器接收机探测到某辐射源的最大距离
		double Rmax=0;
		getAlarmMaxDistance(&Rmax,P,Pgt,Lambda,AntennaGain,G,Ltx,Lp);

		double c=299792458;//光速

		double azimuth_alarm_min=2*M_PI-azimuth_alarm_width/2.0;
		double azimuth_alarm_max=azimuth_alarm_width/2.0;
		double pitch_alarm_min=-pitch_alarm_width/2.0,pitch_alarm_max=pitch_alarm_width/2.0;

		double azimuth_radar_min=2*M_PI-azimuth_radar_width/2.0;
		double azimuth_radar_max=azimuth_radar_width/2.0;
		double pitch_radar_min=-pitch_radar_width/2.0,pitch_radar_max=pitch_radar_width/2.0;


		confirmAlarm(alarm_on,c/Lambda/*光速除波长等于频率*/,f_min,f_max,azimuth_radar_to_target,
			azimuth_alarm_min,azimuth_alarm_max, pitch_radar_to_target, pitch_alarm_min,pitch_alarm_max, distance,Rmax,
			azimuth_target_to_radar,azimuth_radar_min,azimuth_radar_max, pitch_target_to_radar, pitch_radar_min,pitch_radar_max);

	}

	return 0;
}

