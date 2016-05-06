#include <math.h>
#include <stdlib.h>

#include "../CustomHeaderDsp2.h"


/* 顺序按照OY、OZ、OX依次旋转phi、theta、gamma */
//计算坐标,已知从A坐标系转换到B坐标系的旋转关系，求A坐标系中点在B坐标系下坐标
void CoordinateCalculateOriginToTrans(float OriginToTransAngleGamma, float OriginToTransAnglePhi, float OriginToTransAngleTheta,
                                   float OriginalX, float OriginalY, float OriginalZ,
                                   float *TransX, float *TransY, float *TransZ)
{
    float TransMatrix[3][3];

    float OriginToTransAnglePhiRad = OriginToTransAnglePhi / 180 * 3.1415926;
    float OriginToTransAngleThetaRad = OriginToTransAngleTheta / 180 * 3.1415926;
    float OriginToTransAngleGammaRad = OriginToTransAngleGamma / 180 * 3.1415926;

    float CosPhi = cos(OriginToTransAnglePhiRad);
    float SinPhi = sin(OriginToTransAnglePhiRad);
    float CosTheta = cos(OriginToTransAngleThetaRad);
    float SinTheta = sin(OriginToTransAngleThetaRad);
    float CosGamma = cos(OriginToTransAngleGammaRad);
    float SinGamma = sin(OriginToTransAngleGammaRad);

    TransMatrix[0][0] = CosPhi*CosTheta;
    TransMatrix[0][1] = -CosPhi*SinTheta*CosGamma + SinPhi*SinGamma;
    TransMatrix[0][2] = CosPhi*SinTheta*SinGamma + SinPhi*CosGamma;
    TransMatrix[1][0] = SinTheta;
    TransMatrix[1][1] = CosTheta*CosGamma;
    TransMatrix[1][2] = -CosTheta*SinGamma;
    TransMatrix[2][0] = -SinPhi*CosTheta;
    TransMatrix[2][1] = SinPhi*SinTheta*CosGamma + CosPhi*SinGamma;
    TransMatrix[2][2] = -SinPhi*SinTheta*SinGamma + CosPhi*CosGamma;

    *TransX = OriginalX*TransMatrix[0][0] + OriginalY*TransMatrix[0][1] + OriginalZ*TransMatrix[0][2];
    *TransY = OriginalX*TransMatrix[1][0] + OriginalY*TransMatrix[1][1] + OriginalZ*TransMatrix[1][2];
    *TransZ = OriginalX*TransMatrix[2][0] + OriginalY*TransMatrix[2][1] + OriginalZ*TransMatrix[2][2];
}

//计算坐标,已知从A坐标系转换到B坐标系的旋转关系，求B坐标系中点在A坐标系下坐标
void CoordinateCalculateTransToOrigin(float TransToOriginAngleGamma, float TransToOriginAnglePhi, float TransToOriginAngleTheta,
                                   float OriginalX, float OriginalY, float OriginalZ,
                                   float *TransX, float *TransY, float *TransZ)
{
    float TransMatrix[3][3];

    float TransToOriginAnglePhiRad = TransToOriginAnglePhi / 180 * 3.1415926;
    float TransToOriginAngleThetaRad = TransToOriginAngleTheta / 180 * 3.1415926;
    float TransToOriginAngleGammaRad = TransToOriginAngleGamma / 180 * 3.1415926;

    float CosPhi = cos(TransToOriginAnglePhiRad);
    float SinPhi = sin(TransToOriginAnglePhiRad);
    float CosTheta = cos(TransToOriginAngleThetaRad);
    float SinTheta = sin(TransToOriginAngleThetaRad);
    float CosGamma = cos(TransToOriginAngleGammaRad);
    float SinGamma = sin(TransToOriginAngleGammaRad);

    TransMatrix[0][0] = CosPhi*CosTheta;
    TransMatrix[0][1] = SinTheta;
    TransMatrix[0][2] = -SinPhi*CosTheta;
    TransMatrix[1][0] = -CosPhi*SinTheta*CosGamma + SinPhi*SinGamma;
    TransMatrix[1][1] = CosTheta*CosGamma;
    TransMatrix[1][2] = SinPhi*SinTheta*CosGamma + CosPhi*SinGamma;
    TransMatrix[2][0] = CosPhi*SinTheta*SinGamma + SinPhi*CosGamma;
    TransMatrix[2][1] = -CosTheta*SinGamma;
    TransMatrix[2][2] = -SinPhi*SinTheta*SinGamma + CosPhi*CosGamma;

    *TransX = OriginalX*TransMatrix[0][0] + OriginalY*TransMatrix[0][1] + OriginalZ*TransMatrix[0][2];
    *TransY = OriginalX*TransMatrix[1][0] + OriginalY*TransMatrix[1][1] + OriginalZ*TransMatrix[1][2];
    *TransZ = OriginalX*TransMatrix[2][0] + OriginalY*TransMatrix[2][1] + OriginalZ*TransMatrix[2][2];
}


/* ISRJ参数0计算 */
void IsrjParam0Cal(MsgCore0ToCore2 *Msg0To2Ptr, MsgCore2ToCore1 *Msg2To1Ptr, MsgCore2ToCore34567 *Msg2To34567Ptr)
{
	int i;

	//用距离计算延时
	Msg2To1Ptr->JammingDistanceDelay =
			((Msg0To2Ptr->JammingParam.JammingIsrjParam0Hyplink.Distance)
					/ LIGHT_SPEED * FPGA_CLK_FRE) + DISTANCE_DELAY_COMPENSATION;
	//用速度计算频率控制字
	Msg2To1Ptr->JammingDopplerFrePinc = 2 *
								(Msg0To2Ptr->JammingParam.JammingIsrjParam0Hyplink.Speed)
								* WAVE_FRE / LIGHT_SPEED * 0xffffffff / FPGA_CLK_FRE;
	//用功率计算幅度
	Msg2To1Ptr->JammingAmplitude =
			sqrt(pow(10,((Msg0To2Ptr->JammingParam.JammingIsrjParam0Hyplink.Power - SIG_POEWR_MAX_DBM)/10))
					* AMPLITUDE_MAX * AMPLITUDE_MAX);

	//角度赋值
	Msg2To34567Ptr->JammingAngleTheta = Msg0To2Ptr->JammingParam.JammingIsrjParam0Hyplink.Theta;
	Msg2To34567Ptr->JammingAnglePhi = Msg0To2Ptr->JammingParam.JammingIsrjParam0Hyplink.Phi;
}

/* ISRJ参数1计算 */
void IsrjParam1Cal(MsgCore0ToCore2 *Msg0To2Ptr, MsgCore2ToCore1 *Msg2To1Ptr, MsgCore2ToCore34567 *Msg2To34567Ptr)
{
	int i;

	float	TranR, RecvR;
	float	GroundCoordRadarTranX = Msg0To2Ptr->JammingParam.JammingIsrjParam1Hyplink.GroundCoordRadarTranX;
	float 	GroundCoordRadarTranY = Msg0To2Ptr->JammingParam.JammingIsrjParam1Hyplink.GroundCoordRadarTranY;
	float 	GroundCoordRadarTranZ = Msg0To2Ptr->JammingParam.JammingIsrjParam1Hyplink.GroundCoordRadarTranZ;
	float 	GroundCoordJammingX    = Msg0To2Ptr->JammingParam.JammingIsrjParam1Hyplink.GroundCoordJammingX;
	float 	GroundCoordJammingY    = Msg0To2Ptr->JammingParam.JammingIsrjParam1Hyplink.GroundCoordJammingY;
	float 	GroundCoordJammingZ    = Msg0To2Ptr->JammingParam.JammingIsrjParam1Hyplink.GroundCoordJammingZ;
	float 	GroundCoordRadarRecvX = Msg0To2Ptr->JammingParam.JammingIsrjParam1Hyplink.GroundCoordRadarRecvX;
	float 	GroundCoordRadarRecvY = Msg0To2Ptr->JammingParam.JammingIsrjParam1Hyplink.GroundCoordRadarRecvY;
	float 	GroundCoordRadarRecvZ = Msg0To2Ptr->JammingParam.JammingIsrjParam1Hyplink.GroundCoordRadarRecvZ;

	/* 计算发送和接收距离 */
	TranR = sqrt(	(GroundCoordJammingX - GroundCoordRadarTranX)*(GroundCoordJammingX - GroundCoordRadarTranX) +
					(GroundCoordJammingY - GroundCoordRadarTranY)*(GroundCoordJammingY - GroundCoordRadarTranY) +
					(GroundCoordJammingZ - GroundCoordRadarTranZ)*(GroundCoordJammingZ - GroundCoordRadarTranZ));

	RecvR = sqrt(	(GroundCoordRadarRecvX - GroundCoordJammingX)*(GroundCoordRadarRecvX - GroundCoordJammingX) +
					(GroundCoordRadarRecvY - GroundCoordJammingY)*(GroundCoordRadarRecvY - GroundCoordJammingY) +
					(GroundCoordRadarRecvZ - GroundCoordJammingZ)*(GroundCoordRadarRecvZ - GroundCoordJammingZ));

	float	GroundCoordToRadarCoordAngleX  = Msg0To2Ptr->JammingParam.JammingIsrjParam1Hyplink.GroundCoordToRadarCoordAngleX;
	float	GroundCoordToRadarCoordAngleY  = Msg0To2Ptr->JammingParam.JammingIsrjParam1Hyplink.GroundCoordToRadarCoordAngleY;
	float	GroundCoordToRadarCoordAngleZ  = Msg0To2Ptr->JammingParam.JammingIsrjParam1Hyplink.GroundCoordToRadarCoordAngleZ;

	/* 计算目标在阵面坐标系下的方位和俯仰角 */
	float	JammingRadarX, JammingRadarY, JammingRadarZ;
	CoordinateCalculateOriginToTrans(
					GroundCoordToRadarCoordAngleX, GroundCoordToRadarCoordAngleY, GroundCoordToRadarCoordAngleZ,
					GroundCoordJammingX - GroundCoordRadarRecvX,
					GroundCoordJammingY - GroundCoordRadarRecvY,
					GroundCoordJammingZ - GroundCoordRadarRecvZ,
					&JammingRadarX,
					&JammingRadarY,
					&JammingRadarZ
					);
	//得出阵面坐标系下角度
	Msg2To34567Ptr->JammingAngleTheta = atan(JammingRadarZ/JammingRadarX);
	Msg2To34567Ptr->JammingAnglePhi = atan(JammingRadarY / sqrt(JammingRadarX*JammingRadarX + JammingRadarZ*JammingRadarZ));

	//用距离计算延时
	Msg2To1Ptr->JammingDistanceDelay =
			((TranR + RecvR) / LIGHT_SPEED * FPGA_CLK_FRE) + DISTANCE_DELAY_COMPENSATION;
	//用速度计算频率控制字
	Msg2To1Ptr->JammingDopplerFrePinc = 2 *
								(Msg0To2Ptr->JammingParam.JammingIsrjParam0Hyplink.Speed)
								* WAVE_FRE / LIGHT_SPEED * 0xffffffff / FPGA_CLK_FRE;
	//用功率计算幅度
	Msg2To1Ptr->JammingAmplitude =
			sqrt(pow(10,((Msg0To2Ptr->JammingParam.JammingIsrjParam0Hyplink.Power - SIG_POEWR_MAX_DBM)/10))
					* AMPLITUDE_MAX * AMPLITUDE_MAX);

}
