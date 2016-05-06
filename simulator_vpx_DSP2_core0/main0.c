/* BIOS6 include */
#include <ti/sysbios/BIOS.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/csl/csl_ipcAux.h>
#include <ti/csl/csl_chipAux.h>
#include <ti/csl/csl_cacheAux.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/IHeap.h>

#include <ti/ipc/GateMP.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/ListMP.h>
#include <ti/ipc/SharedRegion.h>
#include <ti/ipc/MultiProc.h>

#include <ti/ipc/Ipc.h>
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/HeapBufMP.h>
#include <ti/ipc/MultiProc.h>

#include <math.h>

#include "../CustomHeaderDsp2.h"

#include<stdlib.h>

#include <ti/sysbios/family/c66/tci66xx/CpIntc.h>
#include <ti/sysbios/hal/Hwi.h>
#include <xdc/runtime/Error.h>

#include "hyplnkLLDIFace.h"
#include "hyplnkLLDCfg.h"
#include "hyplnkIsr.h"
#include "hyplnkPlatCfg.h"

#include "ti/platform/platform.h"


//-------------------------------------------------------------------------
//Will be used to send message
MessageQ_QueueId 	QueueIdCore0ToCore1;
MessageQ_QueueId 	QueueIdCore0ToCore2;
MessageQ_QueueId 	QueueIdCore2ToCore0;

MsgCore0ToCore2* 	Msg0To2Ptr;
MsgCore2ToCore0* 	Msg2To0Ptr;

HeapBufMP_Handle 	HeapHandleCore0ToCore1;
HeapBufMP_Handle 	HeapHandleCore0ToCore2;
HeapBufMP_Handle 	HeapHandleCore2ToCore0;

MessageQ_Handle 	MessageQCore2ToCore0;


/*****************************************************************************
 * Cache line size (128 works on any location)
 *****************************************************************************/
#define hyplnk_EXAMPLE_LINE_SIZE      128
/* This is the actual data buffer that will be accessed.
 *
 * When accessing this location directly, the program is playing
 * the role of the remote device (remember we are in loopback).
 *
 * If this is placed in an L2 RAM there are no other dependancies.
 *
 * If this is placed in MSMC or DDR RAM, then the SMS
 * or SES MPAX must be configured to allow the access.
 */
#pragma DATA_SECTION (HyplinkDataDsp1Dsp2Buffer, "hyperlink");
#pragma DATA_ALIGN (HyplinkDataDsp1Dsp2Buffer, hyplnk_EXAMPLE_LINE_SIZE);
HyplinkDataDsp1Dsp2 HyplinkDataDsp1Dsp2Buffer;
/*
 * This pointer is the local address within the Hyperlink's address
 * space which will access dataBuffer via HyperLink.
 *
 */
HyplinkDataDsp1Dsp2 *bufferThroughHypLnk;


/*************************************************************************
 *  @b EVM_init()
 *
 *  @n
 *
 *  Initializes the platform hardware. This routine is configured to start in
 *   the evm.cfg configuration file. It is the first routine that BIOS
 *   calls and is executed before Main is called. If you are debugging within
 *  CCS the default option in your target configuration file may be to execute
 *  all code up until Main as the image loads. To debug this you should disable
 *  that option.
 *
 *  @param[in]  None
 *
 *  @retval
 *      None
 ************************************************************************/
void EVM_init()
{
 platform_init_flags   sFlags;
 platform_init_config  sConfig;
 /* Status of the call to initialize the platform */
 int32_t pform_status;

 /*
  * You can choose what to initialize on the platform by setting the following
  * flags. Things like the DDR, PLL, etc should have been set by the boot loader.
 */
 memset( (void *) &sFlags,  0, sizeof(platform_init_flags));
 memset( (void *) &sConfig, 0, sizeof(platform_init_config));

    sFlags.pll  = 0; 		/* PLLs for clocking    */
    sFlags.ddr  = 0;     	/* External memory    */
    sFlags.tcsl = 1; 		/* Time stamp counter   */
#ifdef _SCBP6618X_
    sFlags.phy  = 0; /* Ethernet       */
#else
    sFlags.phy  = 1; /* Ethernet       */
#endif
    sFlags.ecc  = 0; /* Memory ECC       */

    sConfig.pllm = 0;  /* Use libraries default clock divisor */

 pform_status = platform_init(&sFlags, &sConfig);

 /* If we initialized the platform okay */
 if (pform_status != Platform_EOK) {
   /* Initialization of the platform failed... die */
   while (1) {
     (void) platform_led(1, PLATFORM_LED_ON, PLATFORM_USER_LED_CLASS);
     (void) platform_delay(50000);
     (void) platform_led(1, PLATFORM_LED_OFF, PLATFORM_USER_LED_CLASS);
     (void) platform_delay(50000);
   }
 }
}

/* 初始化Hyperlink */
void HyperlinkInit()
{
	TSCL = 0;
	TSCH = 0;

	int retVal;

	/* Set up the system PLL, PSC, and DDR as required for this HW */
	System_printf ("About to do Hyperlink system setup (PLL, PSC, and DDR)\n");
	if ((retVal = hyplnkExampleSysSetup()) != hyplnk_RET_OK) {
		System_printf ("Hyperlink system setup failed (%d)\n", (int)retVal);
		exit(1);
	}
	System_printf ("Hyperlink system setup worked\n");


	/* Enable the peripheral */
	System_printf ("About to set up HyperLink Peripheral\n");
	if ((retVal = hyplnkExamplePeriphSetup()) != hyplnk_RET_OK) {
		System_printf ("HyperLink system setup failed (%d)\n", (int)retVal);
		exit(1);
	}
	System_printf ("HyperLink Peripheral setup worked\n");

	/* Hyperlink 初始化 */
	/* Hyperlink is not up yet, so we don't have to worry about concurrence */
	memset (&HyplinkDataDsp1Dsp2Buffer, 0, sizeof(HyplinkDataDsp1Dsp2Buffer));

	/* Set up address mapsrc */
	if ((retVal = hyplnkExampleAddrMap (&HyplinkDataDsp1Dsp2Buffer, (void **)&bufferThroughHypLnk)) != hyplnk_RET_OK) {
		System_printf ("Address map setup failed (%d)\n", (int)retVal);
		exit(1);
	}

}

/* Initialize IPC and MessageQ */
void IPC_init()
{
    Int                 status;
    HeapBufMP_Handle    heapHandle;

    //Ipc_start does not Ipc_attach, because 'Ipc.procSync' is set to 'Ipc.ProcSync_PAIR' in *.cfg
    //Ipc reserves some shared memory in SharedRegion zero for synchronization.
    status = Ipc_start();
    if (status < 0)
    {
		System_abort("Ipc_start failed\n");
    }
    /*--------------- Wait IPC attach to other cores ----------------*/
    //!!!! Must attach to the share memory owner - CORE1 firstly
    while ((status = Ipc_attach(PROC_ID_CORE1)) < 0)
	{
    	Task_sleep(1) ;
	}
	while ((status = Ipc_attach(PROC_ID_CORE2)) < 0)
	{
		Task_sleep(1) ;
	}
    System_printf("Ipc_attach() finished.\n");

//    /*------------------------- To CORE1 ----------------------*/
//	do {
//		status = HeapBufMP_open(HEAP_BUF_NAME_CORE0_CORE1, &HeapHandleCore0ToCore1);
//		Task_sleep(1) ;
//	} while (status < 0);
//	System_printf("HeapBufMP_open HEAP_BUF_NAME_CORE0_CORE1 finished. \n");
//
//	do {
//		status = MessageQ_open(MSGQ_NAME_CORE0_CORE1, &QueueIdCore0ToCore1);
//		Task_sleep(1) ;
//	} while (status < 0);
//	System_printf("MessageQ_open MSGQ_NAME_CORE0_CORE1 finished. \n");
//
//	status = MessageQ_registerHeap((IHeap_Handle)HeapHandleCore0ToCore1, HEAP_ID_CORE0_CORE1);
//
//	Msg0To1Ptr = (MsgCore0ToCore1*)MessageQ_alloc(HEAP_ID_CORE0_CORE1, sizeof(MsgCore0ToCore1));
//	if (Msg0To1Ptr == NULL)
//	{
//	   System_abort("MessageQ_alloc failed\n" );
//	}
//	else
//	{
//		System_printf("MessageQ_alloc(HEAP_ID_CORE0_CORE1, sizeof(MsgCore0ToCore1)) finished. \n");
//	}

	/*------------------------- To CORE2 ----------------------*/
	do {
		status = HeapBufMP_open(HEAP_BUF_NAME_CORE0_CORE2, &HeapHandleCore0ToCore2);
		System_printf("status = HeapBufMP_open(HEAP_BUF_NAME_CORE0_CORE2, &HeapHandleCore0ToCore2);\n");
		Task_sleep(1) ;
	} while (status < 0);
	System_printf("HeapBufMP_open HEAP_BUF_NAME_CORE0_CORE2 finished. \n");

	do {
		status = MessageQ_open(MSGQ_NAME_CORE0_CORE2, &QueueIdCore0ToCore2);
		Task_sleep(1) ;
	} while (status < 0);
	System_printf("MessageQ_open MSGQ_NAME_CORE0_CORE2 finished. \n");

	MessageQ_registerHeap((IHeap_Handle)HeapHandleCore0ToCore2, HEAP_ID_CORE0_CORE2);

	Msg0To2Ptr= (MsgCore0ToCore2*)MessageQ_alloc(HEAP_ID_CORE0_CORE2, sizeof(MsgCore0ToCore2));
	if (Msg0To2Ptr == NULL)
	{
	   System_abort("MessageQ_alloc failed\n" );
	}
	else
	{
		System_printf("MessageQ_alloc(HEAP_ID_CORE0_CORE2, sizeof(MsgCore0ToCore23456)) finished. \n");
	}

	/* Create the heap that will be used to allocate messages. */
	//---------------For CORE2--------------------------
//	do {
//		status = HeapBufMP_open(HEAP_BUF_NAME_CORE2_CORE0, &heapHandle);
//		Task_sleep(1) ;
//	} while (status < 0);
//	System_printf("HeapBufMP_open(HeapBufMPStringFromCore2, &heapHandle); \n");
	/* Create the local message queue */
	MessageQCore2ToCore0 = MessageQ_create(MSGQ_NAME_CORE2_CORE0, NULL);
	if (MessageQCore2ToCore0 == NULL)
	{
		System_abort("MessageQ_create failed\n");
	}
	System_printf("MessageQ_create(MSGQ_NAME_CORE2_CORE0, NULL); \n");

//	//-------获得散射点信息存储空间-----------------
//	SharedRegion_SRPtr ScatteringPointSrPtr;	//共享存储空间下的指针
//	ScatteringPointPtr = Memory_alloc(SharedRegion_getHeap(1), sizeof(ScatteringPoint), 128, NULL);	//128为对齐字节，可设置与cacheline相同，此处128为临时设置
//	if(ScatteringPointPtr == NULL)
//	{
//		System_printf("SharedRegion_getHeap() failed!!\n");
//	}
////	ScatteringPointSrPtr = SharedRegion_getSRPtr((Ptr)ScatteringPointPtr, 1);
//
//	status = Notify_sendEvent(PROC_ID_CORE2, LINE_ID_CORE0_CORE2, EVENT_ID_CORE0_CORE2, (Uint32)ScatteringPointPtr, TRUE);
//	if(status < 0)
//	{
//		System_printf("Notify_sendEvent() failed!!\n");
//	}

	System_printf("Debug(Core 0): IPC_init() finished. \n");
}

void MainThread()
{
	int status;
	int CoreNum;
	String 	HeapBufMPStringFromCore2;
	String 	MessageQStringFromCore2;
	String	MessageQStringToCore1;
	int		MsgID = 0;

	MsgCore0ToCore2 	MsgCore0ToCore2Data;
	MsgCore2ToCore0		MsgCore2ToCore0Data;



	/* Initialize IPC and MessageQ */
	IPC_init();

	while(1)
	{
		if(HyplinkDataDsp1Dsp2Buffer.NewFlag == 1)	//等待接收到新数据
		{
			CACHE_invL1d(&HyplinkDataDsp1Dsp2Buffer, sizeof(HyplinkDataDsp1Dsp2), CACHE_WAIT);
			HyplinkDataDsp1Dsp2Buffer.NewFlag = 0;	//表示接收到了Hyperlink帧
			System_printf("接收到Hyplink数据\n");

			//无干扰
			if(HyplinkDataDsp1Dsp2Buffer.JammingFrameId == NO_JAMMING)
			{
				/* Notify core1 无干扰 */
				status = Notify_sendEvent(PROC_ID_CORE1, LINE_ID_CORE0_CORE1, EVENT_ID_CORE0_CORE1, NO_JAMMING, TRUE);
				if (status < 0)
				{
					System_printf("Notify_sendEvent had a failure/error, error code is %d \n", status);
					System_abort(" ");
				}
				System_printf("Notify_sendEvent(PROC_ID_CORE1, LINE_ID_CORE0_CORE1, EVENT_ID_CORE0_CORE1, WORK_PARAM_SET_NOTIFY, TRUE)\n");
			}
			//间歇采样转发式干扰
			else if((HyplinkDataDsp1Dsp2Buffer.JammingFrameId >> 16) == ISRJ)
			{
				MessageQ_setMsgId(&(MsgCore0ToCore2Data.header), MsgID++);
				memcpy(&MsgCore0ToCore2Data.TargetFrameId, &HyplinkDataDsp1Dsp2Buffer.TargetFrameId, sizeof(MsgCore0ToCore2) - sizeof(MessageQ_MsgHeader));

				/* Send message to core 2. */
				status = MessageQ_put(QueueIdCore0ToCore2, &(Msg0To2Ptr->header));
				if (status < 0)
				{
					System_abort("MessageQ_put had a failure/error\n");
				}
				System_printf("MessageQ_put(QueueIdCore0ToCore2, &(Msg0To2Ptr->header))\n");

				/* Notify core1 间歇采样转发式干扰 */
				status = Notify_sendEvent(PROC_ID_CORE1, LINE_ID_CORE0_CORE1, EVENT_ID_CORE0_CORE1, ISRJ, TRUE);
				if (status < 0)
				{
					System_printf("Notify_sendEvent had a failure/error, error code is %d \n", status);
					System_abort(" ");
				}
				System_printf("Notify_sendEvent(PROC_ID_CORE1, LINE_ID_CORE0_CORE1, EVENT_ID_CORE0_CORE1, WORK_PARAM_SET_NOTIFY, TRUE)\n");

				/* 判断是否有回传 */
				if(((HyplinkDataDsp1Dsp2Buffer.TargetFrameId >> 16) == MANUAL_MODE)||
                   ((HyplinkDataDsp1Dsp2Buffer.TargetFrameId >> 16) == AUTO_MODE_PASS_BACK))
				{
					if(HyplinkDataDsp1Dsp2Buffer.JammingFrameId == ISRJ_0)	//不需要core2回传
					{
						HyplinkDataDsp1Dsp2Buffer.JammingParamDsp2ToDsp1.JammingIsrjParam0HyplinkBack = 0x01;
					}
					else	//需要core2回传
					{
						while(MessageQ_get(MessageQCore2ToCore0, (MessageQ_Msg *)&MsgCore2ToCore0Data, MessageQ_FOREVER) != 0);
						HyplinkDataDsp1Dsp2Buffer.JammingParamDsp2ToDsp1.JammingIsrjParam1HyplinkBack.JammingDistance =
										MsgCore2ToCore0Data.JammingParamBack.JammingIsrjParam1HyplinkBack.JammingDistance;
						HyplinkDataDsp1Dsp2Buffer.JammingParamDsp2ToDsp1.JammingIsrjParam1HyplinkBack.JammingTheta =
																MsgCore2ToCore0Data.JammingParamBack.JammingIsrjParam1HyplinkBack.JammingTheta;
						HyplinkDataDsp1Dsp2Buffer.JammingParamDsp2ToDsp1.JammingIsrjParam1HyplinkBack.JammingPhi =
																MsgCore2ToCore0Data.JammingParamBack.JammingIsrjParam1HyplinkBack.JammingPhi;
					}
					CACHE_invL1d(&MessageQCore2ToCore0, sizeof(MsgCore2ToCore0), CACHE_WAIT);	//从cache中invalid
				}
			}
		}
	}
}

//---------------------------------------------------------------------
// Main Entry Point
//---------------------------------------------------------------------
int main()
{
	HyperlinkInit();

	EVM_init();

	/* Start the BIOS 6 Scheduler */
	BIOS_start ();
}
