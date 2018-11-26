/* Copyright (C) 2015-2017
 *
 * imu_xsens.c
 *
 * Fabian Girrbach     <fabiangirrbach@gmail.com>
 * Elias Rosch         <eliasrosch@gmail.com>
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
 *
 * Systems Control and Optimization Laboratory - www.syscop.de
 * Institute for Microsystems Engineering - www.imtek.de
 * Faculty of Engineering - www.tf.uni-freiburg.de
 * University of Freiburg - www.uni-freiburg.de
 *
 * This file is part of the TOPCORE platform - topcore.syscop.de
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 */
#include "imu_xsens.h"
#include "hal_mcu.h"
#include "hal_timer.h"
#include "hal_uart.h"
#include "cirular_buffer.h"

#include "xbus/xbusmessage.h"
#include "xbus/xbusparser.h"
#include "xbus/xsdeviceid.h"





/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */
#define QUEUE_SIZE 32
#define MAX_XBUS_DATA_SIZE 128
#define READ_SIZE MAX_XBUS_DATA_SIZE
#define RESET_GPIO GPIO_PIN_0

struct S_IMU_XSENS_CTX_T
{
	XsensConnType connectionType;
	bool connectionInitialized;
	s_hal_uart_ctx_t* uart;
	struct XbusParser* xbusParser;
	uint32_t deviceID;
	int32_t deviceFunction;
	Imu protoData;
	uint8_t maxMeasurementCount;
	ImuConf configuration;
} ;


/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
void* g_queue_data;
void* g_queue_response;
s_imu_xsens_ctx_t gs_xsens_imu_ctx;

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static void* loc_allocateMessageData(size_t bufSize);
static void loc_deallocateMessageData(void *const buffer);
static void loc_mtMessageHandler(XbusMessage const* message);
static bool loc_sendCommand(s_imu_xsens_ctx_t const* cntxt, XsMessageId cmdId);
static uint32_t loc_readDeviceId(s_imu_xsens_ctx_t const* cntxt);
static bool loc_setOutputConfiguration(s_imu_xsens_ctx_t const* cntxt, OutputConfiguration const* conf, uint8_t elements);
static void loc_sendMessage(s_imu_xsens_ctx_t const* cntx, XbusMessage const* m);
static XbusMessage* loc_doTransaction(s_imu_xsens_ctx_t const* cntx, XbusMessage const* m);
static void loc_flushReadingBuffer(s_imu_xsens_ctx_t const* cntx);
static void loc_dumpMessage(XbusMessage* message);


/* ===========================================================================*/
/*                  Global function implementations                            */
/* ===========================================================================*/

/*!
 * \brief Create instance of Xsens MTi IMU
 * \param connType Type of connection used
 * \param speed frequency or baudrate setting
 *
 * \return pointer to context of IMU instance
 * which has to passed to all usage functions
 */
s_imu_xsens_ctx_t* xsens_createImu(XsensConnType connType, uint32_t speed)
{
	s_imu_xsens_ctx_t* ret = NULL;
	//Reset memory in struct to 0
	memset( &gs_xsens_imu_ctx, 0, sizeof(gs_xsens_imu_ctx) );
	//Do initialization of communication port
	switch (connType){
	case xsensSPI:
		break;
	case xsensI2C:
		break;
	case xsensUART:
		gs_xsens_imu_ctx.uart = hal_uart_init(E_HAL_UART_PORT_0, speed);
		if (gs_xsens_imu_ctx.uart != NULL){
			gs_xsens_imu_ctx.connectionType = xsensUART;
			gs_xsens_imu_ctx.connectionInitialized = true;
		}
		break;
	default:
		break;
	}

	if (gs_xsens_imu_ctx.connectionInitialized)
	{
		//Setting up callbacks for Message parser
		g_queue_data = circular_buffer_init(32);
		g_queue_response = circular_buffer_init(32);
		XbusParserCallback xbusCallback = {};
		xbusCallback.allocateBuffer = loc_allocateMessageData;
		xbusCallback.deallocateBuffer = loc_deallocateMessageData;
		xbusCallback.handleMessage = loc_mtMessageHandler;
		gs_xsens_imu_ctx.xbusParser = XbusParser_create(&xbusCallback);
		// Initalize Protobuf Imu
		gs_xsens_imu_ctx.protoData.measurements_count = 0;
		gs_xsens_imu_ctx.maxMeasurementCount = (uint8_t) sizeof(gs_xsens_imu_ctx.protoData.measurements) / sizeof(Imu);
		ret = &gs_xsens_imu_ctx;
	}
	return ret;
}

/*!
 * \brief Output the contents of a data message to the PC serial port.
 */
bool xsens_getSensorData(s_imu_xsens_ctx_t const* cntx, Imu * const mdata)
{
	bool ret = false;
	size_t bytes_read = 1;
	uint8_t buffer[READ_SIZE];
	while(bytes_read > 0)
	{
		switch (cntx->connectionType) {
		case xsensUART:
			bytes_read = hal_uart_read(cntx->uart, buffer, READ_SIZE);
			XbusParser_parseBuffer(cntx->xbusParser, buffer, bytes_read);
			break;
		default:
			break;
		}
	}

	if (cntx->protoData.measurements_count > 0){
		*mdata = cntx->protoData;
		ret = true;
	}
	return ret;
}

/*!
 * \brief Sets the motion tracker output configuration based on the function
 * of the attached device.
 *
 * The output configuration depends on the type of MTi-1 device connected.
 * An MTI-1 (IMU) device does not have an onboard orientation filter so
 * cannot output quaternion data, only inertial and magnetic measurement
 * data.
 * MTi-2 and MTi-3 devices have an onboard filter so can send quaternions.
 */
int xsens_configureMotionTracker(s_imu_xsens_ctx_t * cntx)
{
	if (xsens_goToConfigMode(cntx)){

		cntx->deviceID = loc_readDeviceId(cntx);

		if (cntx->deviceID)
		{
			if (!XsDeviceId_isMtMk4_X(cntx->deviceID))
			{
				return -5;
			}

			cntx->deviceFunction = XsDeviceId_getFunction(cntx->deviceID);
			if (cntx->deviceFunction == DF_IMU)
			{
				OutputConfiguration conf[] = {
						{XDI_PacketCounter, 65535},
						{XDI_SampleTimeFine, 65535},
						{XDI_Acceleration, 100},
						{XDI_RateOfTurn, 100},
						{XDI_MagneticField, 100}
				};
				return loc_setOutputConfiguration(cntx,conf, sizeof(conf) / sizeof(OutputConfiguration));
			}
			else
			{
				OutputConfiguration conf[] = {
						{XDI_PacketCounter, 65535},
						{XDI_SampleTimeFine, 65535},
						{XDI_Quaternion, 1},
						{XDI_StatusWord, 65535}
				};
				return loc_setOutputConfiguration(cntx, conf,sizeof(conf) / sizeof(OutputConfiguration));
			}
		}
	}

	return false;
}


/*!
 * \brief Send wakeup message to MTi.
 *
 * Sending a wakeup acknowledge will cause the device to stay in configuration
 * mode instead of automatically transitioning to measurement mode with the
 * stored output configuration.
 */
void xsens_wakeUp(s_imu_xsens_ctx_t const* cntx)
{
	loc_sendCommand(cntx, XMID_Wakeup);
}

/*!
 * \brief Send goToConfig message to MTi.
 *
 * Puts device in mode which allows configuration
 */
bool xsens_goToConfigMode(s_imu_xsens_ctx_t const* cntx)
{
	return loc_sendCommand(cntx, XMID_GotoConfig);

}

/*!
 * \brief Send goToMeasurement message to MTi.
 *
 * Puts device in mode which allows receivin measurment data
 */
bool xsens_goToMeasurmentMode(s_imu_xsens_ctx_t const* cntx)
{
	return loc_sendCommand(cntx, XMID_GotoMeasurement);
}
/* ===========================================================================*/
/*                  Local function implementations                            */
/* ===========================================================================*/

/*!
 * \brief Allocate message data buffer from the message data pool.
 */
static void* loc_allocateMessageData(size_t bufSize)
{
	if (bufSize < MAX_XBUS_DATA_SIZE){
		return malloc(bufSize);
	}
	else{
		return NULL;
	}
}


/*!
 * \brief Deallocate message data previously allocated from the message
 * data pool.
 */
static void loc_deallocateMessageData(void *const buffer)
{
	free(buffer);
}


/*!
 * \brief Dump information from a message to the PC serial port.
 */
static void loc_dumpMessage(XbusMessage* message)
{
	if (message->data)
	{
		loc_deallocateMessageData(message->data);
	}
	free(message);
}

/*!
 * \brief XbusParser callback function to handle received messages.
 * \param message Pointer to the last received message.
 *
 * In this example received messages are copied into one of two message
 * queues for later handling by the main thread. Data messages are put
 * in one queue, while all other responses are placed in the second queue.
 * This is done so that data and other messages can be handled separately
 * by the application code.
 */
static void loc_mtMessageHandler(XbusMessage const* message)
{
	if (message)
	{
		if (message->mid == XMID_MtData2)
		{
			ImuMeasurement* measurement = &gs_xsens_imu_ctx.protoData.measurements[gs_xsens_imu_ctx.protoData.measurements_count];
			uint16_t counter;
			if (XbusMessage_getDataItem(&counter, XDI_PacketCounter, message))
			{
				//measurement->time.has_ticks_measured = true;
				//measurement->time.ticks_measured.onepps_counter = counter;
			}
			uint32_t sampleTime;
			if (XbusMessage_getDataItem(&sampleTime, XDI_SampleTimeFine, message))
			{
				//measurement->time.has_ticks_measured = true;
				//measurement->time.ticks_measured.ticks = sampleTime;
			}
			float ori[4];
			if (XbusMessage_getDataItem(ori, XDI_Quaternion, message))
			{
				measurement->has_quaternion = true;
				measurement->quaternion.x = ori[0];
				measurement->quaternion.y = ori[1];
				measurement->quaternion.z = ori[2];
				measurement->quaternion.w = ori[3];
			}
			float acc[3];
			if (XbusMessage_getDataItem(acc, XDI_Acceleration, message))
			{
				measurement->has_accel = true;
				measurement->accel.x = acc[0];
				measurement->accel.y = acc[1];
				measurement->accel.z = acc[2];

			}
			float gyr[3];
			if (XbusMessage_getDataItem(gyr, XDI_RateOfTurn, message))
			{
				measurement->has_gyro = true;
				measurement->gyro.x = gyr[0];
				measurement->gyro.y = gyr[1];
				measurement->gyro.z = gyr[2];
			}
			float mag[3];
			if (XbusMessage_getDataItem(mag, XDI_MagneticField, message))
			{
				measurement->has_mag = true;
				measurement->mag.x = mag[0];
				measurement->mag.y = mag[1];
				measurement->mag.z = mag[2];
			}
			gs_xsens_imu_ctx.protoData.measurements_count++;
			//Check for overflow of array and reset if necessary
			if (gs_xsens_imu_ctx.protoData.measurements_count >= gs_xsens_imu_ctx.maxMeasurementCount){
				gs_xsens_imu_ctx.protoData.measurements_count = 0;
			}
			loc_deallocateMessageData(message->data);
		}
		else
		{
			XbusMessage* m = malloc(sizeof(XbusMessage));
			if (m)
			{
				memcpy(m, message, sizeof(XbusMessage));
				circular_buffer_push(g_queue_response, (void*) m);
			}
			else if (message->data)
			{
				loc_deallocateMessageData(message->data);
			}
		}
	}
}



/*!
 * \brief Send a command to the MT and wait for a response.
 * \param cmdId The XsMessageId of the command to send.
 *
 * Commands are simple messages without and payload data.
 */
static bool loc_sendCommand(s_imu_xsens_ctx_t const* cntxt, XsMessageId cmdId)
{
	bool ret = false;
	XbusMessage m;
	m.mid = cmdId;
	m.length = 0;
	loc_flushReadingBuffer(cntxt);
	XbusMessage* response = loc_doTransaction(cntxt,&m);
	if (response->mid == cmdId + 1){
		ret = true;
	}
	if (response)
	{
		loc_dumpMessage(response);
	}
	return ret;
}


/*!
 * \brief Flush reading buffer.
 * \param cntx of Imu.
 *
 * Clears completely the reading buffer.
 */
static void loc_flushReadingBuffer(s_imu_xsens_ctx_t const* cntx)
{
	uint8_t buffer[READ_SIZE];
	switch (cntx->connectionType){
	case xsensUART:
		while(hal_uart_read(cntx->uart,buffer, READ_SIZE));
		break;
	case xsensSPI:
		break;
	case xsensI2C:
		break;
	}
}




/*!
 * \brief Send a message to the MT and wait for a response.
 * \returns Response message from the MT, or NULL is no response received
 * within 500ms.
 *
 * Blocking behaviour is implemented by waiting for a response to be written
 * to the response queue by the XbusParser.
 */
static XbusMessage* loc_doTransaction(s_imu_xsens_ctx_t const* cntx, XbusMessage const* m)
{
	uint8_t buffer[READ_SIZE];
	loc_sendMessage(cntx, m);
	memset(buffer, 0, sizeof(buffer));
	XbusMessage* response = NULL;
	while (response == NULL)
	{
		uint16_t bytes_read = 0;
		switch (cntx->connectionType){
		case xsensUART:
			bytes_read = hal_uart_read(cntx->uart,buffer, READ_SIZE);
			break;
		case xsensSPI:
			break;
		case xsensI2C:
			break;
		}
		if (bytes_read > 0)
		{
			XbusParser_parseBuffer(cntx->xbusParser,buffer, bytes_read);
			response = (XbusMessage*)circular_buffer_pop(g_queue_response);
		}
	}
	return response;
}


/*!
 * \brief Send a message to the MT
 *
 * This function formats the message data and writes this to the MT SPI
 * interface. It does not wait for any response.
 */
static void loc_sendMessage(s_imu_xsens_ctx_t const* cntx, XbusMessage const* m)
{
	uint8_t buf[64];
	memset(&buf[0], 0, sizeof(buf));
	size_t rawLength;
	switch (cntx->connectionType)
	{
	case xsensI2C:
		rawLength = XbusMessage_format(buf, m, XLLF_I2c);

		break;
	case xsensSPI:
		rawLength = XbusMessage_format(buf, m, XLLF_Spi);
		break;
	case xsensUART:
		rawLength = XbusMessage_format(buf, m, XLLF_Uart);
		hal_uart_write(cntx->uart, buf, rawLength);
		break;
	default:
		break;
	}
}



/*!
 * \brief Read the device ID of the motion tracker.
 */
static uint32_t loc_readDeviceId(s_imu_xsens_ctx_t const* cntxt)
{
	XbusMessage reqDid = {XMID_ReqDid, 0 , NULL};
	XbusMessage const* didRsp = loc_doTransaction(cntxt,&reqDid);
	uint32_t deviceId = 0;
	if (didRsp)
	{
		if (didRsp->mid == XMID_DeviceId)
		{
			deviceId = *(uint32_t*)didRsp->data;
		}
	}
	return deviceId;
}



/*!
 * \brief Sets MT output configuration.
 * \param conf Pointer to an array of OutputConfiguration elements.
 * \param elements The number of elements in the configuration array.
 *
 * The response from the device indicates the actual values that will
 * be used by the motion tracker. These may differ from the requested
 * parameters as the motion tracker validates the requested parameters
 * before applying them.
 */
static bool loc_setOutputConfiguration(s_imu_xsens_ctx_t const* cntxt, OutputConfiguration const* conf, uint8_t elements)
{
	XbusMessage outputConfMsg = {XMID_SetOutputConfig, elements, (void*)conf};
	XbusMessage const* outputConfRsp = loc_doTransaction(cntxt,&outputConfMsg);
	if (outputConfRsp)
	{
		if (outputConfRsp->mid == XMID_OutputConfig)
		{
			return true;
		}
	}
	return false;
}



