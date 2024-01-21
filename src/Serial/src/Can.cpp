#include <Can/Can.h>

float quat[4], gyro[3], acc[3], bullet_speed;
int mode;

TPCANMsg send_msg;

int bytesToInt(BYTE bytes[])
{
    // 位操作时 使用一个unsigned int变量来作为位容器。
    int addr = bytes[0] & 0xFF;
    addr |= ((bytes[1] << 8) & 0xFF00);
    addr |= ((bytes[2] << 16) & 0xFF0000);
    addr |= ((bytes[3] << 24) & 0xFF000000);
    return addr;
}

float bytesToFloat(BYTE bytes[])
{
    // 位操作时 使用一个unsigned int变量来作为位容器。
    return *((float*)bytes);
}

void floatTobytes(float data, BYTE bytes[])
{
    // 位操作时 使用一个unsigned int变量来作为位容器。
    size_t length = sizeof(float);

    BYTE *pdata = (BYTE*)&data; //把float类型的指针强制转换为unsigned char型
    for (int i = 0; i < length; i++){
        bytes[i] = *pdata++;//把相应地址中的数据保存到unsigned char数组中
    }
    return;
}

void Can_receive(unsigned int ID, BYTE data[])
{
	switch(ID){
		case 0x402:
			quat[0] = bytesToFloat(data);
			quat[1] = bytesToFloat(data + 4);
		break;
		case 0x403:
			quat[2] = bytesToFloat(data);
			quat[3] = bytesToFloat(data + 4);
		break;
		case 0x404:
			gyro[0] = bytesToFloat(data);
			gyro[1] = bytesToFloat(data + 4);
		break;
		case 0x405:
			gyro[2] = bytesToFloat(data);
			acc[0] = bytesToFloat(data + 4);
		break;
		case 0x406:
			acc[1] = bytesToFloat(data);
			acc[2] = bytesToFloat(data + 4);
		break;
		case 0x407:
			bullet_speed = bytesToFloat(data);
			mode = bytesToInt(data + 4);
		break;
	}

	return ;
}

std::string GetDataString(BYTE data[], TPCANMessageType msgType, int dataLength)
{
	if ((msgType & PCAN_MESSAGE_RTR) == PCAN_MESSAGE_RTR)
		return "Remote Request";
	else
	{
		char strTemp[MAX_PATH] = { 0 };
		std::string result = "";
		for (int i = 0; i < dataLength; i++)
		{
			sprintf_s(strTemp, sizeof(strTemp), "%02X ", data[i]);
			result.append(strTemp);
		}
		return result;
	}
}

//====================================================//

TimerRead::TimerRead()
{
	ShowCurrentConfiguration(); // Shows the current parameters configuration

	// Checks if PCANBasic.dll is available, if not, the program terminates
	m_DLLFound = CheckForLibrary();
	if (!m_DLLFound)
		return;

	TPCANStatus stsResult;
	// Initialization of the selected channel
	if (IsFD)
		stsResult = CAN_InitializeFD(PcanHandle, BitrateFD);
	else
		stsResult = CAN_Initialize(PcanHandle, Bitrate);

	if (stsResult != PCAN_ERROR_OK)
	{
		std::cout << "Can not initialize. Please check the defines in the code.\n";
		ShowStatus(stsResult);
		return;
	}

	// Reading messages...
	std::cout << "Successfully initialized.\n";
	m_TimerOn = true;
	std::cout << "Started reading messages...\n";
}

TimerRead::~TimerRead()
{
	m_TimerOn = false;
	if (m_DLLFound)
		CAN_Uninitialize(PCAN_NONEBUS);
}

void TimerRead::TimerThread()
{
	while (m_TimerOn)
	{
		usleep(TimerInterval*1000);
		ReadMessages();
	}
}

void TimerRead::ReadMessages()
{
	TPCANStatus stsResult;

	// We read at least one time the queue looking for messages. If a message is found, we look again trying to
	// find more. If the queue is empty or an error occurr, we get out from the dowhile statement.
	do
	{
		stsResult = IsFD ? ReadMessageFD() : ReadMessage();
		if (stsResult != PCAN_ERROR_OK && stsResult != PCAN_ERROR_QRCVEMPTY)
		{
			ShowStatus(stsResult);
			return;
		}
	} while (!(stsResult & PCAN_ERROR_QRCVEMPTY));
}

TPCANStatus TimerRead::ReadMessageFD()
{
	TPCANMsgFD CANMsg;
	TPCANTimestampFD CANTimeStamp;

	// We execute the "Read" function of the PCANBasic
	TPCANStatus stsResult = CAN_ReadFD(PcanHandle, &CANMsg, &CANTimeStamp);
	if (stsResult != PCAN_ERROR_QRCVEMPTY)
		// We process the received message
		ProcessMessageCanFD(CANMsg, CANTimeStamp);

	return stsResult;
}

TPCANStatus TimerRead::ReadMessage()
{
	TPCANMsg CANMsg;
	TPCANTimestamp CANTimeStamp;

	// We execute the "Read" function of the PCANBasic
	TPCANStatus stsResult = CAN_Read(PcanHandle, &CANMsg, &CANTimeStamp);
	if (stsResult != PCAN_ERROR_QRCVEMPTY)
		// We process the received message
		ProcessMessageCan(CANMsg, CANTimeStamp);

	return stsResult;
}

void TimerRead::ProcessMessageCan(TPCANMsg msg, TPCANTimestamp itsTimeStamp)
{
	UINT64 microsTimestamp = ((UINT64)itsTimeStamp.micros + 1000 * (UINT64)itsTimeStamp.millis + 0x100000000 * 1000 * itsTimeStamp.millis_overflow);

	// std::cout << "Type: " << GetMsgTypeString(msg.MSGTYPE) << "\n";
	// std::cout << "ID: " << GetIdString(msg.ID, msg.MSGTYPE) << "\n";
	char result[MAX_PATH] = { 0 };
	sprintf_s(result, sizeof(result), "%i", msg.LEN);
	// std::cout << "Length: " << result << "\n";
	// std::cout << "Time: " << GetTimeString(microsTimestamp) << "\n";
	// std::cout << "Data: " << GetDataString(msg.DATA, msg.MSGTYPE, msg.LEN) << "\n";
	// std::cout << "----------------------------------------------------------\n";
	Can_receive(msg.ID, msg.DATA);
}

void TimerRead::ProcessMessageCanFD(TPCANMsgFD msg, TPCANTimestampFD itsTimeStamp)
{
	std::cout << "Type: " << GetMsgTypeString(msg.MSGTYPE) << "\n";
	std::cout << "ID: " << GetIdString(msg.ID, msg.MSGTYPE) << "\n";
	std::cout << "Length: " << GetLengthFromDLC(msg.DLC) << "\n";
	std::cout << "Time: " << GetTimeString(itsTimeStamp) << "\n";
	std::cout << "Data: " << GetDataString(msg.DATA, msg.MSGTYPE, GetLengthFromDLC(msg.DLC)) << "\n";
	std::cout << "----------------------------------------------------------\n";
}

bool TimerRead::CheckForLibrary()
{
	// Check for dll file
	try
	{
		CAN_Uninitialize(PCAN_NONEBUS);
		return true;
	}
	catch (const std::exception&)
	{
		std::cout << ("Unable to find the library: PCANBasic::dll !\n");
		std::cout << ("Closing...\n");
		std::cout << "Press any key to continue...\n";
		_getch();
	}

	return false;
}

void TimerRead::ShowCurrentConfiguration()
{
	std::cout << "Parameter values used\n";
	std::cout << "----------------------\n";
	char buffer[MAX_PATH];
	FormatChannelName(PcanHandle, buffer, IsFD);
	std::cout << "* PCANHandle: " << buffer << "\n";
	if (IsFD)
		std::cout << "* IsFD: True\n";
	else
		std::cout << "* IsFD: False\n";
	ConvertBitrateToString(Bitrate, buffer);
	std::cout << "* Bitrate: " << buffer << "\n";
	// std::cout << "* BitrateFD: " << BitrateFD << "\n";
	std::cout << "* TimerInterval: " << TimerInterval << "\n";
	std::cout << "\n";
}

void TimerRead::ShowStatus(TPCANStatus status)
{
	std::cout << "=========================================================================================\n";
	char buffer[MAX_PATH];
	GetFormattedError(status, buffer);
	std::cout << buffer << "\n";
	std::cout << "=========================================================================================\n";
}

void TimerRead::FormatChannelName(TPCANHandle handle, LPSTR buffer, bool isFD)
{
	TPCANDevice devDevice;
	BYTE byChannel;

	// Gets the owner device and channel for a PCAN-Basic handle
	if (handle < 0x100)
	{
		devDevice = (TPCANDevice)(handle >> 4);
		byChannel = (BYTE)(handle & 0xF);
	}
	else
	{
		devDevice = (TPCANDevice)(handle >> 8);
		byChannel = (BYTE)(handle & 0xFF);
	}

	// Constructs the PCAN-Basic Channel name and return it
	char handleBuffer[MAX_PATH];
	GetTPCANHandleName(handle, handleBuffer);
	if (isFD)
		sprintf_s(buffer, MAX_PATH, "%s:FD %d (%Xh)", handleBuffer, byChannel, handle);
	else
		sprintf_s(buffer, MAX_PATH, "%s %d (%Xh)", handleBuffer, byChannel, handle);
}

void TimerRead::GetTPCANHandleName(TPCANHandle handle, LPSTR buffer)
{
	strcpy_s(buffer, MAX_PATH, "PCAN_NONE");
	switch (handle)
	{
	case PCAN_PCIBUS1:
	case PCAN_PCIBUS2:
	case PCAN_PCIBUS3:
	case PCAN_PCIBUS4:
	case PCAN_PCIBUS5:
	case PCAN_PCIBUS6:
	case PCAN_PCIBUS7:
	case PCAN_PCIBUS8:
	case PCAN_PCIBUS9:
	case PCAN_PCIBUS10:
	case PCAN_PCIBUS11:
	case PCAN_PCIBUS12:
	case PCAN_PCIBUS13:
	case PCAN_PCIBUS14:
	case PCAN_PCIBUS15:
	case PCAN_PCIBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_PCI");
		break;

	case PCAN_USBBUS1:
	case PCAN_USBBUS2:
	case PCAN_USBBUS3:
	case PCAN_USBBUS4:
	case PCAN_USBBUS5:
	case PCAN_USBBUS6:
	case PCAN_USBBUS7:
	case PCAN_USBBUS8:
	case PCAN_USBBUS9:
	case PCAN_USBBUS10:
	case PCAN_USBBUS11:
	case PCAN_USBBUS12:
	case PCAN_USBBUS13:
	case PCAN_USBBUS14:
	case PCAN_USBBUS15:
	case PCAN_USBBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_USB");
		break;

	case PCAN_LANBUS1:
	case PCAN_LANBUS2:
	case PCAN_LANBUS3:
	case PCAN_LANBUS4:
	case PCAN_LANBUS5:
	case PCAN_LANBUS6:
	case PCAN_LANBUS7:
	case PCAN_LANBUS8:
	case PCAN_LANBUS9:
	case PCAN_LANBUS10:
	case PCAN_LANBUS11:
	case PCAN_LANBUS12:
	case PCAN_LANBUS13:
	case PCAN_LANBUS14:
	case PCAN_LANBUS15:
	case PCAN_LANBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_LAN");
		break;

	default:
		strcpy_s(buffer, MAX_PATH, "UNKNOWN");
		break;
	}
}

void TimerRead::GetFormattedError(TPCANStatus error, LPSTR buffer)
{
	// Gets the text using the GetErrorText API function. If the function success, the translated error is returned.
	// If it fails, a text describing the current error is returned.
	if (CAN_GetErrorText(error, 0x09, buffer) != PCAN_ERROR_OK)
		sprintf_s(buffer, MAX_PATH, "An error occurred. Error-code's text (%Xh) couldn't be retrieved", error);
}

void TimerRead::ConvertBitrateToString(TPCANBaudrate bitrate, LPSTR buffer)
{
	switch (bitrate)
	{
	case PCAN_BAUD_1M:
		strcpy_s(buffer, MAX_PATH, "1 MBit/sec");
		break;
	case PCAN_BAUD_800K:
		strcpy_s(buffer, MAX_PATH, "800 kBit/sec");
		break;
	case PCAN_BAUD_500K:
		strcpy_s(buffer, MAX_PATH, "500 kBit/sec");
		break;
	case PCAN_BAUD_250K:
		strcpy_s(buffer, MAX_PATH, "250 kBit/sec");
		break;
	case PCAN_BAUD_125K:
		strcpy_s(buffer, MAX_PATH, "125 kBit/sec");
		break;
	case PCAN_BAUD_100K:
		strcpy_s(buffer, MAX_PATH, "100 kBit/sec");
		break;
	case PCAN_BAUD_95K:
		strcpy_s(buffer, MAX_PATH, "95,238 kBit/sec");
		break;
	case PCAN_BAUD_83K:
		strcpy_s(buffer, MAX_PATH, "83,333 kBit/sec");
		break;
	case PCAN_BAUD_50K:
		strcpy_s(buffer, MAX_PATH, "50 kBit/sec");
		break;
	case PCAN_BAUD_47K:
		strcpy_s(buffer, MAX_PATH, "47,619 kBit/sec");
		break;
	case PCAN_BAUD_33K:
		strcpy_s(buffer, MAX_PATH, "33,333 kBit/sec");
		break;
	case PCAN_BAUD_20K:
		strcpy_s(buffer, MAX_PATH, "20 kBit/sec");
		break;
	case PCAN_BAUD_10K:
		strcpy_s(buffer, MAX_PATH, "10 kBit/sec");
		break;
	case PCAN_BAUD_5K:
		strcpy_s(buffer, MAX_PATH, "5 kBit/sec");
		break;
	default:
		strcpy_s(buffer, MAX_PATH, "Unknown Bitrate");
		break;
	}
}

std::string TimerRead::GetMsgTypeString(TPCANMessageType msgType)
{
	if ((msgType & PCAN_MESSAGE_STATUS) == PCAN_MESSAGE_STATUS)
		return "STATUS";

	if ((msgType & PCAN_MESSAGE_ERRFRAME) == PCAN_MESSAGE_ERRFRAME)
		return "ERROR";

	std::string strTemp;
	if ((msgType & PCAN_MESSAGE_EXTENDED) == PCAN_MESSAGE_EXTENDED)
		strTemp = "EXT";
	else
		strTemp = "STD";

	if ((msgType & PCAN_MESSAGE_RTR) == PCAN_MESSAGE_RTR)
		strTemp = (strTemp + "/RTR");
	else
		if (msgType > PCAN_MESSAGE_EXTENDED)
		{
			strTemp = (strTemp + " [ ");
			if (msgType & PCAN_MESSAGE_FD)
				strTemp = (strTemp + " FD");
			if (msgType & PCAN_MESSAGE_BRS)
				strTemp = (strTemp + " BRS");
			if (msgType & PCAN_MESSAGE_ESI)
				strTemp = (strTemp + " ESI");
			strTemp = (strTemp + " ]");
		}

	return strTemp;
}

std::string TimerRead::GetIdString(UINT32 id, TPCANMessageType msgType)
{
	char result[MAX_PATH] = { 0 };
	if ((msgType & PCAN_MESSAGE_EXTENDED) == PCAN_MESSAGE_EXTENDED)
	{
		sprintf_s(result, sizeof(result), "%08Xh", id);
		return result;
	}
	sprintf_s(result, sizeof(result), "%03Xh", id);
	return result;
}

int TimerRead::GetLengthFromDLC(BYTE dlc)
{
	switch (dlc)
	{
	case 9: return 12;
	case 10: return 16;
	case 11: return 20;
	case 12: return 24;
	case 13: return 32;
	case 14: return 48;
	case 15: return 64;
	default: return dlc;
	}
}

std::string TimerRead::GetTimeString(TPCANTimestampFD time)
{
	char result[MAX_PATH] = { 0 };
	double fTime = (time / 1000.0);
	sprintf_s(result, sizeof(result), "%.1f", fTime);
	return result;
}

//====================================================//

TimerWrite::TimerWrite()
{
	ShowCurrentConfiguration(); // Shows the current parameters configuration

	// Checks if PCANBasic.dll is available, if not, the program terminates
	m_DLLFound = CheckForLibrary();
	if (!m_DLLFound)
		return;

	TPCANStatus stsResult;
	// Initialization of the selected channel
	if (IsFD)
		stsResult = CAN_InitializeFD(PcanHandle, BitrateFD);
	else
		stsResult = CAN_Initialize(PcanHandle, Bitrate);

	if (stsResult != PCAN_ERROR_OK)
	{
		std::cout << "Can not initialize. Please check the defines in the code.\n";
		ShowStatus(stsResult);
		return;
	}

	// Writing messages...
	std::cout << "Successfully initialized.\n";
	m_TimerOn = true;
	std::cout << "Started writing messages...\n";
}

TimerWrite::~TimerWrite()
{
	m_TimerOn = false;
	if (m_DLLFound)
		CAN_Uninitialize(PCAN_NONEBUS);
}

void TimerWrite::TimerThread()
{
	while (m_TimerOn)
	{
		usleep(TimerInterval*1000);
		WriteMessages();
	}
}

void TimerWrite::WriteMessages()
{
	TPCANStatus stsResult;

	if (IsFD)
		stsResult = WriteMessageFD();
	else
		stsResult = WriteMessage();

	// Checks if the message was sent
	if (stsResult != PCAN_ERROR_OK)
		ShowStatus(stsResult);
	else
		std::cout << "\nMessage was successfully SENT";
}

TPCANStatus TimerWrite::WriteMessage()
{
	// Sends a CAN message with extended ID, and 8 data bytes
	return CAN_Write(PcanHandle, &send_msg);
}

TPCANStatus TimerWrite::WriteMessageFD()
{
	// Sends a CAN-FD message with standard ID, 64 data bytes, and bitrate switch
	TPCANMsgFD msgCanMessageFD;
	msgCanMessageFD.ID = 0x100;
	msgCanMessageFD.DLC = 15;
	msgCanMessageFD.MSGTYPE = PCAN_MESSAGE_FD | PCAN_MESSAGE_BRS;
	for (BYTE i = 0; i < 64; i++)
	{
		msgCanMessageFD.DATA[i] = i;
	}
	return CAN_WriteFD(PcanHandle, &msgCanMessageFD);
}

bool TimerWrite::CheckForLibrary()
{
	// Check for dll file
	try
	{
		CAN_Uninitialize(PCAN_NONEBUS);
		return true;
	}
	catch (const std::exception&)
	{
		std::cout << ("Unable to find the library: PCANBasic::dll !\n");
		std::cout << ("Closing...\n");
		std::cout << "Press any key to continue...\n";
		_getch();
	}

	return false;
}

void TimerWrite::ShowCurrentConfiguration()
{
	std::cout << "Parameter values used\n";
	std::cout << "----------------------\n";
	char buffer[MAX_PATH];
	FormatChannelName(PcanHandle, buffer, IsFD);
	std::cout << "* PCANHandle: " << buffer << "\n";
	if (IsFD)
		std::cout << "* IsFD: True\n";
	else
		std::cout << "* IsFD: False\n";
	ConvertBitrateToString(Bitrate, buffer);
	std::cout << "* Bitrate: " << buffer << "\n";
	// std::cout << "* BitrateFD: " << BitrateFD << "\n";
	std::cout << "* TimerInterval: " << TimerInterval << "\n";
	std::cout << "\n";
}

void TimerWrite::ShowStatus(TPCANStatus status)
{
	std::cout << "=========================================================================================\n";
	char buffer[MAX_PATH];
	GetFormattedError(status, buffer);
	std::cout << buffer << "\n";
	std::cout << "=========================================================================================\n";
}

void TimerWrite::FormatChannelName(TPCANHandle handle, LPSTR buffer, bool isFD)
{
	TPCANDevice devDevice;
	BYTE byChannel;

	// Gets the owner device and channel for a PCAN-Basic handle
	if (handle < 0x100)
	{
		devDevice = (TPCANDevice)(handle >> 4);
		byChannel = (BYTE)(handle & 0xF);
	}
	else
	{
		devDevice = (TPCANDevice)(handle >> 8);
		byChannel = (BYTE)(handle & 0xFF);
	}

	// Constructs the PCAN-Basic Channel name and return it
	char handleBuffer[MAX_PATH];
	GetTPCANHandleName(handle, handleBuffer);
	if (isFD)
		sprintf_s(buffer, MAX_PATH, "%s:FD %d (%Xh)", handleBuffer, byChannel, handle);
	else
		sprintf_s(buffer, MAX_PATH, "%s %d (%Xh)", handleBuffer, byChannel, handle);
}

void TimerWrite::GetTPCANHandleName(TPCANHandle handle, LPSTR buffer)
{
	strcpy_s(buffer, MAX_PATH, "PCAN_NONE");
	switch (handle)
	{
	case PCAN_PCIBUS1:
	case PCAN_PCIBUS2:
	case PCAN_PCIBUS3:
	case PCAN_PCIBUS4:
	case PCAN_PCIBUS5:
	case PCAN_PCIBUS6:
	case PCAN_PCIBUS7:
	case PCAN_PCIBUS8:
	case PCAN_PCIBUS9:
	case PCAN_PCIBUS10:
	case PCAN_PCIBUS11:
	case PCAN_PCIBUS12:
	case PCAN_PCIBUS13:
	case PCAN_PCIBUS14:
	case PCAN_PCIBUS15:
	case PCAN_PCIBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_PCI");
		break;

	case PCAN_USBBUS1:
	case PCAN_USBBUS2:
	case PCAN_USBBUS3:
	case PCAN_USBBUS4:
	case PCAN_USBBUS5:
	case PCAN_USBBUS6:
	case PCAN_USBBUS7:
	case PCAN_USBBUS8:
	case PCAN_USBBUS9:
	case PCAN_USBBUS10:
	case PCAN_USBBUS11:
	case PCAN_USBBUS12:
	case PCAN_USBBUS13:
	case PCAN_USBBUS14:
	case PCAN_USBBUS15:
	case PCAN_USBBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_USB");
		break;

	case PCAN_LANBUS1:
	case PCAN_LANBUS2:
	case PCAN_LANBUS3:
	case PCAN_LANBUS4:
	case PCAN_LANBUS5:
	case PCAN_LANBUS6:
	case PCAN_LANBUS7:
	case PCAN_LANBUS8:
	case PCAN_LANBUS9:
	case PCAN_LANBUS10:
	case PCAN_LANBUS11:
	case PCAN_LANBUS12:
	case PCAN_LANBUS13:
	case PCAN_LANBUS14:
	case PCAN_LANBUS15:
	case PCAN_LANBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_LAN");
		break;

	default:
		strcpy_s(buffer, MAX_PATH, "UNKNOWN");
		break;
	}
}

void TimerWrite::GetFormattedError(TPCANStatus error, LPSTR buffer)
{
	// Gets the text using the GetErrorText API function. If the function success, the translated error is returned.
	// If it fails, a text describing the current error is returned.
	if (CAN_GetErrorText(error, 0x09, buffer) != PCAN_ERROR_OK)
		sprintf_s(buffer, MAX_PATH, "An error occurred. Error-code's text (%Xh) couldn't be retrieved", error);
}

void TimerWrite::ConvertBitrateToString(TPCANBaudrate bitrate, LPSTR buffer)
{
	switch (bitrate)
	{
	case PCAN_BAUD_1M:
		strcpy_s(buffer, MAX_PATH, "1 MBit/sec");
		break;
	case PCAN_BAUD_800K:
		strcpy_s(buffer, MAX_PATH, "800 kBit/sec");
		break;
	case PCAN_BAUD_500K:
		strcpy_s(buffer, MAX_PATH, "500 kBit/sec");
		break;
	case PCAN_BAUD_250K:
		strcpy_s(buffer, MAX_PATH, "250 kBit/sec");
		break;
	case PCAN_BAUD_125K:
		strcpy_s(buffer, MAX_PATH, "125 kBit/sec");
		break;
	case PCAN_BAUD_100K:
		strcpy_s(buffer, MAX_PATH, "100 kBit/sec");
		break;
	case PCAN_BAUD_95K:
		strcpy_s(buffer, MAX_PATH, "95,238 kBit/sec");
		break;
	case PCAN_BAUD_83K:
		strcpy_s(buffer, MAX_PATH, "83,333 kBit/sec");
		break;
	case PCAN_BAUD_50K:
		strcpy_s(buffer, MAX_PATH, "50 kBit/sec");
		break;
	case PCAN_BAUD_47K:
		strcpy_s(buffer, MAX_PATH, "47,619 kBit/sec");
		break;
	case PCAN_BAUD_33K:
		strcpy_s(buffer, MAX_PATH, "33,333 kBit/sec");
		break;
	case PCAN_BAUD_20K:
		strcpy_s(buffer, MAX_PATH, "20 kBit/sec");
		break;
	case PCAN_BAUD_10K:
		strcpy_s(buffer, MAX_PATH, "10 kBit/sec");
		break;
	case PCAN_BAUD_5K:
		strcpy_s(buffer, MAX_PATH, "5 kBit/sec");
		break;
	default:
		strcpy_s(buffer, MAX_PATH, "Unknown Bitrate");
		break;
	}
}

//====================================================//

void callback_send(const rm_msgs::Can_send::ConstPtr& Imsg)
{
	std::cout<<"pitch/yaw angle : "<<Imsg->pitch_angle<<" | "<<Imsg->yaw_angle<<"\n";
	floatTobytes(Imsg->pitch_angle, send_msg.DATA);
	floatTobytes(Imsg->yaw_angle, send_msg.DATA + 4);
	return ;
}

ros::Publisher pub;
rm_msgs::Can_receive Omsg_Can;
void receive_data()
{
	ros::Rate loop_rate(100); // 发布频率为1Hz
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<rm_msgs::Can_receive>("Can_receive", 10);
    while (nh.ok()){
		// std::cout<<quat[0]<<","<<quat[1]<<","<<quat[2]<<","<<quat[3]<<"\n"
			// <<gyro[0]<<","<<gyro[1]<<","<<gyro[2]<<"\n"
			// <<acc[0]<<","<<acc[1]<<","<<acc[2]<<"\n"
			// <<bullet_speed<<","<<mode<<"\n"
			// <<"===============\n";
        Omsg_Can.quat_0 = quat[0];
        Omsg_Can.quat_1 = quat[1];
        Omsg_Can.quat_2 = quat[2];
        Omsg_Can.quat_3 = quat[3];
        Omsg_Can.gyro_0 = gyro[0];
        Omsg_Can.gyro_1 = gyro[1];
        Omsg_Can.gyro_2 = gyro[2];
        Omsg_Can.acc_0 = acc[0];
        Omsg_Can.acc_1 = acc[1];
        Omsg_Can.acc_2 = acc[2];
        Omsg_Can.bullet_speed = bullet_speed;
        Omsg_Can.mode = mode;
        // pub.publish(Omsg_Can);
        ros::spinOnce();
        loop_rate.sleep();
    }
	return ;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    
	send_msg.ID = 0x600;
	send_msg.LEN = (BYTE)8;
	send_msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
	for (BYTE i = 0; i < 8; i++){
		send_msg.DATA[i] = 0;
	}

	TimerRead reader;
	reader.m_hTimer = std::thread([&reader]() { reader.TimerThread(); });

	TimerWrite writer;
	writer.m_hTimer = std::thread([&writer]() { writer.TimerThread(); });

	ros::init(argc, argv, "Can"); //初始化ROS节点
    ros::NodeHandle nh;
	std::thread pub_thread(receive_data);
    ros::Subscriber sub_send = nh.subscribe("Can_send", 100, callback_send);
	ros::spin();
	
	reader.m_hTimer.join();
	writer.m_hTimer.join();
	pub_thread.join();
    return 0;
}