#include <ros/ros.h>
#include <ros/console.h>
#include <rm_msgs/Can_send.h>
#include <rm_msgs/Can_receive.h>

#include "PCANBasic.h"
#include <iostream>
#define UINT32 uint32_t
#include <limits.h>
#define MAX_PATH PATH_MAX
#define sprintf_s snprintf
#include <cstring>
#define strcpy_s(destination, destination_size, source) strcpy(destination, source)

#include <termios.h>
#include <unistd.h>

#include <thread>

static int _getch() {
	struct termios config;
	tcgetattr(STDIN_FILENO, &config);
	struct termios saved_config = config;
	config.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &config);
	int res = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &saved_config);
	return res;
}

class TimerRead
{
private:
	/// <summary>
	/// Sets the PCANHandle (Hardware Channel)
	/// </summary>
	const TPCANHandle PcanHandle = PCAN_USBBUS1;
	/// <summary>
	/// Sets the desired connection mode (CAN = false / CAN-FD = true)
	/// </summary>
	const bool IsFD = false;
	/// <summary>
	/// Sets the bitrate for normal CAN devices
	/// </summary>
	const TPCANBaudrate Bitrate = PCAN_BAUD_1M;
	/// <summary>
	/// Sets the bitrate for CAN FD devices.
	/// Example - Bitrate Nom: 1Mbit/s Data: 2Mbit/s:
	///   "f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1"
	/// </summary>
	TPCANBitrateFD BitrateFD = const_cast<LPSTR>("f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1");
	/// <summary>
	/// Timerinterval (ms) for reading
	/// </summary>
	const int TimerInterval = 250;
	/// <summary>
	/// Shows if DLL was found
	/// </summary>
	bool m_DLLFound;
	/// <summary>
	/// Shows if thread run
	/// </summary>
	bool m_TimerOn;

public:
	/// <summary>
	/// Used for reading
	/// </summary>
	std::thread m_hTimer;
	// TimerRead constructor
	//
	TimerRead();

	// TimerRead destructor
	//
	~TimerRead();

	/// <summary>
	/// Thread function for reading messages
	/// </summary>
	void TimerThread();

private:
	/// <summary>
	/// Function for reading PCAN-Basic messages
	/// </summary>
	void ReadMessages();

	/// <summary>
	/// Function for reading messages on CAN-FD devices
	/// </summary>
	/// <returns>A TPCANStatus error code</returns>
	TPCANStatus ReadMessageFD();

	/// <summary>
	/// Function for reading CAN messages on normal CAN devices
	/// </summary>
	/// <returns>A TPCANStatus error code</returns>
	TPCANStatus ReadMessage();

	/// <summary>
	/// Processes a received CAN message
	/// </summary>
	/// <param name="msg">The received PCAN-Basic CAN message</param>
	/// <param name="itsTimeStamp">Timestamp of the message as TPCANTimestamp structure</param>
	void ProcessMessageCan(TPCANMsg msg, TPCANTimestamp itsTimeStamp);

	/// <summary>
	/// Processes a received CAN-FD message
	/// </summary>
	/// <param name="msg">The received PCAN-Basic CAN-FD message</param>
	/// <param name="itsTimeStamp">Timestamp of the message as microseconds (ulong)</param>
	void ProcessMessageCanFD(TPCANMsgFD msg, TPCANTimestampFD itsTimeStamp);

	/// <summary>
	/// Checks for availability of the PCANBasic labrary
	/// </summary>
	/// <returns>If the library was found or not</returns>
	bool CheckForLibrary();

	/// <summary>
	/// Shows/prints the configurable parameters for this sample and information about them
	/// </summary>
	void ShowConfigurationHelp();

	/// <summary>
	/// Shows/prints the configured paramters
	/// </summary>
	void ShowCurrentConfiguration();

	/// <summary>
	/// Shows formatted status
	/// </summary>
	/// <param name="status">Will be formatted</param>
	void ShowStatus(TPCANStatus status);

	/// <summary>
	/// Gets the formatted text for a PCAN-Basic channel handle
	/// </summary>
	/// <param name="handle">PCAN-Basic Handle to format</param>
	/// <parma name="buffer">A string buffer for the channel name</param>
	/// <param name="isFD">If the channel is FD capable</param>
	void FormatChannelName(TPCANHandle handle, LPSTR buffer, bool isFD);

	/// <summary>
	/// Gets name of a TPCANHandle
	/// </summary>
	/// <param name="handle">TPCANHandle to get name</param>
	/// <param name="buffer">A string buffer for the name of the TPCANHandle (size MAX_PATH)</param>
	void GetTPCANHandleName(TPCANHandle handle, LPSTR buffer);

	/// <summary>
	/// Help Function used to get an error as text
	/// </summary>
	/// <param name="error">Error code to be translated</param>
	/// <param name="buffer">A string buffer for the translated error (size MAX_PATH)</param>
	void GetFormattedError(TPCANStatus error, LPSTR buffer);

	/// <summary>
	/// Convert bitrate c_short value to readable string
	/// </summary>
	/// <param name="bitrate">Bitrate to be converted</param>
	/// <param name="buffer">A string buffer for the converted bitrate (size MAX_PATH)</param>
	void ConvertBitrateToString(TPCANBaudrate bitrate, LPSTR buffer);

	/// <summary>
	/// Gets the string representation of the type of a CAN message
	/// </summary>
	/// <param name="msgType">Type of a CAN message</param>
	/// <returns>The type of the CAN message as string</returns>
	std::string GetMsgTypeString(TPCANMessageType msgType);

	/// <summary>
	/// Gets the string representation of the ID of a CAN message
	/// </summary>
	/// <param name="id">Id to be parsed</param>
	/// <param name="msgType">Type flags of the message the Id belong</param>
	/// <returns>Hexadecimal representation of the ID of a CAN message</returns>
	std::string GetIdString(UINT32 id, TPCANMessageType msgType);

	/// <summary>
	/// Gets the data length of a CAN message
	/// </summary>
	/// <param name="dlc">Data length code of a CAN message</param>
	/// <returns>Data length as integer represented by the given DLC code</returns>
	int GetLengthFromDLC(BYTE dlc);

	/// <summary>
	/// Gets the string representation of the timestamp of a CAN message, in milliseconds
	/// </summary>
	/// <param name="time">Timestamp in microseconds</param>
	/// <returns>String representing the timestamp in milliseconds</returns>
	std::string GetTimeString(TPCANTimestampFD time);
};

class TimerWrite
{
private:
	/// <summary>
	/// Sets the PCANHandle (Hardware Channel)
	/// </summary>
	const TPCANHandle PcanHandle = PCAN_USBBUS1;
	/// <summary>
	/// Sets the desired connection mode (CAN = false / CAN-FD = true)
	/// </summary>
	const bool IsFD = false;
	/// <summary>
	/// Sets the bitrate for normal CAN devices
	/// </summary>
	const TPCANBaudrate Bitrate = PCAN_BAUD_1M;
	/// <summary>
	/// Sets the bitrate for CAN FD devices.
	/// Example - Bitrate Nom: 1Mbit/s Data: 2Mbit/s:
	///   "f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1"
	/// </summary>
	TPCANBitrateFD BitrateFD = const_cast<LPSTR>("f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1");
	/// <summary>
	/// Timerinterval (ms) for writing
	/// </summary>
	const int TimerInterval = 500;
	/// <summary>
	/// Shows if DLL was found
	/// </summary>
	bool m_DLLFound;
	/// <summary>
	/// Shows if thread run
	/// </summary>
	bool m_TimerOn;

public:
	/// <summary>
	/// Thread function for reading messages
	/// </summary>
	void TimerThread();
	/// <summary>
	/// Used for writing
	/// </summary>
	std::thread m_hTimer;
	// TimerWrite constructor
	//
	TimerWrite();

	// TimerWrite destructor
	//
	~TimerWrite();

private:
	/// <summary>
	/// Function for writing PCAN-Basic messages
	/// </summary>
	void WriteMessages();

	/// <summary>
	/// Function for writing messages on CAN devices
	/// </summary>
	/// <returns>A TPCANStatus error code</returns>
	TPCANStatus WriteMessage();

	/// <summary>
	/// Function for writing messages on CAN-FD devices
	/// </summary>
	/// <returns>A TPCANStatus error code</returns>
	TPCANStatus WriteMessageFD();

	/// <summary>
	/// Checks for availability of the PCANBasic labrary
	/// </summary>
	/// <returns>If the library was found or not</returns>
	bool CheckForLibrary();

	/// <summary>
	/// Shows/prints the configurable parameters for this sample and information about them
	/// </summary>
	void ShowConfigurationHelp();

	/// <summary>
	/// Shows/prints the configured paramters
	/// </summary>
	void ShowCurrentConfiguration();

	/// <summary>
	/// Shows formatted status
	/// </summary>
	/// <param name="status">Will be formatted</param>
	void ShowStatus(TPCANStatus status);

	/// <summary>
	/// Gets the formatted text for a PCAN-Basic channel handle
	/// </summary>
	/// <param name="handle">PCAN-Basic Handle to format</param>
	/// <parma name="buffer">A string buffer for the channel name</param>
	/// <param name="isFD">If the channel is FD capable</param>
	void FormatChannelName(TPCANHandle handle, LPSTR buffer, bool isFD);

	/// <summary>
	/// Gets name of a TPCANHandle
	/// </summary>
	/// <param name="handle">TPCANHandle to get name</param>
	/// <param name="buffer">A string buffer for the name of the TPCANHandle (size MAX_PATH)</param>
	void GetTPCANHandleName(TPCANHandle handle, LPSTR buffer);

	/// <summary>
	/// Help Function used to get an error as text
	/// </summary>
	/// <param name="error">Error code to be translated</param>
	/// <param name="buffer">A string buffer for the translated error (size MAX_PATH)</param>
	void GetFormattedError(TPCANStatus error, LPSTR buffer);

	/// <summary>
	/// Convert bitrate c_short value to readable string
	/// </summary>
	/// <param name="bitrate">Bitrate to be converted</param>
	/// <param name="buffer">A string buffer for the converted bitrate (size MAX_PATH)</param>
	void ConvertBitrateToString(TPCANBaudrate bitrate, LPSTR buffer);
};

