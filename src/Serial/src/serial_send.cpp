#include <serial_send/serial_send.h>

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
{
	unsigned char ucIndex;

	while (dwLength--)
	{
		ucIndex = ucCRC8 ^ (*pchMessage++);
		ucCRC8 = CRC8_TAB[ucIndex];
	}

	return (ucCRC8);
}
/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucCRC = 0;

	if ((pchMessage == 0) || (dwLength <= 2)) return;

	ucCRC = Get_CRC8_Check_Sum((unsigned char *)pchMessage, dwLength - 1, CRC8_INIT);
	pchMessage[dwLength - 1] = ucCRC;
}
/**
 *@brief   转换数据并发送
 *@param   data   类型  VisionData(union)  包含pitch,yaw,distance
 *@param   flag   类型  char   用于判断是否瞄准目标，0代表没有，1代表已经瞄准
 */
void TransformData(const VisionData &data)
{
  unsigned char Tdata[30];
  Tdata[0] = 0xA5;

  Tdata[1] = CmdID1;
	Append_CRC8_Check_Sum(Tdata, 3);

  Tdata[3] = data.pitch_angle.c[0];
  Tdata[4] = data.pitch_angle.c[1];
  Tdata[5] = data.pitch_angle.c[2];
  Tdata[6] = data.pitch_angle.c[3];

  Tdata[7] = data.yaw_angle.c[0];
  Tdata[8] = data.yaw_angle.c[1];
  Tdata[9] = data.yaw_angle.c[2];
  Tdata[10] = data.yaw_angle.c[3];

  Tdata[11] = data.dis.c[0];
  Tdata[12] = data.dis.c[1];
  Tdata[13] = data.dis.c[2];
  Tdata[14] = data.dis.c[3];

  Tdata[15] = data.isSwitched;
	Tdata[16] = data.isFindTarget;

  Tdata[17] = data.isSpinning;
  Tdata[18] = data.ismiddle;
  Tdata[19] = 0x00;

	Append_CRC16_Check_Sum(Tdata, 22);

}

int main(int argc, char** argv)
{
  setlocale(LC_ALL,"");
  ros::init(argc, argv, "serial_send"); // ROS初始化 设置节点名称
  c_board number4; // 实例化一个对象

  ros::Rate loop_rate(100); // 发布频率为100Hz
  while (ros::ok())
  {
    number4.TransformData();
    
    ros::spinOnce(); 
    loop_rate.sleep();
  }
  return 0;  
}