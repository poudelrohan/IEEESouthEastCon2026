/**
 * @file WonderLLM_porting.cpp
 * @brief WonderLLM模块驱动(硬件层应用)
 * @author ZhiYuan (Gilbert@hiwonder.com)
 */

#include "WonderLLM_porting.h"
#include "hiwonder_i2c.h"

const char tool_finish[] = 
	"{\"command\":\"mcp_setting\",\"params\":\"true\"}";	

const char tool_buzzer[] = 
	"{\"tool_name\":\"self.robot.set_buzzer\",\"command\":\"控制机器人的蜂鸣器时调用这个工具。'count'是蜂鸣器响的次数\","
	"\"params\":[[\"count\",\"int\"]],\"block\":\"true\",\"return\":\"false\"}";

const char tool_led[] = 
		"{\"tool_name\":\"self.robot.set_led_color\","
		"\"command\":\"设置左右RGB灯颜色。lr,lg,lb是左灯RGB, rr,rg,rb是右灯RGB, 范围0-255。\","
		"\"params\":[[\"lr\",\"int\",0,255],[\"lg\",\"int\",0,255],[\"lb\",\"int\",0,255],"
								"[\"rr\",\"int\",0,255],[\"rg\",\"int\",0,255],[\"rb\",\"int\",0,255]],"
		"\"block\":\"true\",\"return\":\"false\"}";

// const char tool_stop[] = 
// 	"{\"tool_name\":\"self.robot.stop\",\"command\":\"控制机器人恢复初始姿态时调用这个工具。"
// 	"\"params\":[[\"stop\",\"true\"]],\"block\":\"true\",\"return\":\"false\"}";

const char tool_SpeedMode[] = 
	"{\"tool_name\":\"self.robot.set_SpeedMode\",\"command\":\"切换机器人的速度模式时调用这个工具。"
	"可切换的速度模式从1到2档包括：'Low_speed','High_speed'。\",\"params\":[[\"speed_mode\",\"string\"]],\"block\":\"true\",\"return\":\"false\"}";

const char tool_RunningMode[] = 
	"{\"tool_name\":\"self.robot.set_RunningMode\",\"command\":\"切换机器人的运动模式时调用这个工具。distance为距离,默认为200,单位毫米,"
	"可切换的运动模式包括：'normal','intelligent_avoidance', 'intelligent_track'。\",\"params\":[[\"running_mode\",\"string\"],[\"distance\",\"int\"]],\"block\":\"true\",\"return\":\"false\"}";

const char tool_ActionGroup[] = 
	"{\"tool_name\":\"self.robot.ActionGroup\",\"command\":\"控制机器人执行动作组时调用这个工具。actionNum为动作组代号,"
	"3号为唤醒,4号为唤醒奔跑,5号为撒娇,6号为越障,7号为左腿战斗,8号为右腿战斗,9号为左脚向前踢球,"
	"10号为左脚向右踢球,11号为右脚向前踢球,12号为右脚向左踢球,13号为推门,14号为挥手,executeNum为动作组运行次数\","
	"\"params\":[[\"actionNum\",\"int\",3,14],[\"executeNum\",\"int\"]],\"block\":\"true\",\"return\":\"false\"}";

const char tool_status[] ="{\"tool_name\":\"self.robot.get_status\",\"command\":\"获取机器人的实时状态时调用这个工具。"
			"可查询的状态包括：'battery', 'distance', 'running_mode','Speed_Mode','poseture','Bodystate'(2为翻倒,1为趴下,0为站立)。\",\"params\":[[\"status_name\",\"string\"]],\"block\":\"true\",\"return\":\"true\"}";

const char tool_move[] = 
	"{\"tool_name\":\"self.robot.move\","
	// "\"command\":\"控制机器人平移时调用该工具。move表示运动方向,分别为正前、右前、正右、右后、正后、左后、正左、左前, 原地左转, 原地右转,分别使用代号1至10。step_num参数控制运动步数默认为-1。\","
	// "\"params\":[[\"move\",\"int\",1,10],[\"step_num\",\"int\"]],\"block\":\"true\",\"return\":\"false\"}";
	// "\"command\":\"控制机器人平移时调用这个工具。move参数控制运动方向,分别为正前、右前、正右、右后、正后、左后、正左、左前, 原地左转, 原地右转,分别使用代号1至10。step_num参数控制运动步数,该项默认为1,如不指定运动时长为-1。duration是运动时长,,如持续运动默认为1000,单位为毫秒(ms)\","
	// "\"command\":\"控制机器人平移时调用该工具。move参数控制运动方向,分别为正前、右前、正右、右后、正后、左后、正左、左前, 原地左转, 原地右转,分别使用代号1至10。step_num参数控制运动步数,如不指定默认为1。duration是运动时长,如不指定默认1000,单位为毫秒(ms)\","
	// "\"command\":\"控制机器人平移时调用该工具。move表示运动方向,分别为正前、右前、正右、右后、正后、左后、正左、左前, 原地左转, 原地右转,分别使用代号1至10。step_num参数控制运动步数默认为1,如指令明确提及'一直运动'时为-1。duration是运动时长默认1000毫秒\","
	"\"command\":\"控制机器人平移时调用。move为运动方向,可为前、右前、右、右后、后、左后、左、左前, 左转, 右转,使用代号1至10。step_num是运动步数默认为-1。duration是运动时长默认-1毫秒。distance是运动距离默认-1毫米。angle参数是运动角度默认为-1度。\","
	"\"params\":[[\"move\",\"int\",1,10],[\"step_num\",\"int\"],[\"duration\",\"int\"],[\"distance\",\"int\"],[\"angle\",\"int\"]],\"block\":\"true\",\"return\":\"false\"}";

const char tool_BaryCenterMove[] = 
	"{\"tool_name\":\"self.robot.BaryCenterMove\","
	"\"command\":\"控制机器人重心移动时调用这个工具。pose参数控制运动方向,分别为正前、右前、正右、右后、正后、左后、正左、左前、正上、正下,分别使用代号1至10,如需恢复初始姿态使用11。\","
	"\"params\":[[\"pose\",\"int\",1,11]],\"block\":\"true\",\"return\":\"false\"}";

const char tool_InclinationAngleMove[] = 
	"{\"tool_name\":\"self.robot.InclinationAngleMove\","
	"\"command\":\"控制机器人姿态倾斜时调用这个工具。Inclination参数控制运动方向,分别为前倾、后倾、左倾、右倾、左扭、右扭,分别使用代号1至6。\","
	"\"params\":[[\"Inclination\",\"int\",1,6]],\"block\":\"true\",\"return\":\"false\"}";

const char vision_prompt[] = 
	"调用摄像头识别前方画面,如果画面出现人脸,只回复'true',否则只回复'false'";


/**
 * @brief 系统延时功能接口
 */
void delay_ms(int ms_num){
		delay(ms_num);
}
	
/**
 * @brief 系统实时时间获取接口
 */
uint32_t Get_time_now(){
		return millis();
}

/**
 * @brief IIC扫描接口
 * @note  1.执行非必要，仅是出于程序健壮性的考虑
 */
bool Detect_WonderLLM(){
  Wire.beginTransmission(WONDERLLM_SLAVE_ADDRESS);
	delay_ms(15); //等待一段时间，等待以上配置写入寄存器内部并生效
  return (Wire.endTransmission() == 0);
}

/**
 * @brief IIC速率配置接口
 * @note 涉及较长字符串（尤其是中文）的传输，必须使用调用本函数将IIC速率
 *       提升至400,000，否则WonderLLM将无法完整接收数据 
 */
void IIC_Config_MCP_Transmit(){
		Wire.setClock(400000);
}

/**
 * @brief IIC速率配置接口
 * @note 1.执行非必要，如果其他IIC设备均支持40W速率通信，则可不必切换回
 *         较低的100W，执行该函数是出于兼容其他低速IIC设备的考虑
 */
void IIC_Config_normal_Transmit(){
		// 恢复I2C速率
		Wire.setClock(100000);
}

/**
 * @brief I2C底层数据接收接口v1.1
 * @note  适配新版通信协议，JSON字符串分包接收，每包数据加上校验位最大长度为32字节
 * @return 0: success.
 *         1: received NACK.
 *         2: timeout.
 *         3: data lack.
 */
int WonderLLM_Receive_Data(uint8_t* buffer, uint16_t size,bool stop_flag) {
		if(size <= 0) return 0;

    uint16_t index = 0;
		int receive_result = 0;
		uint32_t start_time = 0;

		receive_result = Wire.requestFrom(WONDERLLM_SLAVE_ADDRESS, size, stop_flag);
		if(receive_result == 0){ 
			Serial.println("debug test5");
				return 1;
		} else if(receive_result != size){
				return 3;
		}

		start_time = Get_time_now(); 
		while((index < size) && Wire.available()) {
				buffer[index] = Wire.read();    
				index ++;

				if(Get_time_now() - start_time > 10){
					return 2;
				}
		}

		return 0;
}

/**
 * @brief I2C底层数据发送接口
 * @note 1.Wire库的实现使用了一个32字节的缓冲区，因此任何通信都应在此限制范围内进行。在单次传输中超出字节数的将会被丢弃。
 *       2.基于1的原因，发送时每32byte发送一次，剩余部分再按实际长度发送
 * @return 0: success.
 *         1: data too long to fit in transmit buffer.
 *         2: received NACK on transmit of address.
 *         3: received NACK on transmit of data.
 *         4: other error.
 *         5: timeout
 */
int WonderLLM_Send_Data(uint8_t* buffer, uint16_t len) {
	if(wireWritemultiByte(WONDERLLM_SLAVE_ADDRESS, buffer, len)){
		return 0;
	}else{
		return 4;
	}
		// uint16_t index = 0;
		// int surplus_num = len % 32;
		// int integer_num = len / 32;
		// int transmit_result = 0;
		// Serial.println(integer_num);
		// if(len <= 0) return 0;

	  // for(int i =0; i < integer_num; i++){

		// 	//准备传输批量数据，每次32byte
		// 	Wire.beginTransmission(WONDERLLM_SLAVE_ADDRESS); 

		// 	for(int y =0; y < 32; y ++){
		// 		Wire.write(buffer[index]);             // 逐字节传入缓冲区
		// 		index ++;
		// 	}

	  //   transmit_result = Wire.endTransmission(false);
		// 	if(transmit_result != 0 ){
		// 		return transmit_result;      //发送数据但不产生停止位
		// 	} 

		// }

		// //准备传输剩余不足32byte的数据
		// Wire.beginTransmission(WONDERLLM_SLAVE_ADDRESS); 

		// for(int z =0; z < surplus_num; z ++){
		// 	Wire.write(buffer[index]);             // 逐字节传入缓冲区
		// 	index ++;
		// }

	  // transmit_result = Wire.endTransmission(true);
		// if(transmit_result != 0 ){
		// 	return transmit_result;      //发送数据并产生停止位
		// }

		// return transmit_result;
}

