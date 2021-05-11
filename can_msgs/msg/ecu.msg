Header header

float32 motor # 目标速度
float32 steer # 转向
bool brake # 紧急停车
float32 cur_speed # 当前速度

uint8 shift # 档位
uint8 SHIFT_UNKNOWN = 0 
uint8 SHIFT_D = 1 #前进档位
uint8 SHIFT_N = 2 #停止档位
uint8 SHIFT_R = 3 #后退档位
# uint8 SHIFT_T = 9 #遥控