# dart串口协议

## 电控发视觉

32位包
帧头 0xFF
uint8 mode             模式      1位 
float32 current_value  3508电流  4位      
int16 turns            2006圈数  2位
float32 pitch          当前pitch 4位
float32 yaw            当前yaw   4位
中间空15位0
帧尾 0x0D

## 视觉发电控

帧头 0xFF
float32 pitch          目标pitch 4位          
float32 yaw            目标yaw   4位
uint8 fire_advice      火控      1位
中间空5位0
帧尾 0x0D

