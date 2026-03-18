# 弹道参数消息
std_msgs/Header header

# 基本参数
bool enable                 # 弹道解算开关
float64 gravity             # 重力加速度 (m/s²)
float64 drag_coeff          # 阻力系数
float64 muzzle_velocity     # 枪口初速度 (m/s)

# 距离参数
float64 sentinel_distance   # 到前哨站的水平距离 (m)
float64 base_distance       # 到基地的水平距离 (m)
float64 vertical_offset     # 竖直距离 (m)

# 当前使用的参数
uint8 mode                  # 当前模式 (1: 前哨站, 2: 基地)
float64 current_distance    # 当前使用的水平距离 (m)
