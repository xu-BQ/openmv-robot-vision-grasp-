# ==============================================================
#  桌面移动复合机器人 — OpenMV 视觉抓取主程序
#  功能：颜色识别 → 对准木块 → 自动抓取 → 寻找粉色放置区 → 投放归位
#  抓取顺序：先黄色木块 → 再橙色木块
#  通信：通过串口与 STM32 主控交互
# ==============================================================

from pyb import UART, Pin, Timer
import time, pyb
import math, sensor


# ==============================================================
#  固定抓取顺序：先黄后橙（如需修改，改这两个颜色代号即可）
#  Y=黄(Yellow)  O=橙(Orange)
# ==============================================================
TARGET_COLOR_1 = 'Y'   # 第一个抓取：黄色
TARGET_COLOR_2 = 'O'   # 第二个抓取：橙色


# ====================== 任务状态机 ======================
# task_stage：
#   0 = 空闲待命
#   1 = 等待抓取第一个目标颜色（黄）
#   3 = 等待抓取第二个目标颜色（橙）
task_stage = 0
task_lock = False  # 任务锁定标记，防止重复触发


# ====================== 硬件初始化 ======================
light = Pin('P0', Pin.OUT_PP)
light.low()
led = pyb.LED(3)

# 串口3：与STM32通信，波特率115200
uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)
uart.write("$KMS:0,100,95,1000!\n")

# 定时器4：PWM控制补光灯
tim = Timer(4, freq=1000)
led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
led_dac.pulse_width_percent(50)    # 初始亮度 50%

run_app_status = 0
uart.write("#openmv reset\n\n!")


# ====================== 机械臂抓取偏移量 ======================
# 若抓取偏斜，调整这三个变量（单位：mm）
#   cx：偏右减小，偏左增大
#   cy：偏前减小，偏后增大
#   cz：偏高减小，偏低增大
cx = 0
cy = 100
cz = 0


# ====================== 目标高度配置（核心参数）======================
# 当前场景：底盘 11.5cm | 桌面 27cm | 木块 3cm 立方体
#
# Alpha=-30° 时末端最高 z ≈ 230mm，摄像头 z ≈ 280mm（高于桌面 10mm）
# 搜索姿态：L1+L2 近竖直 ↑，L3 下倾 30° ↘，斜向下俯视 27cm 桌面
SEARCH_Y_START    = 135   # 搜索起始 y
TARGET_GRAB_Z     = 180   # 抓取高度
TARGET_APPROACH_Z = 180   # 接近高度
TARGET_LIFT_Z     = 220   # 抬起高度
SEARCH_Z          = 180   # 搜索高度
# ==================================================================


# ====================== 蜂鸣器提示 ======================
def beep():
    """短促鸣叫+LED闪烁，用于确认指令接收"""
    uart.write("$BEEP!\n")
    led.on()
    time.sleep_ms(100)
    led.off()
    time.sleep_ms(100)


# ==============================================================
#  逆运动学类 —— 六自由度机械臂关节角度解算
#  机械参数（单位：mm×10，内部放大10倍用整数运算）：
#    L0=115mm  底座离地高度
#    L1=105mm  大臂长度
#    L2=88mm   小臂长度
#    L3=155mm  末端（腕部+夹爪）长度
#
#  Alpha 搜索范围：+90°（过肩上举）→ -135°（极限下俯）
#  选取策略：所有有效解中选最负（最"低头"）的 Alpha
# ==============================================================
class Kinematics():
    L0 = 1150
    L1 = 1050
    L2 = 880
    L3 = 1550

    pi = 3.1415926
    time = 1500

    uart = UART(3, 115200)
    uart.init(115200, bits=8, parity=None, stop=1)

    def send_str(self, cmd: str):
        """直接发送字符串到舵机总线"""
        self.uart.write(cmd)

    def kinematics_analysis(self, x: float, y: float, z: float, Alpha: float):
        """
        逆运动学核心解算
        参数:
            x, y  — 基坐标系下平面坐标（mm）
            z     — 夹爪离地高度（mm）
            Alpha — 夹爪与水平面夹角（°），正=上仰，负=下俯
        返回:
            成功 → 舵机指令字符串
            失败 → 错误码（int）
        """
        x = x * 10
        y = y * 10
        z = z * 10

        l0 = float(self.L0)
        l1 = float(self.L1)
        l2 = float(self.L2)
        l3 = float(self.L3)

        # ---- 腰部旋转角 theta6 ----
        if x == 0:
            theta6 = 0.0
        else:
            theta6 = math.atan(x / y) * 270.0 / self.pi

        # ---- 夹爪→腕部坐标转换 ----
        y = math.sqrt(x * x + y * y)
        y = y - l3 * math.cos(Alpha * self.pi / 180.0)
        z = z - l0 - l3 * math.sin(Alpha * self.pi / 180.0)

        # ---- 腕部可达性检查 ----
        if z < -l0:
            return 1
        if math.sqrt(y * y + z * z) > (l1 + l2):
            return 2

        # ---- 大臂关节角 theta5 ----
        ccc = math.acos(y / math.sqrt(y * y + z * z))
        bbb = (y * y + z * z + l1 * l1 - l2 * l2) / (2 * l1 * math.sqrt(y * y + z * z))
        if bbb > 1 or bbb < -1:
            return 5

        zf_flag = -1 if z < 0 else 1
        theta5 = ccc * zf_flag + math.acos(bbb)
        theta5 = theta5 * 180.0 / self.pi
        if theta5 > 180.0 or theta5 < 0.0:
            return 6

        # ---- 小臂关节角 theta4 ----
        aaa = -(y * y + z * z - l1 * l1 - l2 * l2) / (2 * l1 * l2)
        if aaa > 1 or aaa < -1:
            return 3

        theta4 = math.acos(aaa)
        theta4 = 180.0 - theta4 * 180.0 / self.pi
        if theta4 > 135.0 or theta4 < -135.0:
            return 4

        # ---- 腕部关节角 theta3 ----
        theta3 = Alpha - theta5 + theta4
        if theta3 > 90.0 or theta3 < -90.0:
            return 7

        # ---- 角度→舵机脉宽（500~2500μs, 1500μs=0°）----
        servo_angle0 = theta6
        servo_angle1 = theta5 - 90
        servo_angle2 = theta4
        servo_angle3 = theta3

        servo_pwm0 = int(1500 - 2000.0 * servo_angle0 / 270.0)
        servo_pwm1 = int(1500 + 2000.0 * servo_angle1 / 270.0)
        servo_pwm2 = int(1500 + 2000.0 * servo_angle2 / 270.0)
        servo_pwm3 = int(1500 - 2000.0 * servo_angle3 / 270.0)

        servo_pwm3 = 3000 - servo_pwm3

        arm_str = (
            "{{#000P{0:0>4d}T{4:0>4d}!"
            "#001P{1:0>4d}T{4:0>4d}!"
            "#002P{2:0>4d}T{4:0>4d}!"
            "#003P{3:0>4d}T{4:0>4d}!}}"
        ).format(servo_pwm0, servo_pwm1, servo_pwm2, servo_pwm3, self.time)
        return arm_str

    def kinematics_move(self, x: float, y: float, z: float, time1: int) -> int:
        """
        机械臂移动到目标点（自动搜索最佳 Alpha）
        Alpha 搜索范围：+90° → -135°，选最负的有效解
        """
        self.time = time1

        if y < 0:
            return 0

        flag = 0
        cnt = 0
        for i in range(90, -136, -1):
            result = self.kinematics_analysis(x, y, z, i)
            if isinstance(result, str):
                if flag == 0 or i < cnt:
                    cnt = i
                    flag = 1

        if flag:
            arm_str = self.kinematics_analysis(x, y, z, cnt)
            return self.uart.write(arm_str)

        return 0


# ==============================================================
#  颜色分拣类 —— 视觉识别 + 抓取投放四阶段状态机
#
#  状态机流程：
#    init      → 归位到搜索姿态
#    status=0  → 对准阶段：微调 x,y 使木块居中
#    status=1  → 抓取阶段：接近→下移→夹紧→抬起→旋转
#    status=2  → 找粉色放置区：仅在真正识别到粉色时才计数对准
#    status=3  → 投放阶段：接近→下移→松开→抬起→归位
#
#  只识别两种颜色：黄色(Y)、橙色(O)
#  放置区识别：粉色色块（约 10cm×10cm）
# ==============================================================
class ColorSort():
    # ---- 颜色阈值（LAB色彩空间）----
    yellow_threshold = (57, 100, -33, 70, 48, 127)    # 黄色木块
    orange_threshold = (35, 80, 20, 50, 40, 70)       # 橙色木块
    pink_threshold   = (48, 71, 11, 32, -12, 13)     # 粉色放置区色块（约10cm×10cm）

    # 图像中心（QQVGA: 160×120）
    mid_block_cx = 80
    mid_block_cy = 60

    def init(self):
        """
        初始化摄像头，机械臂就位到搜索姿态
        """
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QQVGA)
        sensor.skip_frames(n=2000)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)

        self.kinematic = kinematic
        self.cap_color_status = 0          # 已抓颜色（0=未抓）
        self.move_x = 0                    # 当前 x
        self.move_y = SEARCH_Y_START       # 当前 y
        self.mid_block_cnt = 0             # 对准计数器
        self.move_status = 0               # 状态机阶段

        # 开补光灯 → 机械臂就位到搜索姿态
        self.led_dac = led_dac
        self.led_dac.pulse_width_percent(50)
        self.kinematic.kinematics_move(self.move_x, self.move_y, SEARCH_Z, 1000)
        time.sleep_ms(1000)

    def run(self, cx=0, cy=100, cz=20):
        """
        主循环：每帧执行一次
        """
        # ---- 变量初始化 ----
        block_cx = self.mid_block_cx
        block_cy = self.mid_block_cy
        color_read_succed = 0
        color_status = 0

        # ---- 第1步：采集图像 ----
        img = sensor.snapshot()

        # ---- 第2步：颜色检测（仅黄、橙、粉三种）----
        yellow_blobs = img.find_blobs([self.yellow_threshold],
                                      x_stride=5, y_stride=5,
                                      pixels_threshold=200, margin=20, merge=True)
        orange_blobs = img.find_blobs([self.orange_threshold],
                                      x_stride=5, y_stride=5,
                                      pixels_threshold=200, margin=20, merge=True)
        # 粉色放置区色块（约10cm×10cm，像素面积较大）
        pink_blobs   = img.find_blobs([self.pink_threshold],
                                      x_stride=5, y_stride=5,
                                      pixels_threshold=500, margin=20, merge=True)

        # ---- 第3步：粉色放置区坐标提取 ----
        pink_cx = self.mid_block_cx
        pink_cy = self.mid_block_cy
        pink_detected = bool(pink_blobs)   # ★ 粉色是否被真实检测到
        if pink_blobs:
            for b in pink_blobs:
                img.draw_rectangle(b[0], b[1], b[2], b[3], color=(255, 192, 203))
                img.draw_cross(b[5], b[6], size=2, color=(255, 192, 203))
                img.draw_string(b[0], (b[1] - 10), "PINK", color=(255, 192, 203))
                pink_cx = b[5]
                pink_cy = b[6]

        # ---- 第4步：任务颜色过滤 ----
        # 阶段1=只认黄，阶段3=只认橙，阶段0=全认（手动模式）
        global task_stage

        # ---- 第5步：黄色识别 ----
        if yellow_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'Y'):
            if task_stage == 0 or task_stage == 1:
                color_status = 'Y'
                color_read_succed = 1
                for y in yellow_blobs:
                    img.draw_rectangle(y[0], y[1], y[2], y[3], color=(255, 255, 0))
                    img.draw_cross(y[5], y[6], size=2, color=(255, 255, 0))
                    img.draw_string(y[0], (y[1] - 10), "YELLOW", color=(255, 255, 0))
                    block_cx = y[5]
                    block_cy = y[6]

        # ---- 第6步：橙色识别 ----
        if orange_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'O'):
            if task_stage == 0 or task_stage == 3:
                color_status = 'O'
                color_read_succed = 1
                for y in orange_blobs:
                    img.draw_rectangle(y[0], y[1], y[2], y[3], color=(255, 165, 0))
                    img.draw_cross(y[5], y[6], size=2, color=(255, 165, 0))
                    img.draw_string(y[0], (y[1] - 10), "ORANGE", color=(255, 165, 0))
                    block_cx = y[5]
                    block_cy = y[6]

        # ==========================================================
        #  四阶段运动状态机
        #  入口条件：识别到颜色 或 正在执行状态1/2/3
        # ==========================================================
        if color_read_succed == 1 or self.move_status >= 1:

            # ========== 第0阶段：对准木块 ==========
            if self.move_status == 0:
                if abs(block_cx - self.mid_block_cx) > 8:
                    if block_cx > self.mid_block_cx:
                        self.move_x += 1.0
                    else:
                        self.move_x -= 1.0
                if abs(block_cy - self.mid_block_cy) > 8:
                    if block_cy > self.mid_block_cy and self.move_y > 80:
                        self.move_y -= 1.2
                    else:
                        self.move_y += 1.2

                if abs(block_cy - self.mid_block_cy) <= 8 and \
                   abs(block_cx - self.mid_block_cx) <= 8:
                    self.mid_block_cnt += 1
                    print(f"对准计数：{self.mid_block_cnt}")
                    if self.mid_block_cnt > 10:
                        print("触发抓取！")
                        self.mid_block_cnt = 0
                        self.move_status = 1
                        self.cap_color_status = color_status
                else:
                    self.mid_block_cnt = 0
                    self.kinematic.kinematics_move(self.move_x, self.move_y,
                                                   SEARCH_Z, 50)
                    time.sleep_ms(50)

            # ========== 第1阶段：抓取木块 ==========
            elif self.move_status == 1:
                self.move_status = 2
                time.sleep_ms(100)

                # 打开夹爪
                self.kinematic.send_str("{#005P1000T1000!}")
                time.sleep_ms(500)

                # 计算抓取点（向外延伸）
                l = math.sqrt(self.move_x * self.move_x +
                              self.move_y * self.move_y)
                sin = self.move_y / l
                cos = self.move_x / l
                self.move_x = (l + 85 + cy) * cos + cx
                self.move_y = (l + 85 + cy) * sin
                time.sleep_ms(500)

                # ① 移动到木块正上方（接近高度）
                self.kinematic.kinematics_move(self.move_x, self.move_y,
                                               TARGET_APPROACH_Z, 1000)
                time.sleep_ms(1500)
                # ② 下移到抓取位置
                self.kinematic.kinematics_move(self.move_x, self.move_y,
                                               TARGET_GRAB_Z, 1000)
                time.sleep_ms(1500)
                # ③ 夹紧
                self.kinematic.send_str("{#005P1700T1000!}")
                time.sleep_ms(1500)
                # ④ 抬起
                self.kinematic.kinematics_move(self.move_x, self.move_y,
                                               TARGET_LIFT_Z, 1000)
                time.sleep_ms(1500)
                # ⑤ 旋转到放置区方向
                self.move_x = 100
                self.move_y = 60
                self.kinematic.kinematics_move(self.move_x, self.move_y,
                                               TARGET_LIFT_Z, 1000)
                time.sleep_ms(1200)

            # ========== 第2阶段：寻找粉色放置区 ==========
            #  ★ 修复：仅在真正识别到粉色色块时才计数对准
            #  未识别到粉色时原地等待，计数器清零
            elif self.move_status == 2:
                if pink_detected:
                    # 粉色可见 → 正常微调对准
                    if abs(pink_cx - self.mid_block_cx) > 5:
                        if pink_cx > self.mid_block_cx and self.move_y > 1:
                            self.move_y -= 0.3
                        else:
                            self.move_y += 0.3
                    if abs(pink_cy - self.mid_block_cy) > 5:
                        if pink_cy > self.mid_block_cy:
                            self.move_y -= 0.3
                        else:
                            self.move_x += 0.2

                    if abs(pink_cy - self.mid_block_cy) <= 5 and \
                       abs(pink_cx - self.mid_block_cx) <= 5:
                        self.mid_block_cnt += 1
                        if self.mid_block_cnt > 10:
                            self.mid_block_cnt = 0
                            self.move_status = 3
                            self.cap_color_status = color_status
                    else:
                        self.mid_block_cnt = 0
                        self.kinematic.kinematics_move(self.move_x, self.move_y,
                                                       TARGET_APPROACH_Z, 10)
                        time.sleep_ms(10)
                else:
                    # 粉色不可见 → 重置计数器，原地不动等待
                    self.mid_block_cnt = 0
                    self.kinematic.kinematics_move(self.move_x, self.move_y,
                                                   TARGET_APPROACH_Z, 10)
                    time.sleep_ms(10)

            # ========== 第3阶段：投放木块并归位 ==========
            elif self.move_status == 3:
                self.move_status = 0

                # 计算投放点
                l = math.sqrt(self.move_x * self.move_x +
                              self.move_y * self.move_y)
                sin = self.move_y / l
                cos = self.move_x / l
                self.move_x = (l + 85 + cy) * cos
                self.move_y = (l + 85 + cy) * sin
                time.sleep_ms(100)

                # ① 移动到粉色放置区上方（接近高度）
                self.kinematic.kinematics_move(self.move_x, self.move_y,
                                               TARGET_APPROACH_Z, 1000)
                time.sleep_ms(1000)
                # ② 下移到投放位置
                self.kinematic.kinematics_move(self.move_x, self.move_y,
                                               TARGET_GRAB_Z, 1000)
                time.sleep_ms(1200)
                # ③ 松开夹爪
                self.kinematic.send_str("{#005P1000T1000!}")
                time.sleep_ms(1200)
                # ④ 抬起
                self.kinematic.kinematics_move(self.move_x, self.move_y,
                                               TARGET_LIFT_Z, 1000)
                time.sleep_ms(1200)
                # ⑤ 归位
                self.move_x = 0
                self.move_y = 135
                self.kinematic.kinematics_move(self.move_x, self.move_y,
                                               180, 1000)
                time.sleep_ms(1200)

                # 重置
                self.mid_block_cnt = 0
                self.cap_color_status = 0

                # 任务自动流转：黄→橙→结束
                global task_stage
                if task_stage == 1:
                    task_stage = 3   # 黄色投放完成 → 切换抓橙色
                elif task_stage == 3:
                    task_stage = 0   # 橙色投放完成 → 任务结束
                    task_lock = False


# ====================== 全局实例化 ======================
kinematic = Kinematics()
color_Sort = ColorSort()


# ==============================================================
#  主程序入口
# ==============================================================
if __name__ == "__main__":
    colorSort = ColorSort()
    colorSort.init()
    while(1):
        colorSort.run(0, 0, 0)

if __name__ == "__main__":
    while(True):
        if uart.any():
            try:
                string = uart.read()
                print(string, isinstance(string.decode(), str))
                if string:
                    string = string.decode()

                    if string.find("#StartLed!") >= 0:
                        led_dac.pulse_width_percent(50)
                        beep()

                    elif string.find("#StopLed!") >= 0:
                        led_dac.pulse_width_percent(0)
                        beep()

                    elif string.find("#RunStop!") >= 0:
                        run_app_status = 0
                        led_dac.pulse_width_percent(0)
                        beep()
                        task_stage = 0
                        task_lock = False

                    elif string.find("#ColorSort!") >= 0:
                        run_app_status = 1
                        color_Sort.init()
                        beep()

                    elif string.find("#TASKSTART!") >= 0:
                        if not task_lock:
                            task_stage = 1
                            task_lock = True
                            run_app_status = 1
                            color_Sort.init()
                            beep()
                            print("任务启动：先抓黄色，完成后自动抓橙色")
                        else:
                            beep()
                            print("任务已在运行中！")

            except Exception as e:
                print("Error:", e)

        if run_app_status == 1:
            color_Sort.run(cx, cy, cz)
