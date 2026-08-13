from maix import camera, display, image, time, app
from maix import uart, pinmap, pwm, err

# ============================================================
# MaixCam Pro -> STM32F407 飞控视觉联调程序
# 协议：
#   飞控 -> 相机：BB code FF
#   相机 -> 飞控：BB code ctrl x_sign x_hi x_lo y_sign y_hi y_lo FF
# 坐标定义：
#   x 正方向：机头前方
#   y 正方向：机头左侧
# ============================================================

# ===================== 图像与串口配置 =====================
CAM_W = 320
CAM_H = 240
UART_DEVICE = "/dev/ttyS1"
BAUD = 115200

# ===================== 坐标标定参数 =====================
CAMERA_HEIGHT_CM = 85.0
FOCUS_LEN_PX = 300.0

CONTROL_GAIN_X = 1.0
CONTROL_GAIN_Y = 1.0

GAIN_X_POS = 1.0
GAIN_X_NEG = 1.0
GAIN_Y_POS = 1.0
GAIN_Y_NEG = 1.0

BODY_X_SIGN = 1.0
BODY_Y_SIGN = 1.0

CAMERA_YAW_DEG = 0.0

# 图像中心死区：小于该像素误差时直接发送 0，避免中心附近抖动
CENTER_DEADBAND_PX = 6

# 一阶低通滤波，越大响应越快，越小越稳
FILTER_ALPHA = 0.40

# 发送值限幅，单位 cm
SEND_LIMIT_CM = 80

# ===================== 投放判断参数 =====================
# 二维码中心进入这个像素范围后，开始认为“具备投放条件”
DROP_CENTER_THRESHOLD_PX = 24

# 需要在投放区域内持续保持的时间，单位 ms
# 2000ms = 2秒
DROP_HOLD_MS = 2000

# 允许短暂抖动 / 丢帧的时间
# 例如某一两帧偏出去，不会立刻重新计时
DROP_LOST_GRACE_MS = 300

# ===================== 舵机配置 =====================
SERVO_PWM_ID = 4
SERVO_PIN_NAME = "A16"
SERVO_FREQ = 50
SERVO_MIN_DUTY = 2.5
SERVO_MAX_DUTY = 12.5

# 舵机动作参数：非阻塞执行
SERVO_CLOSE_ANGLE = 0
SERVO_OPEN_ANGLE = 100
SERVO_STEP_DEG = 15
SERVO_UPDATE_INTERVAL_MS = 10
SERVO_HOLD_MS = 800

# ===================== 协议常量 =====================
QR_CODE_A = 0xCA
QR_CODE_B = 0xCB
QR_CODE_C = 0xCC
CAM_CTRL_FC = 0x50
CAM_CTRL_CAM = 0x60
FRAME_HEAD = 0xBB
FRAME_END = 0xFF

# ---------- 初始化 ----------
cam = camera.Camera(CAM_W, CAM_H)
disp = display.Display()

print("Setting UART1 pins...")
pinmap.set_pin_function("A19", "UART1_TX")
pinmap.set_pin_function("A18", "UART1_RX")
serial = uart.UART(UART_DEVICE, BAUD)
print("UART1 ready: {} @ {}".format(UART_DEVICE, BAUD))

print("Setting {} as PWM{}".format(SERVO_PIN_NAME, SERVO_PWM_ID))
err.check_raise(pinmap.set_pin_function(SERVO_PIN_NAME, "PWM{}".format(SERVO_PWM_ID)))
servo = pwm.PWM(SERVO_PWM_ID, freq=SERVO_FREQ, duty=SERVO_MIN_DUTY, enable=True)
print("Servo PWM on {} (PWM{})".format(SERVO_PIN_NAME, SERVO_PWM_ID))

print("Initializing servo to 0 deg...")
servo.duty(SERVO_MIN_DUTY)
time.sleep(2)
print("Servo initialization done.")

# ---------- 全局状态 ----------
target_qr_type = None

filter_inited = False
filt_x = 0.0
filt_y = 0.0

release_frame_div = 0
cam_cx, cam_cy = CAM_W // 2, CAM_H // 2

# 投放保持状态
drop_holding = False
drop_hold_start_ms = 0
drop_last_good_ms = 0
servo_triggered = False

# 舵机非阻塞状态
servo_state = "IDLE"
servo_current_angle = SERVO_CLOSE_ANGLE
servo_last_update_ms = 0
servo_hold_start_ms = 0


# ---------- 工具函数 ----------
def now_ms():
    return time.ticks_ms()


def elapsed_ms(start_ms):
    return time.ticks_ms() - start_ms


def tag_id_to_qr_type(tag_id):
    if tag_id == 2:
        return QR_CODE_A
    elif tag_id == 0:
        return QR_CODE_B
    elif tag_id == 1:
        return QR_CODE_C
    else:
        return None


def is_valid_qr_type(code):
    return code in (QR_CODE_A, QR_CODE_B, QR_CODE_C)


def qr_name(code):
    if code == QR_CODE_A:
        return "A"
    if code == QR_CODE_B:
        return "B"
    if code == QR_CODE_C:
        return "C"
    return "?"


def clamp(v, v_min, v_max):
    if v < v_min:
        return v_min
    if v > v_max:
        return v_max
    return v


def pack_coord_with_sign(value_cm):
    """按协议打包：符号位 + uint16 绝对值。"""
    sign = 0x01 if value_cm < 0 else 0x00
    abs_val = int(round(abs(value_cm)))
    if abs_val > 32767:
        abs_val = 32767
    return sign, (abs_val >> 8) & 0xFF, abs_val & 0xFF


def send_cam_frame(code_type, control, x_cm, y_cm):
    if not is_valid_qr_type(code_type):
        return

    x_sign, x_hi, x_lo = pack_coord_with_sign(x_cm)
    y_sign, y_hi, y_lo = pack_coord_with_sign(y_cm)

    frame = bytearray([
        FRAME_HEAD, code_type, control,
        x_sign, x_hi, x_lo,
        y_sign, y_hi, y_lo,
        FRAME_END,
    ])

    serial.write(bytes(frame))


def parse_fc_cmd():
    """解析飞控下发的 BB code FF。"""
    raw = serial.read()
    if not raw:
        return None

    for i in range(len(raw) - 2):
        if raw[i] == FRAME_HEAD and raw[i + 2] == FRAME_END:
            code = raw[i + 1]
            if is_valid_qr_type(code):
                return code

    return None


def angle_to_duty(angle):
    angle = max(0, min(180, angle))
    return SERVO_MIN_DUTY + (angle / 180.0) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY)


def sin_deg(deg):
    import math
    return math.sin(math.radians(deg))


def cos_deg(deg):
    import math
    return math.cos(math.radians(deg))


def apply_direction_gain(x_cm, y_cm):
    if x_cm >= 0:
        x_cm *= GAIN_X_POS
    else:
        x_cm *= GAIN_X_NEG

    if y_cm >= 0:
        y_cm *= GAIN_Y_POS
    else:
        y_cm *= GAIN_Y_NEG

    return x_cm, y_cm


def calc_body_error_from_pixel(dx_pixel, dy_pixel):
    """
    图像误差 -> 机体系误差。
    图像 dx: 右为正；图像 dy: 下为正。
    机体系 x: 机头前为正；机体系 y: 机头左为正。
    当前保持原有映射：body_x = -dy_ground, body_y = -dx_ground。
    """
    if abs(dx_pixel) <= CENTER_DEADBAND_PX:
        dx_pixel = 0
    if abs(dy_pixel) <= CENTER_DEADBAND_PX:
        dy_pixel = 0

    cm_per_pixel = CAMERA_HEIGHT_CM / FOCUS_LEN_PX

    dx_ground = dx_pixel * cm_per_pixel
    dy_ground = dy_pixel * cm_per_pixel

    body_x = BODY_X_SIGN * (-dy_ground) * CONTROL_GAIN_X
    body_y = BODY_Y_SIGN * (-dx_ground) * CONTROL_GAIN_Y

    if CAMERA_YAW_DEG != 0.0:
        c = cos_deg(CAMERA_YAW_DEG)
        s = sin_deg(CAMERA_YAW_DEG)
        rot_x = body_x * c - body_y * s
        rot_y = body_x * s + body_y * c
        body_x, body_y = rot_x, rot_y

    body_x, body_y = apply_direction_gain(body_x, body_y)

    body_x = clamp(body_x, -SEND_LIMIT_CM, SEND_LIMIT_CM)
    body_y = clamp(body_y, -SEND_LIMIT_CM, SEND_LIMIT_CM)

    return body_x, body_y


def filter_error(x_cm, y_cm, target_found):
    global filter_inited, filt_x, filt_y

    if not target_found:
        filter_inited = False
        filt_x = 0.0
        filt_y = 0.0
        return 0.0, 0.0

    if not filter_inited:
        filt_x = x_cm
        filt_y = y_cm
        filter_inited = True
    else:
        filt_x = FILTER_ALPHA * x_cm + (1.0 - FILTER_ALPHA) * filt_x
        filt_y = FILTER_ALPHA * y_cm + (1.0 - FILTER_ALPHA) * filt_y

    return filt_x, filt_y


def find_best_target_tag(tags, target_code):
    """只选择目标二维码；若多个目标同时出现，选面积最大的。"""
    best_tag = None
    best_area = -1
    best_qr = None

    if target_code is None:
        return None, None

    for tag in tags:
        qr_type = tag_id_to_qr_type(tag.id())
        if qr_type != target_code:
            continue

        area = tag.w() * tag.h()

        if area > best_area:
            best_area = area
            best_tag = tag
            best_qr = qr_type

    return best_tag, best_qr


def reset_drop_hold():
    global drop_holding, drop_hold_start_ms, drop_last_good_ms

    drop_holding = False
    drop_hold_start_ms = 0
    drop_last_good_ms = 0


def update_drop_hold(target_found, dx_pixel, dy_pixel):
    """
    非阻塞投放保持判断。

    逻辑：
    1. 目标进入 DROP_CENTER_THRESHOLD_PX 范围，开始计时；
    2. 持续保持 DROP_HOLD_MS 后，允许投放；
    3. 中途短暂偏出 / 丢帧，小于 DROP_LOST_GRACE_MS 不重置；
    4. 偏出太久则重新计时。
    """
    global drop_holding, drop_hold_start_ms, drop_last_good_ms

    if servo_triggered:
        return False

    if servo_state != "IDLE":
        return False

    now = now_ms()

    in_drop_zone = (
        target_found and
        abs(dx_pixel) <= DROP_CENTER_THRESHOLD_PX and
        abs(dy_pixel) <= DROP_CENTER_THRESHOLD_PX
    )

    if in_drop_zone:
        if not drop_holding:
            drop_holding = True
            drop_hold_start_ms = now
            print("Drop hold start")

        drop_last_good_ms = now

    else:
        if drop_holding:
            if elapsed_ms(drop_last_good_ms) > DROP_LOST_GRACE_MS:
                print("Drop hold reset: target moved out too long")
                reset_drop_hold()

        return False

    if drop_holding and elapsed_ms(drop_hold_start_ms) >= DROP_HOLD_MS:
        return True

    return False


def get_drop_hold_progress():
    if not drop_holding:
        return 0

    progress = elapsed_ms(drop_hold_start_ms)
    if progress < 0:
        progress = 0

    if progress > DROP_HOLD_MS:
        progress = DROP_HOLD_MS

    return progress


def reset_servo_state():
    global servo_state, servo_current_angle, servo_last_update_ms, servo_hold_start_ms

    servo_state = "IDLE"
    servo_current_angle = SERVO_CLOSE_ANGLE
    servo_last_update_ms = now_ms()
    servo_hold_start_ms = 0
    servo.duty(angle_to_duty(SERVO_CLOSE_ANGLE))


def start_servo_action():
    global servo_state, servo_current_angle, servo_last_update_ms, servo_hold_start_ms

    servo_state = "OPENING"
    servo_current_angle = SERVO_CLOSE_ANGLE
    servo_last_update_ms = now_ms()
    servo_hold_start_ms = 0
    servo.duty(angle_to_duty(servo_current_angle))

    print("Servo action start")


def update_servo_non_blocking():
    """
    非阻塞舵机动作状态机。
    每次主循环只更新一点点角度，不使用 sleep 卡住主循环。
    """
    global servo_state, servo_current_angle, servo_last_update_ms, servo_hold_start_ms

    if servo_state == "IDLE":
        return

    now = now_ms()

    if elapsed_ms(servo_last_update_ms) < SERVO_UPDATE_INTERVAL_MS:
        return

    servo_last_update_ms = now

    if servo_state == "OPENING":
        servo_current_angle += SERVO_STEP_DEG

        if servo_current_angle >= SERVO_OPEN_ANGLE:
            servo_current_angle = SERVO_OPEN_ANGLE
            servo_state = "HOLDING"
            servo_hold_start_ms = now
            print("Servo opened, holding")

        servo.duty(angle_to_duty(servo_current_angle))

    elif servo_state == "HOLDING":
        servo.duty(angle_to_duty(SERVO_OPEN_ANGLE))

        if elapsed_ms(servo_hold_start_ms) >= SERVO_HOLD_MS:
            servo_state = "CLOSING"
            print("Servo closing")

    elif servo_state == "CLOSING":
        servo_current_angle -= SERVO_STEP_DEG

        if servo_current_angle <= SERVO_CLOSE_ANGLE:
            servo_current_angle = SERVO_CLOSE_ANGLE
            servo_state = "IDLE"
            print("Servo action done")

        servo.duty(angle_to_duty(servo_current_angle))


# ---------- 主循环 ----------
print("Waiting for FC command: BB code FF ...")

print_div = 0

while not app.need_exit():
    img = cam.read()
    img.draw_cross(cam_cx, cam_cy, image.COLOR_YELLOW, 5)

    # 1. 接收飞控指定二维码类别
    cmd = parse_fc_cmd()

    if cmd is not None:
        target_qr_type = cmd

        filter_inited = False
        reset_drop_hold()
        reset_servo_state()
        servo_triggered = False

        print("New target QR: 0x{:02X} ({})".format(cmd, qr_name(cmd)))

    # 2. 查找目标二维码
    tags = img.find_apriltags()

    target_found = False
    dx_pixel = 0
    dy_pixel = 0
    send_x = 0.0
    send_y = 0.0

    target_tag, qr_type = find_best_target_tag(tags, target_qr_type) if tags else (None, None)

    if target_tag is not None:
        target_found = True

        tag_cx = target_tag.x() + target_tag.w() // 2
        tag_cy = target_tag.y() + target_tag.h() // 2

        dx_pixel = tag_cx - cam_cx
        dy_pixel = tag_cy - cam_cy

        raw_x, raw_y = calc_body_error_from_pixel(dx_pixel, dy_pixel)
        send_x, send_y = filter_error(raw_x, raw_y, True)

        # 按协议发送相机接管帧
        send_cam_frame(qr_type, CAM_CTRL_CAM, send_x, send_y)

        img.draw_rect(target_tag.x(), target_tag.y(), target_tag.w(), target_tag.h(), image.COLOR_GREEN, 2)
        img.draw_string(
            target_tag.x(),
            target_tag.y() - 10,
            "ID:{} ({})".format(target_tag.id(), qr_name(qr_type)),
            image.COLOR_RED
        )
        img.draw_line(cam_cx, cam_cy, tag_cx, tag_cy, image.COLOR_RED, 1)

        print_div += 1
        if print_div >= 10:
            print_div = 0
            print("Sent CAM 0x{:02X} px=({:+d},{:+d}) err=({:+.1f},{:+.1f}) hold={}ms servo={}".format(
                qr_type,
                dx_pixel,
                dy_pixel,
                send_x,
                send_y,
                get_drop_hold_progress(),
                servo_state
            ))

    else:
        filter_error(0.0, 0.0, False)

        # 未识别到目标：周期性发送 0x50 释放控制权
        if target_qr_type is not None:
            release_frame_div += 1

            if release_frame_div >= 5:
                release_frame_div = 0
                send_cam_frame(target_qr_type, CAM_CTRL_FC, 0, 0)
        else:
            release_frame_div = 0

    # 3. 投放保持判断：进入中心区域后，保持 2 秒再投放
    should_drop = update_drop_hold(target_found, dx_pixel, dy_pixel)

    if should_drop and not servo_triggered:
        print("Target held for {} ms, executing servo...".format(DROP_HOLD_MS))
        servo_triggered = True
        reset_drop_hold()
        start_servo_action()

    # 4. 非阻塞更新舵机
    update_servo_non_blocking()

    # 5. 屏幕显示
    status = "Target: {}".format(qr_name(target_qr_type) if target_qr_type else "None")

    if target_found:
        status += " | px:({:+d},{:+d})".format(dx_pixel, dy_pixel)
        status += " | XY:({:+.0f},{:+.0f})".format(send_x, send_y)

        if drop_holding:
            status += " | Hold:{}/{}ms".format(get_drop_hold_progress(), DROP_HOLD_MS)
        else:
            status += " | Hold:0/{}ms".format(DROP_HOLD_MS)

        status += " | Servo:{}".format(servo_state)

        if servo_triggered:
            status += " done"
    else:
        status += " | No target"
        if drop_holding:
            status += " | Grace"

    img.draw_string(
        0,
        20,
        "H:{:.0f}cm F:{:.0f}px".format(CAMERA_HEIGHT_CM, FOCUS_LEN_PX),
        image.COLOR_BLUE
    )

    img.draw_string(
        0,
        40,
        "Drop:{}px Hold:{}ms".format(DROP_CENTER_THRESHOLD_PX, DROP_HOLD_MS),
        image.COLOR_BLUE
    )

    img.draw_string(
        0,
        CAM_H - 20,
        status,
        image.COLOR_GREEN
    )

    disp.show(img)