# -*- coding: utf-8 -*-
"""Runtime translation support for the Motor Control GUI.

English strings are used as the lookup keys, so any string that has no
translation registered simply falls back to its English original.  This keeps
the source readable and makes it impossible for a missing key to blank out the
interface.

Strings that embed values use ``{}`` placeholders and are formatted by the
caller, e.g. ``tr("Loaded: {}").format(filename)``.
"""

LANGUAGES = [
    ("en", "English"),
    ("zh_CN", "简体中文"),
    ("zh_TW", "繁體中文"),
]

DEFAULT_LANGUAGE = "en"

LANGUAGE_CODES = [code for code, _ in LANGUAGES]

_ZH_CN = {
    # ── window / tabs ─────────────────────────────────────────────
    "Motor Control GUI": "电机控制界面",
    "Connection": "连接",
    "Motor Control": "电机控制",
    "PID Tuning": "PID 调参",
    "Real-time Data": "实时数据",
    "Limits": "限制",
    "Manual": "手动",
    "IMU 3D": "IMU 3D",

    # ── menus ─────────────────────────────────────────────────────
    "&View": "视图(&V)",
    "&Theme": "主题(&T)",
    "&Language": "语言(&L)",
    "&Help": "帮助(&H)",
    "Light": "浅色",
    "Dark": "深色",
    "About": "关于",
    "About Text": (
        "FOC Motor Control GUI\n\n"
        "A PyQt5 desktop interface for FOC motor controllers over "
        "UART/RS485 or CAN.\n\n"
        "Features: connection management, operating modes, PID tuning, "
        "real-time plots, limits, raw hex console and IMU 3D visualisation."
    ),
    "Close": "关闭",

    # ── connection tab ────────────────────────────────────────────
    "Interface:": "接口：",
    "Serial (UART/RS485)": "串口 (UART/RS485)",
    "CAN": "CAN",
    "Serial Port:": "串口：",
    "Refresh": "刷新",
    "Baudrate:": "波特率：",
    "CAN Channel:": "CAN 通道：",
    "CAN Bustype:": "CAN 适配器类型：",
    "CAN Bitrate:": "CAN 比特率：",
    "Connect": "连接",
    "Disconnect": "断开",
    "Detect Motor ID": "检测电机 ID",
    "Detected Motor ID:": "检测到的电机 ID：",

    # ── configuration group ───────────────────────────────────────
    "Configuration": "配置",
    "Config:": "配置：",
    "Load": "加载",
    "Import…": "导入…",
    "Save As…": "另存为…",
    "No config loaded": "未加载配置",
    "Active: {}": "当前：{}",

    # ── motor selection / mode ────────────────────────────────────
    "Motor Selection": "电机选择",
    "Motor ID:": "电机 ID：",
    "None": "无",
    "Operating Mode": "运行模式",
    "Mode:": "模式：",
    "Set": "设置",
    "Get": "读取",
    "Stop": "停止",
    "Self-test": "自检",
    "Calibration": "校准",
    "Open-loop": "开环",
    "Current loop": "电流环",
    "Speed loop": "速度环",
    "Position loop": "位置环",

    # ── motor parameters ──────────────────────────────────────────
    "Motor Parameters": "电机参数",
    "Pole Pairs:": "极对数：",
    "Zero Offset (°):": "零点偏移 (°)：",
    "Encoder Direction:": "编码器方向：",
    "Get Parameters": "读取参数",

    # ── targets ───────────────────────────────────────────────────
    "Target Values": "目标值",
    "Iq:": "Iq：",
    "Id:": "Id：",
    "Speed (rpm):": "速度 (rpm)：",
    "Position (deg):": "位置 (deg)：",
    "Uq:": "Uq：",
    "Ud:": "Ud：",
    "Set All": "全部设置",
    "Get All": "全部读取",
    "Get Speed": "读取速度",

    # ── phase currents ────────────────────────────────────────────
    "Phase Currents": "相电流",
    "Ia:": "Ia：",
    "Ib:": "Ib：",
    "Ic:": "Ic：",

    # ── auto refresh ──────────────────────────────────────────────
    "Auto Refresh": "自动刷新",
    "Enable": "启用",
    "Interval:": "间隔：",
    "Auto Refresh Preview": "自动刷新预览",

    # ── motor preview ─────────────────────────────────────────────
    "Motor Preview": "电机预览",
    "Gear Ratio:": "减速比：",
    "Actual Angle: --- °": "实际角度：--- °",
    "Raw Motor Angle: --- °": "电机原始角度：--- °",
    "Total rotations: ---": "总圈数：---",
    "Mod angle (0-360°): ---": "取模角度 (0-360°)：---",
    "Motor Speed: --- rpm": "电机转速：--- rpm",
    "Actual Angle: {}°  (Rot: {}, Mod: {}°)": "实际角度：{}°  （圈数：{}，取模：{}°）",
    "Raw Motor Angle: {}°  (Rot: {}, Mod: {}°)": "电机原始角度：{}°  （圈数：{}，取模：{}°）",
    "Total rotations: {}": "总圈数：{}",
    "Mod angle (0-360°): {}°": "取模角度 (0-360°)：{}°",
    "Motor Speed: {} rpm": "电机转速：{} rpm",

    # ── PID ───────────────────────────────────────────────────────
    "{} PID": "{} PID",
    "P:": "P：",
    "I:": "I：",
    "D:": "D：",

    # ── real-time data ────────────────────────────────────────────
    "Plot:": "曲线：",
    "Enable Polling": "启用轮询",
    "Value": "数值",
    "Time (samples)": "时间（采样点）",
    "Save Data to CSV": "保存数据为 CSV",

    # ── limits ────────────────────────────────────────────────────
    "Iq max:": "Iq 上限：",
    "Iq min:": "Iq 下限：",
    "Id max:": "Id 上限：",
    "Id min:": "Id 下限：",
    "Speed max:": "速度上限：",
    "Speed min:": "速度下限：",
    "Position max:": "位置上限：",
    "Position min:": "位置下限：",
    "Set Limits": "设置限制",
    "Get Limits": "读取限制",

    # ── manual tab ────────────────────────────────────────────────
    "Send Command (Hex)": "发送命令（十六进制）",
    "Send": "发送",
    "Response (Raw Hex)": "响应（原始十六进制）",
    "Clear": "清除",

    # ── IMU tab ───────────────────────────────────────────────────
    "Enable IMU Polling": "启用 IMU 轮询",
    "Interval (ms):": "间隔 (ms)：",
    "3D Model:": "3D 模型：",
    "Browse": "浏览",
    "Reset": "重置",
    "Default Cube": "默认立方体",
    "IMU Data": "IMU 数据",
    "Acc (g):": "加速度 (g)：",
    "Gyro (dps):": "角速度 (dps)：",
    "Orientation (°):": "姿态 (°)：",
    "IMU Data Log": "IMU 数据记录",
    "Clear Log": "清除记录",
    "Save Log to CSV": "保存记录为 CSV",
    "Auto Log": "自动记录",
    "IMU Debug Data": "IMU 调试数据",
    "Copy Current IMU Data": "复制当前 IMU 数据",
    "Roll: ---": "横滚：---",
    "Pitch: ---": "俯仰：---",
    "Yaw: ---": "偏航：---",
    "Roll: {}°": "横滚：{}°",
    "Pitch: {}°": "俯仰：{}°",
    "Yaw: {}°": "偏航：{}°",

    # ── IMU debug dump ────────────────────────────────────────────
    "Timestamp:": "时间戳：",
    "Accel (g):": "加速度 (g)：",
    "Gyro raw (dps):": "陀螺仪原始值 (dps)：",
    "Gyro bias (dps):": "陀螺仪零偏 (dps)：",
    "Gyro cal (dps):": "陀螺仪校准值 (dps)：",
    "Temperature:": "温度：",
    "Orientation (deg):": "姿态 (deg)：",
    "Raw LSBs:": "原始 LSB：",

    # ── status bar ────────────────────────────────────────────────
    "Not connected": "未连接",
    "Connected": "已连接",
    "Error: {}": "错误：{}",
    "IMU data copied to clipboard": "IMU 数据已复制到剪贴板",
    "IMU calibrating... Keep device still": "IMU 校准中…请保持设备静止",
    "IMU polling stopped": "IMU 轮询已停止",
    "Calibrating IMU... {}/{}": "IMU 校准中… {}/{}",
    "IMU ready (bias: {}, {}, {})": "IMU 就绪（零偏：{}, {}, {}）",

    # ── dialogs / messages ────────────────────────────────────────
    "Error": "错误",
    "Warning": "警告",
    "Detect": "检测",
    "Detected IDs: {}": "检测到的 ID：{}",
    "No motor found": "未找到电机",
    "No motor ID": "未选择电机 ID",
    "No motor ID selected": "未选择电机 ID",
    "Polling": "轮询",
    "Detect motor ID first": "请先检测电机 ID",
    "Comm Error": "通信错误",
    "No serial port": "无可用串口",
    "python-can not installed": "未安装 python-can",
    "Saved": "已保存",
    "Log saved to {}": "记录已保存至 {}",
    "Save Data": "保存数据",
    "Save IMU Log": "保存 IMU 记录",
    "Select 3D Model": "选择 3D 模型",
    "3D Models (*.stl *.obj *.ply *.step *.stp)": "3D 模型 (*.stl *.obj *.ply *.step *.stp)",
    "Missing Library": "缺少库",
    "trimesh not installed": "未安装 trimesh",
    "trimesh library not installed.": "未安装 trimesh 库。",
    "Load Failed": "加载失败",
    "Failed to load {}": "无法加载 {}",
    "Invalid hex string": "无效的十六进制字符串",
    "No Config": "无配置",
    "No config file selected.": "未选择配置文件。",
    "Config": "配置",
    "Loaded: {}": "已加载：{}",
    "Failed to load: {}": "加载失败：{}",
    "Import Config": "导入配置",
    "Imported": "已导入",
    "Imported and loaded: {}": "已导入并加载：{}",
    "Import failed: {}": "导入失败：{}",
    "Save Config As": "配置另存为",
    "Saved to {}": "已保存至 {}",
    "Save failed: {}": "保存失败：{}",
    "Auto-refresh ports": "自动刷新串口",
    "Searching for motor ID...": "正在搜索电机 ID...",
    "Motor ID not detected": "未检测到电机 ID",
}

_ZH_TW = {
    # ── window / tabs ─────────────────────────────────────────────
    "Motor Control GUI": "馬達控制介面",
    "Connection": "連線",
    "Motor Control": "馬達控制",
    "PID Tuning": "PID 調參",
    "Real-time Data": "即時資料",
    "Limits": "限制",
    "Manual": "手動",
    "IMU 3D": "IMU 3D",

    # ── menus ─────────────────────────────────────────────────────
    "&View": "檢視(&V)",
    "&Theme": "主題(&T)",
    "&Language": "語言(&L)",
    "&Help": "說明(&H)",
    "Light": "淺色",
    "Dark": "深色",
    "About": "關於",
    "About Text": (
        "FOC 馬達控制介面\n\n"
        "透過 UART/RS485 或 CAN 控制 FOC 馬達的 PyQt5 桌面程式。\n\n"
        "功能：連線管理、運轉模式、PID 調參、即時曲線、限制設定、"
        "原始十六進位主控台與 IMU 3D 視覺化。"
    ),
    "Close": "關閉",

    # ── connection tab ────────────────────────────────────────────
    "Interface:": "介面：",
    "Serial (UART/RS485)": "串列埠 (UART/RS485)",
    "CAN": "CAN",
    "Serial Port:": "串列埠：",
    "Refresh": "重新整理",
    "Baudrate:": "鮑率：",
    "CAN Channel:": "CAN 通道：",
    "CAN Bustype:": "CAN 匯流排類型：",
    "CAN Bitrate:": "CAN 位元率：",
    "Connect": "連線",
    "Disconnect": "中斷連線",
    "Detect Motor ID": "偵測馬達 ID",
    "Detected Motor ID:": "偵測到的馬達 ID：",

    # ── configuration group ───────────────────────────────────────
    "Configuration": "設定",
    "Config:": "設定：",
    "Load": "載入",
    "Import…": "匯入…",
    "Save As…": "另存新檔…",
    "No config loaded": "未載入設定",
    "Active: {}": "目前：{}",

    # ── motor selection / mode ────────────────────────────────────
    "Motor Selection": "馬達選擇",
    "Motor ID:": "馬達 ID：",
    "None": "無",
    "Operating Mode": "運轉模式",
    "Mode:": "模式：",
    "Set": "設定",
    "Get": "讀取",
    "Stop": "停止",
    "Self-test": "自我測試",
    "Calibration": "校正",
    "Open-loop": "開迴路",
    "Current loop": "電流環",
    "Speed loop": "速度環",
    "Position loop": "位置環",

    # ── motor parameters ──────────────────────────────────────────
    "Motor Parameters": "馬達參數",
    "Pole Pairs:": "極對數：",
    "Zero Offset (°):": "零點偏移 (°)：",
    "Encoder Direction:": "編碼器方向：",
    "Get Parameters": "讀取參數",

    # ── targets ───────────────────────────────────────────────────
    "Target Values": "目標值",
    "Iq:": "Iq：",
    "Id:": "Id：",
    "Speed (rpm):": "轉速 (rpm)：",
    "Position (deg):": "位置 (deg)：",
    "Uq:": "Uq：",
    "Ud:": "Ud：",
    "Set All": "全部設定",
    "Get All": "全部讀取",
    "Get Speed": "讀取轉速",

    # ── phase currents ────────────────────────────────────────────
    "Phase Currents": "相電流",
    "Ia:": "Ia：",
    "Ib:": "Ib：",
    "Ic:": "Ic：",

    # ── auto refresh ──────────────────────────────────────────────
    "Auto Refresh": "自動重新整理",
    "Enable": "啟用",
    "Interval:": "間隔：",
    "Auto Refresh Preview": "自動重新整理預覽",

    # ── motor preview ─────────────────────────────────────────────
    "Motor Preview": "馬達預覽",
    "Gear Ratio:": "減速比：",
    "Actual Angle: --- °": "實際角度：--- °",
    "Raw Motor Angle: --- °": "馬達原始角度：--- °",
    "Total rotations: ---": "總圈數：---",
    "Mod angle (0-360°): ---": "取模角度 (0-360°)：---",
    "Motor Speed: --- rpm": "馬達轉速：--- rpm",
    "Actual Angle: {}°  (Rot: {}, Mod: {}°)": "實際角度：{}°  （圈數：{}，取模：{}°）",
    "Raw Motor Angle: {}°  (Rot: {}, Mod: {}°)": "馬達原始角度：{}°  （圈數：{}，取模：{}°）",
    "Total rotations: {}": "總圈數：{}",
    "Mod angle (0-360°): {}°": "取模角度 (0-360°)：{}°",
    "Motor Speed: {} rpm": "馬達轉速：{} rpm",

    # ── PID ───────────────────────────────────────────────────────
    "{} PID": "{} PID",
    "P:": "P：",
    "I:": "I：",
    "D:": "D：",

    # ── real-time data ────────────────────────────────────────────
    "Plot:": "曲線：",
    "Enable Polling": "啟用輪詢",
    "Value": "數值",
    "Time (samples)": "時間（取樣點）",
    "Save Data to CSV": "儲存資料為 CSV",

    # ── limits ────────────────────────────────────────────────────
    "Iq max:": "Iq 上限：",
    "Iq min:": "Iq 下限：",
    "Id max:": "Id 上限：",
    "Id min:": "Id 下限：",
    "Speed max:": "轉速上限：",
    "Speed min:": "轉速下限：",
    "Position max:": "位置上限：",
    "Position min:": "位置下限：",
    "Set Limits": "設定限制",
    "Get Limits": "讀取限制",

    # ── manual tab ────────────────────────────────────────────────
    "Send Command (Hex)": "傳送命令（十六進位）",
    "Send": "傳送",
    "Response (Raw Hex)": "回應（原始十六進位）",
    "Clear": "清除",

    # ── IMU tab ───────────────────────────────────────────────────
    "Enable IMU Polling": "啟用 IMU 輪詢",
    "Interval (ms):": "間隔 (ms)：",
    "3D Model:": "3D 模型：",
    "Browse": "瀏覽",
    "Reset": "重設",
    "Default Cube": "預設立方體",
    "IMU Data": "IMU 資料",
    "Acc (g):": "加速度 (g)：",
    "Gyro (dps):": "角速度 (dps)：",
    "Orientation (°):": "姿態 (°)：",
    "IMU Data Log": "IMU 資料記錄",
    "Clear Log": "清除記錄",
    "Save Log to CSV": "儲存記錄為 CSV",
    "Auto Log": "自動記錄",
    "IMU Debug Data": "IMU 偵錯資料",
    "Copy Current IMU Data": "複製目前 IMU 資料",
    "Roll: ---": "橫滾：---",
    "Pitch: ---": "俯仰：---",
    "Yaw: ---": "偏航：---",
    "Roll: {}°": "橫滾：{}°",
    "Pitch: {}°": "俯仰：{}°",
    "Yaw: {}°": "偏航：{}°",

    # ── IMU debug dump ────────────────────────────────────────────
    "Timestamp:": "時間戳記：",
    "Accel (g):": "加速度 (g)：",
    "Gyro raw (dps):": "陀螺儀原始值 (dps)：",
    "Gyro bias (dps):": "陀螺儀零偏 (dps)：",
    "Gyro cal (dps):": "陀螺儀校正值 (dps)：",
    "Temperature:": "溫度：",
    "Orientation (deg):": "姿態 (deg)：",
    "Raw LSBs:": "原始 LSB：",

    # ── status bar ────────────────────────────────────────────────
    "Not connected": "未連線",
    "Connected": "已連線",
    "Error: {}": "錯誤：{}",
    "IMU data copied to clipboard": "IMU 資料已複製到剪貼簿",
    "IMU calibrating... Keep device still": "IMU 校正中…請保持裝置靜止",
    "IMU polling stopped": "IMU 輪詢已停止",
    "Calibrating IMU... {}/{}": "IMU 校正中… {}/{}",
    "IMU ready (bias: {}, {}, {})": "IMU 就緒（零偏：{}, {}, {}）",

    # ── dialogs / messages ────────────────────────────────────────
    "Error": "錯誤",
    "Warning": "警告",
    "Detect": "偵測",
    "Detected IDs: {}": "偵測到的 ID：{}",
    "No motor found": "未找到馬達",
    "No motor ID": "未選擇馬達 ID",
    "No motor ID selected": "未選擇馬達 ID",
    "Polling": "輪詢",
    "Detect motor ID first": "請先偵測馬達 ID",
    "Comm Error": "通訊錯誤",
    "No serial port": "無可用串列埠",
    "python-can not installed": "未安裝 python-can",
    "Saved": "已儲存",
    "Log saved to {}": "記錄已儲存至 {}",
    "Save Data": "儲存資料",
    "Save IMU Log": "儲存 IMU 記錄",
    "Select 3D Model": "選擇 3D 模型",
    "3D Models (*.stl *.obj *.ply *.step *.stp)": "3D 模型 (*.stl *.obj *.ply *.step *.stp)",
    "Missing Library": "缺少函式庫",
    "trimesh not installed": "未安裝 trimesh",
    "trimesh library not installed.": "未安裝 trimesh 函式庫。",
    "Load Failed": "載入失敗",
    "Failed to load {}": "無法載入 {}",
    "Invalid hex string": "無效的十六進位字串",
    "No Config": "無設定",
    "No config file selected.": "未選擇設定檔。",
    "Config": "設定",
    "Loaded: {}": "已載入：{}",
    "Failed to load: {}": "載入失敗：{}",
    "Import Config": "匯入設定",
    "Imported": "已匯入",
    "Imported and loaded: {}": "已匯入並載入：{}",
    "Import failed: {}": "匯入失敗：{}",
    "Save Config As": "設定另存新檔",
    "Saved to {}": "已儲存至 {}",
    "Save failed: {}": "儲存失敗：{}",
    "Auto-refresh ports": "自動重新整理序列埠",
    "Searching for motor ID...": "正在搜尋馬達 ID...",
    "Motor ID not detected": "未偵測到馬達 ID",
}

_TRANSLATIONS = {
    "zh_CN": _ZH_CN,
    "zh_TW": _ZH_TW,
}


def language_names():
    """Return a {code: native_name} mapping for the language selector."""
    return dict(LANGUAGES)


class Translator:
    """Callable lookup table that resolves a source string for one language."""

    def __init__(self, language=DEFAULT_LANGUAGE):
        self._language = language if language in LANGUAGE_CODES else DEFAULT_LANGUAGE

    @property
    def language(self):
        return self._language

    def set_language(self, language):
        """Switch language.  Returns True when the language actually changed."""
        if language not in LANGUAGE_CODES or language == self._language:
            return False
        self._language = language
        return True

    def __call__(self, text):
        return _TRANSLATIONS.get(self._language, {}).get(text, text)


# Module level translator shared by every widget.
tr = Translator()
