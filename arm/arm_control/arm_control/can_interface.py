from .PCANBasic import *
import time


class CANInterface:
    def __init__(self):
        self.pcan = PCANBasic()
        self.channel = PCAN_USBBUS1
        self.is_fd = False
        self.is_open = False
        self.baudrate = PCAN_BAUD_500K
        self.bitrate_fd = "f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1"

    def initialize(self, channel, baudrate, is_fd=False):
        self.channel = channel
        self.baudrate = baudrate
        self.is_fd = is_fd

        try:
            if is_fd:
                result = self.pcan.InitializeFD(channel, self.bitrate_fd)
            else:
                result = self.pcan.Initialize(channel, baudrate)

            if result == PCAN_ERROR_OK:
                self.is_open = True
                return True, "CAN接口初始化成功"
            else:
                self.is_open = True   # 让 describe_status 能查 GetErrorText
                detail = self.describe_status(result)
                self.is_open = False
                return False, f"CAN接口初始化失败: {detail}"
        except Exception as e:
            return False, f"CAN接口初始化异常: {str(e)}"

    def close(self):
        try:
            result = self.pcan.Uninitialize(self.channel)
            if result == PCAN_ERROR_OK:
                self.is_open = False
                return True, "CAN接口关闭成功"
            else:
                _, msg = self.pcan.GetErrorText(result)
                return False, f"CAN接口关闭失败: {msg}"
        except Exception as e:
            return False, f"CAN接口关闭异常: {str(e)}"

    def send_message(self, can_id, data, is_extended=False):
        if not self.is_open:
            return False, "CAN接口未初始化"

        try:
            msg = TPCANMsg()
            msg.ID = can_id
            msg.LEN = len(data)
            msg.MSGTYPE = PCAN_MESSAGE_EXTENDED if is_extended else PCAN_MESSAGE_STANDARD

            for i in range(len(data)):
                msg.DATA[i] = data[i]

            result = self.pcan.Write(self.channel, msg)
            if result == PCAN_ERROR_OK:
                return True, "消息发送成功"
            else:
                return False, f"消息发送失败: {self.describe_status(result)}"
        except Exception as e:
            return False, f"消息发送异常: {str(e)}"

    def receive_message(self):
        if not self.is_open:
            return False, "CAN接口未初始化"

        try:
            result, msg, _ = self.pcan.Read(self.channel)
            if result == PCAN_ERROR_OK:
                message = {
                    'id': msg.ID,
                    'data': [msg.DATA[i] for i in range(msg.LEN)],
                    'is_extended': msg.MSGTYPE == PCAN_MESSAGE_EXTENDED,
                    'timestamp': time.time()
                }
                return True, message
            elif result == PCAN_ERROR_QRCVEMPTY:
                return False, "接收队列为空"
            else:
                # 原先只回一句人话文案就把 result 丢了, 于是"没人应答(ACK error)"与
                # "波特率不匹配/线缆问题(bit/stuff error)"与"控制器已 bus-off"三种
                # 完全不同的故障在日志里长得一模一样, 无法据此定位。现在把错误码原样带上。
                return False, f"接收消息失败: {self.describe_status(result)}"
        except Exception as e:
            return False, f"接收消息异常: {str(e)}"

    def describe_status(self, status) -> str:
        """把 TPCANStatus 位掩码翻成可定位的文案(含原始码)。

        PCAN 的 status 是**位掩码**可同时置多位, 所以逐位查而不是相等比较。
        错误类型直接指向不同的排查方向:
          BUSOFF   控制器已自我隔离, 必须 reset 才恢复
          BUSHEAVY/BUSLIGHT/BUSPASSIVE  错误计数在涨 —— 物理层或波特率不匹配
          QRCVEMPTY 正常(队列空)
        """
        raw = status.value if hasattr(status, 'value') else int(status)
        if raw == 0:
            return "OK"
        flags = [
            (PCAN_ERROR_BUSOFF, 'BUSOFF(控制器已隔离,需 reset)'),
            (PCAN_ERROR_BUSPASSIVE, 'BUSPASSIVE(错误被动态)'),
            (PCAN_ERROR_BUSHEAVY, 'BUSHEAVY(错误计数达重限)'),
            (PCAN_ERROR_BUSLIGHT, 'BUSLIGHT(错误计数达轻限)'),
            (PCAN_ERROR_OVERRUN, 'OVERRUN(读太晚)'),
            (PCAN_ERROR_QOVERRUN, 'QOVERRUN(接收队列读太晚)'),
            (PCAN_ERROR_QRCVEMPTY, 'QRCVEMPTY(队列空)'),
            (PCAN_ERROR_QXMTFULL, 'QXMTFULL(发送队列满)'),
            (PCAN_ERROR_XMTFULL, 'XMTFULL(控制器发送缓冲满)'),
            (PCAN_ERROR_NODRIVER, 'NODRIVER(驱动未加载)'),
            (PCAN_ERROR_ILLHW, 'ILLHW(硬件句柄无效)'),
        ]
        hit = [name for flag, name in flags
               if raw & (flag.value if hasattr(flag, 'value') else int(flag))]
        _, text = self.pcan.GetErrorText(status)
        if isinstance(text, bytes):
            text = text.decode('utf-8', 'replace')
        return '0x%05X [%s] %s' % (raw, ' | '.join(hit) if hit else '未知位', text.strip())

    def get_status(self):
        """查控制器当前状态。返回 (raw_code, 文案)。

        必须有: Read() 只在**有帧可读**时才报错, 总线彻底静默(如 bus-off 后没人发)时
        它一直返回 QRCVEMPTY, 于是"总线死了"和"暂时没数据"无法区分。GetStatus 直接问
        控制器自己的状态位, 是唯一能识破静默型故障的入口。
        """
        if not self.is_open:
            return None, "CAN接口未初始化"
        try:
            status = self.pcan.GetStatus(self.channel)
            raw = status.value if hasattr(status, 'value') else int(status)
            return raw, self.describe_status(status)
        except Exception as e:
            return None, f"查状态异常: {str(e)}"

    def is_bus_off(self) -> bool:
        raw, _ = self.get_status()
        if raw is None:
            return False
        busoff = PCAN_ERROR_BUSOFF.value if hasattr(PCAN_ERROR_BUSOFF, 'value') else int(PCAN_ERROR_BUSOFF)
        return bool(raw & busoff)

    def reset(self):
        """重置 CAN 控制器 + 清收发队列。bus-off 后唯一的软恢复路径。

        ⚠️ 只重置 CAN 控制器状态, **不重发电机使能帧** —— 与 can_bridge 的 reenable
        服务是两件事。驱动器若因堵转进了保护态, reset 只会让总线通, 电机仍不动。
        """
        if not self.is_open:
            return False, "CAN接口未初始化"
        try:
            result = self.pcan.Reset(self.channel)
            if result == PCAN_ERROR_OK:
                return True, "CAN 控制器已重置"
            return False, f"CAN 重置失败: {self.describe_status(result)}"
        except Exception as e:
            return False, f"CAN 重置异常: {str(e)}"

    def set_filter(self, from_id, to_id, is_extended=False):
        if not self.is_open:
            return False, "CAN接口未初始化"

        try:
            mode = PCAN_MESSAGE_EXTENDED if is_extended else PCAN_MESSAGE_STANDARD
            result = self.pcan.FilterMessages(self.channel, from_id, to_id, mode)
            if result == PCAN_ERROR_OK:
                return True, "过滤器设置成功"
            else:
                _, msg = self.pcan.GetErrorText(result)
                return False, f"过滤器设置失败: {msg}"
        except Exception as e:
            return False, f"过滤器设置异常: {str(e)}"
