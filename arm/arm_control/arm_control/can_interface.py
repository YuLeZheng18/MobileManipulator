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
                _, msg = self.pcan.GetErrorText(result)
                return False, f"CAN接口初始化失败: {msg}"
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
                _, msg = self.pcan.GetErrorText(result)
                return False, f"消息发送失败: {msg}"
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
                _, err_msg = self.pcan.GetErrorText(result)
                return False, f"接收消息失败: {err_msg}"
        except Exception as e:
            return False, f"接收消息异常: {str(e)}"

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
