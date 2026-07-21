# for motion switcher
import json
import logging_mp
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
# for loco client
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.rpc.client import Client

logger_mp = logging_mp.getLogger(__name__)

# MotionSwitcher used to switch mode between debug mode and ai mode
class MotionSwitcher:
    def __init__(self):
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(1.0)
        self.msc.Init()

    def Enter_Debug_Mode(self):
        try:
            status, result = self.msc.CheckMode()
            while result['name']:
                self.msc.ReleaseMode()
                status, result = self.msc.CheckMode()
                time.sleep(1)
            return status, result
        except Exception as e:
            return None, None
    
    def Exit_Debug_Mode(self):
        try:
            status, result = self.msc.SelectMode(nameOrAlias='ai')
            return status, result
        except Exception as e:
            return None, None

class LocoClientWrapper:
    def __init__(self):
        self.client = LocoClient()
        self.client.SetTimeout(0.0001)
        self.client.Init()

    def Enter_Damp_Mode(self):
        self.client.Damp()

    def Damp(self):
        self.Enter_Damp_Mode()
    
    def Move(self, vx, vy, vyaw):
        self.client.Move(vx, vy, vyaw, continous_move=False)

    def Stop(self):
        self.client.StopMove()


class G1DAgvClient(Client):
    def __init__(self):
        super().__init__("agv", False)

    def Init(self):
        self._SetApiVerson("1.0.0.1")
        self._RegistApi(1001, 0)
        self._RegistApi(1002, 0)

    def Move(self, vx, vy, vyaw):
        parameter = json.dumps({"vx": vx, "vy": vy, "vyaw": vyaw})
        code, _ = self._Call(1001, parameter)
        return code

    def HeightAdjust(self, vz):
        parameter = json.dumps({"data": vz})
        code, _ = self._Call(1002, parameter)
        return code


class G1DAgvClientWrapper:
    def __init__(self):
        self.client = G1DAgvClient()
        self.client.SetTimeout(0.1)
        self.client.Init()
        self._last_warn_time = 0.0
        self._lock = threading.Lock()
        self._vx = 0.0
        self._vyaw = 0.0
        self._height_vz = 0.0
        self._worker = threading.Thread(target=self._send_loop, daemon=True)
        self._worker.start()

    def _call(self, label, fn):
        try:
            ret = fn()
        except Exception as e:
            self._warn(f"{label} exception: {e}")
            return None

        if ret != 0:
            self._warn(f"{label} returned {ret}")
        return ret

    def _warn(self, message):
        now = time.time()
        if now - self._last_warn_time > 1.0:
            logger_mp.warning(f"[G1DAgvClientWrapper] {message}")
            self._last_warn_time = now

    def Damp(self):
        self.Stop()

    def Move(self, vx, vy, vyaw):
        with self._lock:
            self._vx = vx
            self._vyaw = vyaw

    def HeightAdjust(self, vz):
        with self._lock:
            self._height_vz = vz

    def Stop(self):
        self.Move(0.0, 0.0, 0.0)
        self.HeightAdjust(0.0)

    def _send_loop(self):
        while True:
            with self._lock:
                vx = self._vx
                vyaw = self._vyaw
                height_vz = self._height_vz

            self._call("Move", lambda: self.client.Move(vx, 0.0, vyaw))
            self._call("HeightAdjust", lambda: self.client.HeightAdjust(height_vz))
            time.sleep(0.05)

if __name__ == '__main__':
    ChannelFactoryInitialize(1) # 0 for real robot, 1 for simulation
    ms = MotionSwitcher()
    status, result = ms.Enter_Debug_Mode()
    print("Enter debug mode:", status, result)
    time.sleep(5)
    status, result = ms.Exit_Debug_Mode()
    print("Exit debug mode:", status, result)
    time.sleep(2)
