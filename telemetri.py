import math
from pymavlink import mavutil


class Telemetri:
    paket_no = 0

    def __init__(self, iha):
        self.global_position = iha.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
        self.sys_status = iha.recv_match(type='SYS_STATUS', blocking=True).to_dict()
        self.pitot = iha.recv_match(type='VFR_HUD', blocking=True)
        self.signals = iha.recv_match(type="RC_CHANNELS", blocking=True)
        self.scaled_pressure = iha.recv_match(type='SCALED_PRESSURE', blocking=True).to_dict()
        self.gps = iha.recv_match(type="GPS_RAW_INT", blocking=True).to_dict()
        self.terrain_report = iha.recv_match(type="TERRAIN_REPORT", blocking=True).to_dict()
        self.mode = iha.recv_match(type="HEARTBEAT", blocking=True)
        self.attitude = iha.recv_match(type='ATTITUDE', blocking=True).to_dict()



        self.alt = self.global_position["alt"] / 1e7
        self.relative_alt = self.global_position["relative_alt"] / 1000
        self.lat = self.global_position["lat"] / 1e7
        self.long = self.global_position["lon"] / 1e7
        self.mod = mavutil.mode_string_v10(self.mode)  # round(self.scaled_pressure["press_abs"], 2)
        self.hava_hizi = self.pitot.airspeed
        self.roll = self.attitude["roll"] * 180 / math.pi
        self.pitch = self.attitude["pitch"] * 180 / math.pi
        self.roll_speed = self.attitude["rollspeed"] * 180 / math.pi
        self.yaw = self.pitot.heading
        self.sinyal = self.signals.rssi
        self.satellites_visible = self.gps['satellites_visible']
        self.baseMod = self.mode.base_mode
        Telemetri.paket_no += 1

    def __str__(self):
        return (f"{self.alt}, {self.lat}, {self.long}, {self.irtifa_farki}, {self.basinc}, {self.sicaklik}, "
                f"{self.pil_gerilimi}, {self.roll}, {self.pitch}, {self.roll_speed}")


