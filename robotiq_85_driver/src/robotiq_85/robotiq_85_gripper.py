import serial
import array

from .gripper_io import GripperIO
from .modbus_crc import verify_modbus_rtu_crc

class Robotiq85Gripper:
    def __init__(self,num_grippers=1,comport='/dev/ttyUSB0',baud=115200):
        assert num_grippers in [1, 2]
        try:
            self.ser = serial.Serial(comport,baud,timeout = 0.2)
        except:
            self.init_success = False
            return

        self._gripper = [GripperIO(i) for i in range(num_grippers)]
        self._num_grippers = num_grippers
        self.init_success = True
        self._shutdown_driver = False

    def shutdown(self):
        self._shutdown_driver = True
        self.ser.close()

    def process_cmds(self,dev=0):
        assert dev >= 0 and dev < self._num_grippers
        if self._shutdown_driver:
            print("driver is shutdown")
            return False
        try:
            self.ser.write(self._gripper[dev].act_cmd_bytes)
            rsp = self.ser.read(8)
            if (len(rsp) != 8):
                print("response wrong length")
                return False
            if not verify_modbus_rtu_crc(rsp):
                print("cannot verify modbus")
                return False

            self.ser.write(self._gripper[dev].stat_cmd_bytes)
            rsp = self.ser.read(21)
            if (len(rsp) != 21):
                print("response wrong length")
                return False
            parsed_rsp = self._gripper[dev].parse_rsp(rsp)
            return parsed_rsp
        except Exception as e:
            raise e

    def activate_gripper(self,dev=0):
        assert dev >= 0 and dev < self._num_grippers
        self._gripper[dev].activate_gripper()

    def deactivate_gripper(self,dev=0):
        assert dev >= 0 and dev < self._num_grippers
        self._gripper[dev].deactivate_gripper()

    def activate_emergency_release(self,dev=0,open_gripper=True):
        assert dev >= 0 and dev < self._num_grippers
        self._gripper[dev].activate_emergency_release(open_gripper)

    def deactivate_emergency_release(self,dev=0):
        assert dev >= 0 and dev < self._num_grippers
        self._gripper[dev].deactivate_emergency_release()

    def goto(self, dev=0, pos=0.0, vel=1.0, force=1.0):
        assert dev >= 0 and dev < self._num_grippers
        self._gripper[dev].goto(pos, vel, force)

    def stop(self,dev=0):
        assert dev >= 0 and dev < self._num_grippers
        self._gripper[dev].stop()

    def is_ready(self,dev=0):
        assert dev >= 0 and dev < self._num_grippers
        return self._gripper[dev].is_ready()

    def is_reset(self,dev=0):
        assert dev >= 0 and dev < self._num_grippers
        return self._gripper[dev].is_reset()

    def is_moving(self,dev=0):
        assert dev >= 0 and dev < self._num_grippers
        return self._gripper[dev].is_moving()

    def is_stopped(self,dev=0):
        assert dev >= 0 and dev < self._num_grippers
        return self._gripper[dev].is_moving()

    def object_detected(self,dev=0):
        assert dev >= 0 and dev < self._num_grippers
        return self._gripper[dev].object_detected()

    def get_fault_status(self,dev=0):
        assert dev >= 0 and dev < self._num_grippers
        return self._gripper[dev].get_fault_status()

    def get_pos(self,dev=0):
        assert dev >= 0 and dev < self._num_grippers
        return self._gripper[dev].get_pos()

    def get_req_pos(self,dev=0):
        assert dev >= 0 and dev < self._num_grippers
        return self._gripper[dev].get_req_pos()

    def get_current(self,dev=0):
        assert dev >= 0 and dev < self._num_grippers
        return self._gripper[dev].get_current()
