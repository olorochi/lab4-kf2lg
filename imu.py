import time
from threading import Thread, Event, Lock
from icm20948 import ICM20948
from param import ICM20948_DT

class IMU:
    def __init__(self, imu: ICM20948):
        self.imu = imu
        self.quit = Event()
        self.lock = Lock()
        self.thread = Thread(target=self.poll_thr, daemon=True, args=(imu, self.quit))
        self.thread.start()

        accel_data = None
        magne_data = None

    def __del__(self):
        self.quit.set()
        self.thread.join()

    def obtenir_déplacement():
        with self.lock:
            ax, ay, az, gx, gy, gz = self.accel_data

    def obtenir_cap_magnétique():
        with self.lock:
            mx, my, mz = self.magne_data

    def poll_thr(self):
        start = time.monotonic()
        last = 0
        try:
            while not self.quit.is_set():
                with self.lock:
                    activation = time.monotonic() - start
                    self.accel_data = self.imu.read_accelerometer_gyro_data()
                    self.magne_data = self.imu.read_magnetometer_data()
                    t = time.monotonic() - start
                    dt = t - last
                    last = t
                    
                    time.sleep(ICM20948_DT - (t - activation))

        except (Exception) as error:
            print(error)

        print("poll_thr exit")


if __name__ == "__main__":
    imu = IMU(ICM20948())
    time.sleep(5)
    del(imu)
