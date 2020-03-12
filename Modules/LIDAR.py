import time
import melopero_vl53l1x as mp


def Lidar(mode):
     Lidarsensor = mp.VL53L1X()
     if mode == 0:
          Lidarsensor.start_ranging(mp.VL53L1X.SHORT_DST_MODE)
     elif mode == 1:
          Lidarsensor.start_ranging(mp.VL53L1X.MEDIUM_DST_MODE)
     elif mode == 2:
          Lidarsensor.start_ranging(mp.VL53L1X.LONG_DST_MODE)
     TOF = Lidarsensor.get_measurement()
     TOF = TOF/10
     Lidarsensor.stop_ranging()
     Lidarsensor.close_connection()
     print("Lidar")
     return TOF