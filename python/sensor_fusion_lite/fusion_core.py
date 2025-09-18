from logging import Filter
import time
from enum import Enum
from typing import List, Callable, Optional

class Quanternion:
  def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
    self.w = w
    self.x = x
    self.y = y
    self.z = z
    
class State:
  def __init__(self, dim: int = 6):
    self.position = [0.0, 0.0, 0.0]
    self.velocity = [0.0, 0.0, 0.0]
    self.orientation = Quanternion()
    self.covariance = [[0.0 for _ in range(dim)] for _ in range(dim)]
    self.timestamp = time.time()
    
class ImuMeasurement:
  def __init__(self):
    self.linear_accel = [0.0, 0.0, 0.0]
    self.angular_vel = [0.0, 0.0, 0.0]
    self.orientation: Optional[Quanternion] = None
    self.timestamp = time.time()
    
class OdomMeasurement:
  def __init__(self):
    self.position = [0.0, 0.0, 0.0]
    self.linear_velocity = [0.0, 0.0, 0.0]
    self.orientation = Quanternion()
    self.cov_diag = [0.0] * 6
    self.timestamp = time.time()
    
class GpsMeasurement:
  def __init__(self):
    self.position = [0.0, 0.0, 0.0]
    self.cov = [[0.0]*3 for _ in range(3)]
    self.timestamp = time.time()
    
class PoseMeasurement:
  def __init__(self):
    self.position = [0.0, 0.0, 0.0]
    self.orientation = Quanternion()
    self.cov_diag = [0.0] * 6
    self.timestamp = time.time()
    
class FilterType(Enum):
  COMPLEMENTARY = 1
  EKF = 2
  UKF = 3
  
class FusionCore:
  def __init__(self, filter_type=FilterType.COMPLEMENTARY, state_dim=6, initial_state: Optional[State] = None):
    self.filter_type = filter_type
    self.state_dim = state_dim
    self.state = initial_state if initial_state else State(state_dim)
    self.state_callbacks: dict[int, Callable[[State], None]] = {}
    self.diagnostic_callback: Optional[Callable[[str], None]] = None
    self.next_cb_id = 0
    
  def initialize(self):
    self.state.covariance = [[0.0 for _ in range(self.state_dim)] for _ in range(self.state_dim)]
    for i in range(self.state_dim):
      self.state.covariance[i][i] = 1.0
      
  def start(self): pass
  def stop(self): pass
  
  def set_filter_type(self, t: FilterType): self.filter_type = t
  def get_filter_type(self) -> FilterType: return self.filter_type
  
  def set_imu_noise(self, accel_cov, gyro_cov): pass
  def set_odom_covariance_diag(self, diag): pass
  def set_gps_covariance(self, cov): pass
  def set_pose_cov_diag(self, diag): pass
  
  def enable_sensor(self, sensor_name: str, enabled: bool): pass
  
  def predict(self, dt: float) -> bool: return True
  def update_imu(self, imu: ImuMeasurement) -> bool: return True
  def update_odom(self, odom: OdomMeasurement) -> bool: return True
  def update_gps(self, gps: GpsMeasurement) -> bool: return True
  def update_pose(self, pose: PoseMeasurement) -> bool: return True
  
  def update_custom(self, H, z, R, timestamp): return True
  
  def get_state(self) -> State: return self.state
  def get_covariance(self) -> List[List[float]]: return self.state.covariance
  def set_state(self, s: State): self.state = s
  
  def register_state_callback(self, cb: Callable[[State], None]) -> int:
    cid = self.next_cb_id
    self.state_callbacks[cid] = cb
    self.next_cb_id += 1
    return cid
  
  def unregister_state_callback(self, cid: int):
    if cid in self.state_callbacks: del self.state_callbacks[cid]
    
  def set_diagnostic_callback(self, cb: Callable[[str], None]):
    self.diagnostic_callback = cb    