
from  selfdrive.kegman_conf import kegman_conf

class AtomConf():
  def __init__(self, CP=None):
    self.kegman = kegman_conf()

    self.tun_type   = 'lqr'
    self.sR_KPH         = [30, 40, 80]   # Speed  kph
    self.sR_BPV         = [[0.],      [0.],      [0.]     ]
    self.sR_steerRatioV = [[13.95,13.85,13.95],[13.95,13.85,13.95],[13.95,13.85,13.95]]
    self.sR_ActuatorDelayV = [[0.25,0.5,0.25],[0.25,0.8,0.25],[0.25,0.8,0.25]]
    self.sR_pid_KiV     = [[0.02,0.01,0.02],[0.03,0.02,0.03],[0.03,0.02,0.03]]
    self.sR_pid_KpV     = [[0.20,0.15,0.20],[0.25,0.20,0.25],[0.25,0.20,0.25]]
    self.sR_pid_deadzone  = 0.1
    self.sR_lqr_kiV     = [[0.005],   [0.015],   [0.02]   ]
    self.sR_lqr_scaleV  = [[2000],    [1900.0],  [1850.0] ]

    self.cv_KPH    = [0.]   # Speed  kph
    self.cv_BPV    = [[150., 255.]]  # CV
    self.cv_sMaxV  = [[255., 230.]]
    self.cv_sdUPV  = [[3,3]]
    self.cv_sdDNV  = [[7,5]]

    self.steerOffset = 0.0
    self.steerRateCost = 0.4
    self.steerLimitTimer = 0.4    
    self.steerActuatorDelay = 0.1
    self.cameraOffset = 0.06
    self.ap_learner = 1
    self.ap_autoReasume = 1
    self.ap_autoScnOffTime = 0

    self.read_tune()


  def read_tune(self):
    conf = self.kegman.read_config()
    self.ap_learner = conf['ap_learner']
    self.ap_autoReasume = conf['ap_autoReasume']
    self.ap_autoScnOffTime = conf['ap_autoScnOffTime']
    self.tun_type   = conf['tun_type']
    self.sR_KPH   = conf['sR_KPH']
    self.sR_BPV  = conf['sR_BPV']
    self.sR_steerRatioV  = conf['sR_steerRatioV']
    self.sR_ActuatorDelayV = conf['sR_ActuatorDelayV']    
    self.sR_pid_KiV  = conf['sR_pid_KiV']
    self.sR_pid_KpV  = conf['sR_pid_KpV']
    self.sR_pid_deadzone  = conf['sR_pid_deadzone']
    self.sR_lqr_kiV  = conf['sR_lqr_kiV']
    self.sR_lqr_scaleV  = conf['sR_lqr_scaleV']
    self.cv_KPH  = conf['cv_KPH']
    self.cv_BPV  = conf['cv_BPV']
    self.cv_sMaxV  = conf['cv_sMaxV']
    self.cv_sdUPV  = conf['cv_sdUPV']
    self.cv_sdDNV = conf['cv_sdDNV']
    self.steerOffset = conf['steerOffset']
    self.steerRateCost = conf['steerRateCost']
    self.steerLimitTimer = conf['steerLimitTimer']
    self.cameraOffset = conf['cameraOffset']
