# Defines the control commands sent to the autopilot block

float64 normalized_roll		# nomalized roll, roll rate, or aleraon deflection (-1 to 1)
float64 normalized_pitch	# nomalized pitch, pitch rate or elevator deflection (-1 to 1)
float64 normalized_yaw		# nomalized yaw rate or rudder deflection (-1 to 1)
float64 normalized_throttle	# nomalized throttle (0 to 1)
uint8 MODE_ACRO=0
uint8 MODE_ANGLE=1
uint8 MODE_FIXED_WING=2
uint8 mode
