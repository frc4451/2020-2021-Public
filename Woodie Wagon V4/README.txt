Motion Magic setpoints:

	Climber:
		Normal Climb:
			-135467


Autonomous 2 backup:
	-60000 units
turn:
	12000

Autonomous 4:
backup:
back up:
	78000
		85000 custom
turn:
	20000 units
backup:
	-87000 units

Palmetto shot speeds:
	Low: 8500
	Mid: 9600
	High: 11950

Testing shot speeds:
	Low: 8750
	Mid: 9777
	High: 11950

5 Meter forward testing:
    trial 1: 257015
    trial 2: ~257300 (overshot)
    trial 3 (backwards): ~-247600 (500 less)
    trial 4: ~257200

    consensus: 257100 = 5 meters

Thursday motor rotations to meters calculations

    25.107421875 motor rotations per meter
    0.0398288604 meters per motor rotation

    characterization gave about a factor of 4 times as much distance

    divided by 4 constant:
    0.0099572151

    turns out it is a bug with characterization
    decided to use the actual constant and just multiply encoder counts by 4

    encoder counts per motor rev: 2048
    encoder edges per motor rev: 8192

    Characterization Config File:

   {
       # Ports for motors
       # If doing drive test, treat this as the left side of the drivetrain
       "motorPorts": [1, 3],

       # Only if you are doing drive (leave empty "[]" if not)
       "rightMotorPorts": [0, 2],

       # Class names of motor controllers used.
       # 'WPI_TalonSRX'
       # 'WPI_VictorSPX'
       # 'WPI_TalonFX'
       # If doing drive test, treat this as the left side of the drivetrain
       "controllerTypes": ['WPI_TalonFX', 'WPI_TalonFX'],

       # Only if you are doing drive (leave empty "[]" if not)
       "rightControllerTypes": ['WPI_TalonFX', 'WPI_TalonFX'],

       # Set motors to inverted or not
       # If doing drive test, treat this as the left side of the drivetrain
       "motorsInverted": ['True', 'True'],

       # Only if you are doing drive (leave empty "[]" if not)
       "rightMotorsInverted": ['True', 'True'],

       # Encoder edges-per-revolution (*NOT* cycles per revolution!)
       # For the CTRE Mag Encoder, use 16384 (4 * 4096 = 16384)
       "encoderEPR": 8192,

       # Gearing accounts for the gearing between the encoder and the output shaft
       "gearing": 1,

       # Encoder ports (leave empty "[]" if not needed)
       # Specifying encoder ports indicates you want to use Rio-side encoders
       # If doing drive test, treat this as the left side of the drivetrain
       "encoderPorts": [],

       # Only if you are doing drive (leave empty "[]" if not)
       "rightEncoderPorts": [],

       # Set to True if encoders need to be inverted
       # If doing drive test, treat this as the left side of the drivetrain
       "encoderInverted": 'False',

       # Only if you are doing drive (set to False if not needed)
       "rightEncoderInverted": 'False',

       # ** The following is only if you are using a gyro for the DriveTrain test**
       # Gyro type (one of "NavX", "Pigeon", "ADXRS450", "AnalogGyro", or "None")
       "gyroType": "Pigeon",

       # Whatever you put into the constructor of your gyro
       # Could be:
       # "SPI.Port.kMXP" (MXP SPI port for NavX or ADXRS450),
       # "SerialPort.Port.kMXP" (MXP Serial port for NavX),
       # "I2C.Port.kOnboard" (Onboard I2C port for NavX),
       # "0" (Pigeon CAN ID or AnalogGyro channel),
       # "new WPI_TalonSRX(3)" (Pigeon on a Talon SRX),
       # "" (NavX using default SPI, ADXRS450 using onboard CS0, or no gyro)
       "gyroPort": "0",
   }







