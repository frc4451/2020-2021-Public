package frc.robot;

public class Constants {


	// Config is general code settings
	public static final class Config {

		// This sets debug messages to run or not. Recommend turning OFF for performance situations (competitions)
		public static final boolean kDebugEnabled = false; //TODO

		/**
		 * Set to zero to skip waiting for confirmation, set to nonzero to wait
		 * and report to the DS if an action fails.
		 */
		public static final int DEF_TIMEOUT_MS = 30;

		/**
		 * Default deadband for controller inputs.
		 */
		public static final double DEF_DEADBAND = .1;


	}

	// Settings is specific
	public static final class Settings {

		/**
		 * Default and Max Feed Forwards for various motors as decimal percentages.
		 * FF Stands for FeedForward
		 */
		public static final double DEF_INTAKE_FF = .8;
		public static final double DEF_AGITATOR_FF = .5;
		public static final double DEF_EGGBEATER_FF = .3;
		public static final double DEF_TOWER_PRIME_FF = .18;
		public static final double DEF_TOWER_SHOOT_FF = .3;
		public static final double MAX_CLIMB_FF = .5;
		public static final double DEF_WHEEL_FF = .7;
		public static final double MAX_TURRET_FF = -.5;

		// threshold to consider shooter motor up to speed (UTS) as a decimal percentage
		// used to consider whether safe shoot should start feeding
		public static final double SHOOTER_UTS_THRESH = .985; // *the ivory grease constant*

		// Motion Magic constants, ACC=Acceleration, VEL=Velocity, DT=DriveTrain

		public static final int DT_MAX_ACC = 12000;
		public static final int DT_CRUISE_VEL = 12000;

		public static final int CLIMBER_MAX_ACC = 15000;
		public static final int CLIMBER_CRUISE_VEL = 15000;

		// turret turning friction constant
		public static final double TURRET_TURN_FRICT_CONST = 0.06;

		// these are fixed values for limelight Field of View screen size
		public static final double SCREEN_SIZE_X = 27.0;
		public static final double SCREEN_SIZE_Y = 20.5;
	}

	public static final class IDs {

		// Talon FX
		public static final class FALCON {

			// Drivetrain
			public static final int LEFT_BACK_DRIVETRAIN = 0;
			public static final int RIGHT_BACK_DRIVETRAIN = 1;
			public static final int LEFT_FRONT_DRIVETRAIN = 2;
			public static final int RIGHT_FRONT_DRIVETRAIN = 3;

			// Other Falcons

			public static final int SHOOTER = 4;

			public static final int CLIMBER = 5;
		}

		// Talon SRX
		public static final class TALON {

			public static final int TOWER_BELT = 6;

			public static final int TURRET_YAW = 7;

			public static final int EGG_BEATER = 8;

		}

		// Victor SPX
		public static final class VICTOR {

			public static final int INTAKE = 0;

			public static final int AGITATOR = 1;

			public static final int COLOR_WHEEL = 2;

		}

		/** Pneumatics */

		public static final int INTAKE_SOLENOID_A = 0;
		public static final int INTAKE_SOLENOID_B = 1;

		public static final int CLIMBER_SOLENOID = 2;

		/** Other */

		//Pigeon IMU Gyro
		public static final int GYRO_PIGEON = 0;

		// NT stands for Network Table / Tables
		public static final String LIMELIGHT_NT_KEY = "limelight";

		// Status light solenoid
		public static final int STATUS_LIGHT = 3;

		// DIO Ports

//		public static final int LIDAR = 0;

		public static final int TOWER_BEAM_BREAK_TOP = 1;

		// DIO 2 is Top Beam Break second part

		public static final int TOWER_BEAM_BREAK_BOTTOM = 3;

		// DIO 4 is Bottom Beam Break second part

	}

	public static final class PIDs {

		/**
		 * Which PID slot to pull gains from. Starting 2018, you can choose from
		 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
		 * configuration.
		 */
		public static final int kSlotIdx = 0;

		/**
		 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops.
		 * For now we just want the primary one.
		 */
		public static final int kPIDLoopIdx = 0;

		/**
		 * PID Gains may have to be adjusted based on the responsiveness of
		 * control loop.
		 * kF: 1023 represents output value to Talon at 100%
		 * 7200 represents Velocity units at 100% output
		 * */
		public static final Gains kGainsLeft = new Gains(
				.1, // kP, from characterization, probably not 2.29
				0.0, // kI
				0.0, // kD
				1023.0/20300.0, // kF | 1023 / Max tickspeed
				0.739, // static
				2.5, // velocity
				0.278, // acceleration
				300, // Iz
				1.00); // PeakOut

		public static final Gains kGainsRight = new Gains(
				.1, // kP, from characterization, probably not 2.29
				0.0, // kI
				0.0, // kD
				1023.0/20300.0, // kF | 1023 / Max tickspeed
				0.739, // static
				2.5, // velocity
				0.278, // acceleration
				300, // Iz
				1.00 // PeakOut
		);

		public static final Gains kGainsShooter = new Gains(
				0.3, // kP
				0.0, // kI
				0.0, // kD
				1023.0/20300.0, // kF | 1023 / Max tickspeed
				0, // not used
				0, // not used
				0, // not used
				300, // Iz
				1.00 // PeakOut
		);

		public static final Gains kGainsTurret = new Gains(
				0.1, // kP
				0.0, // kI
				0.0, // kD
				1023.0/20300.0, // kF | 1023 / Max tickspeed
				0, // not used
				0, // not used
				0, // not used
				300, // Iz
				1.00 // PeakOut
		);

		public static final Gains kGainsClimber = new Gains(
				0.1, // kP
				0.0, // kI
				0.0, // kD
				1023.0/20300.0, // kF | 1023 / Max tickspeed
				0, // not used
				0, // not used
				0, // not used
				300, // Iz
				1.00 // PeakOut
		);

	}

	// Motor/drivetrain specs, gear ratios
	public static final class TechnicalConstants {

		public static final double kFalconTicks = 2048;

		public static final double kTalonTicks = 4096;

		public static final double kTurretGearRatio = 16.0/168.0;

		public static final double kDriveTrainGearRatio = 11.25;

		public static final double kDriveTrainWidth = .8; // meters, calculated trackwidth manually is .62 meter

		public static final double kMetersPerMotorRotation = 0.0398288604;

		public static final double kMetersPerTick = kMetersPerMotorRotation / kFalconTicks;

		public static final double kMeterPerSecondsPerTicksPerDeciSecond =
				kMetersPerMotorRotation / kFalconTicks * 10;

	}

	// Enums

	public enum GameState {
		Disabled(0),
		Autonomous(1),
		TeleOp(2),
		Test(3);

		private final int _gameState;

		GameState(int gameState) {
			this._gameState = gameState;
		}

		public int val() {
			return this._gameState;
		}

		public String string() {
			switch (this._gameState) { // don't need breaks when you're returning!
				case 0:
					return("Disabled");
				case 1:
					return("Autonomous");
				case 2:
					return("TeleOp");
				case 3:
					return("Test");
				default:
					return("ERROR");
			}
		}
	}

	public enum LimelightPipeline {
		Default(0), // turns light off
		Short(1), // not used (right now)
		Medium(2), // not used (right now)
		Long(3), // not used (right now)
		Auto(4), // not used (right now)
		NormalAim(9);

		private final int _pipeline;

		LimelightPipeline(int pipeline) {
			this._pipeline = pipeline;
		}

		public int val() {
			return this._pipeline;
		}
	}

	public enum SolenoidState {
		Default(false),
		Inactive(false),
		Active(true),
		BrakeEngaged(false),
		BrakeDisengaged(true);

		private final boolean _state;

		SolenoidState (boolean state) {
			this._state = state;
		}

		public boolean val() {
			return this._state;
		}
	}

}
