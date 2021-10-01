
package frc.robot;

import edu.wpi.first.wpilibj.Timer;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class
 * (or one of its inner classes) wherever the constants are needed, to reduce
 * verbosity.
 */
public final class Constants {

	public enum LimelightValue {
		Default(0),
		Short(1),
		Medium(2),
		Long(3);

		private final int _pipeline;

		LimelightValue (int pipeline) {
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

	// simple methods used for a lot of things

	public static double deadband(double input, double limit) {
		return (Math.abs(input) <= limit ? 0 : input);
	}

	public static double deadband(double input, double center, double limit) {
		return (Math.abs(input-center) <= limit ? 0 : input);
	}

	public static double clamp(double input, double limit) {
		return (Math.abs(input) <= limit ? input : Math.copySign(limit, input));
	}

	public static double clamp(double input, double center, double limit) {
		return (Math.abs(input-center) <= limit ? input-center : Math.copySign(limit, input));
	}

	public static double square(double input) {
		return Math.copySign(Math.pow(input,2), input);
	}

	// time methods

	public static double getTime() {
		return Timer.getFPGATimestamp();
	}

	public static double getTimeSince(double value) {
		return Timer.getFPGATimestamp()-value;
	}

	public static double getMatchTime() {
		return Timer.getMatchTime();
	} // only works in an actual match

	public static final class Config {

		/**
		 * Set to zero to skip waiting for confirmation, set to nonzero to wait
		 * and report to the DS if an action fails.
		 */
		public static final int kTimeoutMs = 30;
		
		/**
		 * Default deadband for controller inputs.
		 */
		public static final double kDefaultDeadband = .1;

		/**
		 * Default and Max Feed Forwards for various motors as decimal percentages.
		 * FF Stands for FeedForward
		 */
		public static final double kDefaultIntakeFF = .6;
		public static final double kDefaultAgitatorFF = .5; // originally .35
		public static final double kDefaultEggBeaterFF = .3; // tentative .3
		public static final double kDefaultTowerPrimeFF = .18;
		public static final double kDefaultTowerFeedFF = .3; // .45
		public static final double kMaxClimbFF = .5;
		public static final double kDefaultWheelFF = .70;

		// threshold to consider shooter motor up to speed as a decimal percentage
		public static final double kUpToSpeedThreshold = .985; // the ivory soap constant

		// Motion Magic constants

		public static final int kDriveTrainMaxAcceleration = 12000;
		public static final int kDriveTrainCruiseVelocity = 12000;

		public static final int kClimberMaxAcceleration = 15000;
		public static final int kClimberCruiseVelocity = 15000;

		// limelight screen sizes

		public static final double screenSizeX = 27.0;
		public static final double screenSizeY = 20.5;

		// limelight tracking offset
		public static final double aimOffsetX = 0;
		// turret turning friction constant
		public static final double turretTurnFrictionConstant = 0.06;

	}

	public static final class MotorConstants {

		public static final double kFalconTicks = 2048;

		public static final double kTalonTicks = 4096;
		
		public static final double kTurretGearRatio = 16.0/168.0;

		public static final double kDriveTrainGearRatio = 11.25;

		public static final double kMetersPerMotorRotation = 0.0398288604;

		public static final double kMetersPerTick = kMetersPerMotorRotation / kFalconTicks;

	}

	public static final class PID {

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
		public final static Gains kGainsLeft = new Gains(
			0.1, // kP 0.1
			0.0, // kI
			0.0, // kD
			1023.0/20300.0, // kF | 1023 / Max tickspeed
			300, // Iz
			1.00); // PeakOut

		public final static Gains kGainsRight = new Gains(
			0.1, // kP 0.1
			0.0, // kI
			0.0, // kD
			1023.0/20300.0, // kF | 1023 / Max tickspeed
			300, // Iz
			1.00 // PeakOut
			);			
		
		public final static Gains kGainsShooter = new Gains(
			0.3, // kP
			0.0, // kI
			0.0, // kD
			1023.0/20300.0, // kF | 1023 / Max tickspeed
			300, // Iz
			1.00 // PeakOut
		);

		public static final Gains kGainsTurret = new Gains(
			0.1, // kP
			0.0, // kI
			0.0, // kD
			1023.0/20300.0, // kF | 1023 / Max tickspeed
			300, // Iz
			1.00 // PeakOut
		);

		public static final Gains kGainsClimber = new Gains(
			0.1, // kP
			0.0, // kI
			0.0, // kD
			1023.0/20300.0, // kF | 1023 / Max tickspeed
			300, // Iz
			1.00 // PeakOut
		);
	}

	/** IDS */

	public final static class IDs {

//
//		// IDs.FALCON.LEFT_FRONT
//		public final static class DRIVE_BASE {
//			// public static final int LEFT_FRONT_FALCON = 2;
//			// public static final int LEFT_BACK_FALCON = 0;
//			// public static final int RIGHT_FRONT_FALCON = 3;
//			// public static final int RIGHT_BACK_FALCON = 1;
//		}

		/** Drivetrain Falcons */

		public static final int LEFT_FRONT_FALCON = 2;
		public static final int LEFT_BACK_FALCON = 0;
		public static final int RIGHT_FRONT_FALCON = 3;
		public static final int RIGHT_BACK_FALCON = 1;

		/** System Falcons */

		public static final int SHOOTER_FALCON = 4;

		public static final int CLIMBER_FALCON = 5;

		/** System Talons */

//		public static final int WHEEL_TALON = 8;

		public static final int EGG_TALON = 8;

		public static final int TURRET_TALON = 7;

		public static final int TOWER_TALON = 6;

		/** System Victors */

		public static final int AGITATOR_VICTOR = 1;

		public static final int INTAKE_VICTOR = 0;

		/** Pneumatics */

		public static final int INTAKE_SOLENOID_1 = 0;
		public static final int INTAKE_SOLENOID_2 = 1;

		public static final int CLIMBER_SOLENOID = 2;

		/** Other */

		public static final int GYRO_PIGEON = 0;

		public static final int LIDAR_DIO = 0;

		public static final int TOWER_BEAM_BREAK_A_DIO = 1;

		public static final int TOWER_BEAM_BREAK_B_DIO = 3;

	}

}
