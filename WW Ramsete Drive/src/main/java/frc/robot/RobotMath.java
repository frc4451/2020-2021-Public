package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class RobotMath {

	public static final class IDs {

		/*
		 * F = Falcon
		 * T = Talon
		 * V = Victor
		 * P = Pigeon
		 * S = Solenoid
		 * D = DIO (Digital I/O)
		 */

		// DriveBase

		// quadrants are when robot facing positive x
		public static final int kDrivetrainLeftFrontF = 2; // quadrant 1
		public static final int kDrivetrainLeftBackF = 0; // quadrant 2
		public static final int kDrivetrainRightFrontF = 3; // quadrant 4
		public static final int kDrivetrainRightBackF = 1; // quadrant 3

		// gyro
		public static final int kGyroP = 0;

		// Intake

		public static final int kIntakeV = 0; // victor

		// Solenoids

		public static final int kIntakeS0 = 0; // double solenoid
		public static final int kIntakeS1 = 1;

		// Jack stand
		public static final int kJackS = 2; // solenoid

		// Indicator light
		public static final int kStatusL = 3;

		// from base code

		/** System Falcons */

		public static final int SHOOTER_FALCON = 4;

		/** System Talons */

		public static final int TURRET_TALON = 7;

		public static final int TOWER_TALON = 6;

		public static final int EGG_TALON = 8;

		/** System Victors */

		public static final int AGITATOR_VICTOR = 1;

		/** Pneumatics */

		/** Other */

		public static final int TOWER_BEAM_BREAK_A_DIO = 1;

		public static final int TOWER_BEAM_BREAK_B_DIO = 3;

	}

	public static final class Constants {

		public static final double kFalconTicks = 2048;

		public static final double kTalonTicks = 4096;

		public static final double kTurretGearRatio = 16.0/168.0;

		public static final double kDriveTrainGearRatio = 11.25;

		public static final double kDriveTrainWidth = .8; // calculated trackwidth manually is .62

		public static final double kMetersPerMotorRotation = 0.0398288604;

		public static final double kMetersPerTick = kMetersPerMotorRotation / kFalconTicks;

		public static final double kMeterPerSecondsPerTicksPerDeciSecond =
				kMetersPerMotorRotation / kFalconTicks * 10;

		// constants from Base code

		public static final int kTimeoutMs = 30;
		public static final double kDefaultDeadband = .1;

		public static final double kDefaultIntakeFF = .6;
		public static final double kDefaultAgitatorFF = .5; // originally .35
		public static final double kDefaultEggBeaterFF = .3; // tentative .3
		public static final double kDefaultTowerPrimeFF = .18;
		public static final double kDefaultTowerFeedFF = .3; // .45
		public static final double kMaxClimbFF = .5;
		public static final double kDefaultWheelFF = .70;

		public static final double kUpToSpeedThreshold = .98;
		public static final double screenSizeX = 27.0;
		public static final double screenSizeY = 20.5;
//		public static final double aimOffsetX = 0; // not being used
		public static final double turretTurnFrictionConstant = 0.1; // 0.06

		public static final int kDriveTrainMaxAcceleration = 12000;
		public static final int kDriveTrainCruiseVelocity = 12000;

		public static final int kClimberMaxAcceleration = 15000;
		public static final int kClimberCruiseVelocity = 15000;
	}

	// General math functions for controls

	public static int kTimeoutMs = 30;

	public static double kDefaultDeadband = .1;

	public static double deadBand(double input, double limit) {
		return (Math.abs(input) <= limit ? 0 : input);
	}

	public static boolean inBand(double input, double center, double range) {
		return Math.abs(input-center) <= range;
	}

	public static double smoothBand(double input, double center, double range) {
		return Math.pow(2, -((input-center)/range)*((input-center)/range));
		// power 2 bell curve that is 1 when exactly center and .5 when on border of range
	}

	public static double square(double input) {
		return Math.copySign(Math.pow(input,2), input);
	}

	public static double pythag(double _a, double _b) {
		return Math.pow(Math.pow(_a,2)+Math.pow(_b,2),.5);
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

	public enum LimelightValue {
		Default(0),
		Auto(4),
		TeleOp(9);

		private final int _pipeline;

		LimelightValue(int pipeline) {
			this._pipeline = pipeline;
		}

		public int val() {
			return this._pipeline;
		}
	}

	// PID Gains

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
	}

	public static final class limelightCoord {
		public final double tX;
		public final double tY;
		public final double tA;

		public limelightCoord(double _tX, double _tY, double _tA) {
			this.tX = _tX;
			this.tY = _tY;
			this.tA = _tA;
		}

		public static limelightCoord toCoord(double _tX, double _tY, double _tA) {
			return new limelightCoord(_tX, _tY, _tA);
		}

		public double match(limelightCoord input, limelightCoord _range) {
			return smoothBand(
					0,
					pythag(
							pythag(
									(input.tX-this.tX)/_range.tX,
									// dividing by range to squish the space into a unit circle where range is 1
									(input.tY-this.tY)/_range.tY
							), (input.tA-this.tA)/_range.tA // nested pythag for 3d space (3 variables)
					),
					1
			);
		}

		public boolean squareMatch(limelightCoord input, limelightCoord _range) {
			return inBand(this.tX, input.tX, _range.tX)
					&& inBand(this.tY, input.tY, _range.tY)
					&& inBand(this.tA, input.tA, _range.tA);
		}
	}

	// hardcoded limelight ball target coordinates for different field configurations, from previous

	public static final limelightCoord kRedATarget = new limelightCoord(22.2, 0.9, 1.37);
	public static final limelightCoord kRedBTarget = new limelightCoord(-5.3, -1.5, 1.32);
	public static final limelightCoord kBlueATarget = new limelightCoord(26.0, 20.0, 0.22);
	public static final limelightCoord kBlueBTarget = new limelightCoord(16.5, 19.8, 0.21);

	public static final limelightCoord kBallTargetRange = new limelightCoord(2, 2, 0.5);
}
