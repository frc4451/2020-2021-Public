package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static frc.robot.Debug.debug;
import static frc.robot.RobotContainer.*;
import static frc.robot.RobotMath.*;

public class DriveTrain extends SubsystemBase {

	/* =================================================================================================================
	 * Internal Objects
	 */

	// physical objects
	private final TalonFX kFalconLeftFront;
	private final TalonFX kFalconLeftBack;
	private final TalonFX kFalconRightFront;
	private final TalonFX kFalconRightBack;

	private final PigeonIMU kPigeon;

	// ramsete conversions
	private final DifferentialDriveOdometry kOdometry;
	private final DifferentialDriveKinematics kKinematics;

	// speed controllers
	private final PIDController kLeftPIDController = new PIDController(
			PIDs.kGainsLeft.kP,
			PIDs.kGainsLeft.kI,
			PIDs.kGainsLeft.kD);

	private final PIDController kRightPIDController = new PIDController(
			PIDs.kGainsRight.kP,
			PIDs.kGainsRight.kI,
			PIDs.kGainsRight.kD);

	private final SimpleMotorFeedforward kLeftFeedForward = new SimpleMotorFeedforward(
			PIDs.kGainsLeft.kS,
			PIDs.kGainsLeft.kV,
			PIDs.kGainsLeft.kA
	);

	private final SimpleMotorFeedforward kRightFeedForward = new SimpleMotorFeedforward(
			PIDs.kGainsRight.kS,
			PIDs.kGainsRight.kV,
			PIDs.kGainsRight.kA
	);

	/* =================================================================================================================
	 * Constructor
	 */

	public DriveTrain(
			TalonFX _kFalconLeftBack,
			TalonFX _kFalconRightBack,
			TalonFX _kFalconLeftFront,
			TalonFX _kFalconRightFront,
			PigeonIMU _kPigeon,
			DifferentialDriveOdometry odometry,
			DifferentialDriveKinematics kinematics
	) {

		this.kFalconLeftFront = _kFalconLeftFront; // setting local variable to passed in objects
		this.kFalconLeftBack = _kFalconLeftBack;
		this.kFalconRightFront = _kFalconRightFront;
		this.kFalconRightBack = _kFalconRightBack;

		this.kPigeon = _kPigeon;

		this.kOdometry = odometry;
		this.kKinematics = kinematics;

		kFalconLeftFront.configFactoryDefault(); // reset all existing config
		kFalconLeftBack.configFactoryDefault();
		kFalconRightFront.configFactoryDefault();
		kFalconRightBack.configFactoryDefault();

		kFalconLeftBack.follow(kFalconLeftFront); // slave back to front motors
		kFalconRightBack.follow(kFalconRightFront);

		kFalconLeftFront.setSensorPhase(false); // set sides sensor phase
		kFalconRightFront.setSensorPhase(false);

		kFalconLeftFront.setInverted(false); // invert right side
		kFalconLeftBack.setInverted(false);
		kFalconRightFront.setInverted(true);
		kFalconRightBack.setInverted(true);

		/* Config sensor used for Primary PID [Velocity] */
		kFalconLeftFront.configSelectedFeedbackSensor(
				FeedbackDevice.IntegratedSensor,
				PIDs.kPIDLoopIdx,
				Config.DEF_TIMEOUT_MS);


		kFalconRightFront.configSelectedFeedbackSensor(
				FeedbackDevice.IntegratedSensor,
				PIDs.kPIDLoopIdx,
				Config.DEF_TIMEOUT_MS);

		/*
		  Phase sensor accordingly.
		 Positive Sensor Reading should match Green (blinking) Leds on Talon
		*/
		kFalconLeftFront.setSensorPhase(true);
		kFalconRightFront.setSensorPhase(false);

		/* Config the peak and nominal outputs */
		kFalconLeftFront.configNominalOutputForward(0, Config.DEF_TIMEOUT_MS);
		kFalconLeftFront.configNominalOutputReverse(0, Config.DEF_TIMEOUT_MS);
		kFalconLeftFront.configPeakOutputForward(1, Config.DEF_TIMEOUT_MS);
		kFalconLeftFront.configPeakOutputReverse(-1, Config.DEF_TIMEOUT_MS);

		kFalconRightFront.configNominalOutputForward(0, Config.DEF_TIMEOUT_MS);
		kFalconRightFront.configNominalOutputReverse(0, Config.DEF_TIMEOUT_MS);
		kFalconRightFront.configPeakOutputForward(1, Config.DEF_TIMEOUT_MS);
		kFalconRightFront.configPeakOutputReverse(-1, Config.DEF_TIMEOUT_MS);

		kFalconLeftFront.configMotionAcceleration(Settings.DT_MAX_ACC);
		kFalconLeftFront.configMotionCruiseVelocity(Settings.DT_CRUISE_VEL);

		kFalconRightFront.configMotionAcceleration(Settings.DT_MAX_ACC);
		kFalconRightFront.configMotionCruiseVelocity(Settings.DT_CRUISE_VEL);

		/* Config the Velocity closed loop gains in slot0 */

		kFalconLeftFront.config_kP(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsLeft.kP,
				Config.DEF_TIMEOUT_MS
		);
		kFalconLeftFront.config_kI(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsLeft.kI,
				Config.DEF_TIMEOUT_MS
		);
		kFalconLeftFront.config_kD(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsLeft.kD,
				Config.DEF_TIMEOUT_MS
		);
		kFalconLeftFront.config_kF(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsLeft.kF,
				Config.DEF_TIMEOUT_MS
		);

		kFalconRightFront.config_kP(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsRight.kP,
				Config.DEF_TIMEOUT_MS
		);
		kFalconRightFront.config_kI(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsRight.kI,
				Config.DEF_TIMEOUT_MS
		);
		kFalconRightFront.config_kD(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsRight.kD,
				Config.DEF_TIMEOUT_MS
		);
		kFalconRightFront.config_kF(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsRight.kF,
				Config.DEF_TIMEOUT_MS
		);

		kFalconLeftFront.configClosedloopRamp(.75);
		kFalconRightFront.configClosedloopRamp(.75);

		resetNavigation();

		resetOdometry();


	}

	public void enableInit() {
		resetNavigation();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	/* =================================================================================================================
	 * DRIVE METHODS
	 */

	public void runDriveTrain() {

		// reset encoders tool
		if (IO.Driver.lCenter()) {
			resetEncoders();
		}

		/* Teleop Drive with Velocity control */
		//get forward intention from driver left stick Y direction
		double driverForward = square(deadBand(
				-IO.Driver.leftY(),
				Config.DEF_DEADBAND
		));
		//get rotation intention from driver right stick X direction
		double driverTurn = square(deadBand(
				IO.Driver.rightX(),
				Config.DEF_DEADBAND
		));

		if (IO.Driver.lBump()) {
			velocityDrive(driverForward, -.5);
		} else if (IO.Driver.rBump()) {
			velocityDrive(driverForward, .5);
		} else {
			if (!IO.Driver.lCenter()) { // if not disabled by left center button tool (this is for manual pushing)
				velocityDrive(driverForward, driverTurn);
			}
		}

	}

	public void arcadeDrive() {

		//get forward intention from driver left stick Y direction
		double driverForward = square(deadBand(
				-IO.Driver.leftY(),
				Config.DEF_DEADBAND
		));

		//get rotation intention from driver right stick X direction
		double driverTurn = square(deadBand(
				IO.Driver.rightX(),
				Config.DEF_DEADBAND
		));

		double MaxForward = .5;
		double MaxTurn = .5;

		kFalconLeftFront.set(ControlMode.PercentOutput,
				driverForward*MaxForward
						+driverTurn*MaxTurn
		);
		kFalconRightFront.set(ControlMode.PercentOutput,
				driverForward*MaxForward
						-driverTurn*MaxTurn
		);
	}

	/**
	 * @param fwd The Forward intention from -1 to 1 (decimal percentage of max linear speed)
	 * @param rot The Rotation intention from -1 to 1 (decimal percentage of max rotation speed)
	 */
	private void velocityDrive(double fwd, double rot) {

		double kMaxSpeed = 14000; // 12000 (max is 16000)
		double kMaxSpin = 4000; // 4000 previously 7000, previously 8000, adjusted for Isaac
		double targetVelocity_UnitsPer100ms = fwd * kMaxSpeed;
		double turnOffset = rot * kMaxSpin;
		/* 500 RPM in either direction */
		kFalconLeftFront.set(ControlMode.Velocity,
				targetVelocity_UnitsPer100ms+turnOffset
		);
		kFalconRightFront.set(ControlMode.Velocity,
				targetVelocity_UnitsPer100ms-turnOffset
		);
	}

	public void drive(double linear, double angular) {
		var wheelSpeeds = kKinematics.toWheelSpeeds(
				new ChassisSpeeds(
						linear, 0.0, angular
				)
		);
		setWheelSpeeds(wheelSpeeds);
	}

	public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {

		final double leftFeedforward = kLeftFeedForward.calculate(speeds.leftMetersPerSecond);
		final double rightFeedforward = kRightFeedForward.calculate(speeds.rightMetersPerSecond);

		final double leftOutput =
				kLeftPIDController.calculate(getLeftEncoderMeterRate(), speeds.leftMetersPerSecond);
		final double rightOutput =
				kRightPIDController.calculate(getRightEncoderMeterRate(), speeds.rightMetersPerSecond);

		double leftVolts = leftOutput + leftFeedforward;
		double rightVolts = rightOutput + rightFeedforward;

		kFalconLeftFront.set(ControlMode.PercentOutput, leftVolts / RobotController.getBatteryVoltage());
		kFalconRightFront.set(ControlMode.PercentOutput, rightVolts / RobotController.getBatteryVoltage());
	}

	/* =================================================================================================================
	 * RAMSETE RELATED
	 */

	public Rotation2d getGyroRotation2d() {
		double[] ypr = new double[3];
		kPigeon.getYawPitchRoll(ypr);
		return Rotation2d.fromDegrees(ypr[0]);
	}

	public double getGyroYawRaw() {
		double[] ypr = new double[3];
		kPigeon.getYawPitchRoll(ypr);
		return ypr[0];
	}

	public Pose2d getPose() {
		return kOdometry.getPoseMeters();
	}

	// might need to pass in a specific pose when you want it to remember where you are at
	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		kOdometry.resetPosition(pose, getGyroRotation2d());
	}

	// version without pass in
	public void resetOdometry() {
		resetEncoders();
		kOdometry.resetPosition(new Pose2d(), getGyroRotation2d());
	}

	public void updateOdometry() {
		kOdometry.update(getGyroRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters());
	}

	/* =================================================================================================================
	 * DERIVED METHODS
	 */

	private void resetNavigation() {
		resetEncoders();
		resetGyro();
	}

	public double getLeftEncoderMeters() {
		return getLeftEncoderPos() * TechnicalConstants.kMetersPerTick;
	}

	public double getRightEncoderMeters() {
		return getRightEncoderPos() * TechnicalConstants.kMetersPerTick;
	}

	public double getLeftEncoderMeterRate() {
		return getLeftEncoderVel() * TechnicalConstants.kMeterPerSecondsPerTicksPerDeciSecond;
	}

	public double getRightEncoderMeterRate() {
		return getRightEncoderVel() * TechnicalConstants.kMeterPerSecondsPerTicksPerDeciSecond;
	}

	/* =================================================================================================================
	 * ELEMENTARY METHODS
	 */

	private void resetEncoders() {
		kFalconLeftFront.setSelectedSensorPosition(0);
		kFalconLeftBack.setSelectedSensorPosition(0);
		kFalconRightFront.setSelectedSensorPosition(0);
		kFalconRightBack.setSelectedSensorPosition(0);
	}

	private void resetGyro() {
		kPigeon.setYaw(0, Config.DEF_TIMEOUT_MS);
	}

	public double getLeftEncoderPos() {
		return (kFalconLeftFront.getSelectedSensorPosition() + kFalconLeftBack.getSelectedSensorPosition()) / 2.0;
	}

	public double getRightEncoderPos() {
		return (kFalconRightFront.getSelectedSensorPosition() + kFalconRightBack.getSelectedSensorPosition()) / 2.0;
	}

	public double getLeftEncoderVel() {
		return (kFalconLeftFront.getSelectedSensorVelocity() + kFalconLeftBack.getSelectedSensorVelocity()) / 2.0;
	}

	public double getRightEncoderVel() {
		return (kFalconRightFront.getSelectedSensorVelocity() + kFalconRightBack.getSelectedSensorVelocity()) / 2.0;
	}

}
