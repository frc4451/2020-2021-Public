package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotController;

import static frc.robot.RobotMath.*;
import static frc.robot.RobotContainer.*;

public class Drivetrain extends SubsystemBase {

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
			PID.kGainsLeft.kP,
			PID.kGainsLeft.kI,
			PID.kGainsLeft.kD);

	private final PIDController kRightPIDController = new PIDController(
			PID.kGainsRight.kP,
			PID.kGainsRight.kI,
			PID.kGainsRight.kD);

	private final SimpleMotorFeedforward kLeftFeedForward = new SimpleMotorFeedforward(
			PID.kGainsLeft.kS,
			PID.kGainsLeft.kV,
			PID.kGainsLeft.kA
	);

	private final SimpleMotorFeedforward kRightFeedForward = new SimpleMotorFeedforward(
			PID.kGainsRight.kS,
			PID.kGainsRight.kV,
			PID.kGainsRight.kA
			);

	public Drivetrain(
			TalonFX _kFalconLeftFront,
			TalonFX _kFalconLeftBack,
			TalonFX _kFalconRightFront,
			TalonFX _kFalconRightBack,
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
				PID.kPIDLoopIdx,
				kTimeoutMs);


		kFalconRightFront.configSelectedFeedbackSensor(
				FeedbackDevice.IntegratedSensor,
				PID.kPIDLoopIdx,
				kTimeoutMs);
		/*
		  Phase sensor accordingly.
		 Positive Sensor Reading should match Green (blinking) Leds on Talon
		*/
		kFalconLeftFront.setSensorPhase(true); // TODO uncomment
		kFalconRightFront.setSensorPhase(false);

		/* Config the peak and nominal outputs */
		kFalconLeftFront.configNominalOutputForward(0, kTimeoutMs);
		kFalconLeftFront.configNominalOutputReverse(0, kTimeoutMs);
		kFalconLeftFront.configPeakOutputForward(1, kTimeoutMs);
		kFalconLeftFront.configPeakOutputReverse(-1, kTimeoutMs);

		kFalconRightFront.configNominalOutputForward(0, kTimeoutMs);
		kFalconRightFront.configNominalOutputReverse(0, kTimeoutMs);
		kFalconRightFront.configPeakOutputForward(1, kTimeoutMs);
		kFalconRightFront.configPeakOutputReverse(-1, kTimeoutMs);

		kFalconLeftFront.configMotionAcceleration(Constants.kDriveTrainMaxAcceleration);
		kFalconLeftFront.configMotionCruiseVelocity(Constants.kDriveTrainCruiseVelocity);

		kFalconRightFront.configMotionAcceleration(Constants.kDriveTrainMaxAcceleration);
		kFalconRightFront.configMotionCruiseVelocity(Constants.kDriveTrainCruiseVelocity);

		/* Config the Velocity closed loop gains in slot0 */

		kFalconLeftFront.config_kP(
				PID.kPIDLoopIdx,
				PID.kGainsLeft.kP,
				kTimeoutMs
		);
		kFalconLeftFront.config_kI(
				PID.kPIDLoopIdx,
				PID.kGainsLeft.kI,
				kTimeoutMs
		);
		kFalconLeftFront.config_kD(
				PID.kPIDLoopIdx,
				PID.kGainsLeft.kD,
				kTimeoutMs
		);
		kFalconLeftFront.config_kF(
				PID.kPIDLoopIdx,
				PID.kGainsLeft.kF,
				kTimeoutMs
		);

		kFalconRightFront.config_kP(
				PID.kPIDLoopIdx,
				PID.kGainsRight.kP,
				kTimeoutMs
		);
		kFalconRightFront.config_kI(
				PID.kPIDLoopIdx,
				PID.kGainsRight.kI,
				kTimeoutMs
		);
		kFalconRightFront.config_kD(
				PID.kPIDLoopIdx,
				PID.kGainsRight.kD,
				kTimeoutMs
		);
		kFalconRightFront.config_kF(
				PID.kPIDLoopIdx,
				PID.kGainsRight.kF,
				kTimeoutMs
		);

		kFalconLeftFront.configClosedloopRamp(.75);
		kFalconRightFront.configClosedloopRamp(.75);

		resetEncoders();
		resetGyro();


	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void runDriveTrain() {

		// reset encoders tool
		if (kDriver.getBackButton()) {
			resetEncoders();
		}



		/* Teleop Drive with Velocity control */
		//get forward intention from driver left stick Y direction
		double driverForward = square(deadBand(
				-kDriver.getY(GenericHID.Hand.kLeft),
				kDefaultDeadband
		));
		//get rotation intention from driver right stick X direction
		double driverTurn = square(deadBand(
				kDriver.getX(GenericHID.Hand.kRight),
				kDefaultDeadband
		));

		System.out.println("#runDriveTrain " + driverForward);

		if (kDriver.getBumper(GenericHID.Hand.kLeft)) {
			velocityDrive(driverForward, -.5);
		} else if (kDriver.getBumper(GenericHID.Hand.kRight)) {
			velocityDrive(driverForward, .5);
		} else {
			if (!kDriver.getStartButton()) { // if not disabled by start button tool (this is for manual moving)
				velocityDrive(driverForward, driverTurn);
			}
		}

	}

	public void arcadeDrive() {

		//get forward intention from driver left stick Y direction
		double driverForward = square(deadBand(
				-kDriver.getY(GenericHID.Hand.kLeft),
				kDefaultDeadband
		));

		//get rotation intention from driver right stick X direction
		double driverTurn = square(deadBand(
				kDriver.getX(GenericHID.Hand.kRight),
				kDefaultDeadband
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
	 * @param fwd The Forward intention from -1 to 1
	 * @param rot The Rotation intention from -1 to 1
	 */
	private void velocityDrive(double fwd, double rot) {

		System.out.println("#velocityDrive " + fwd + " " + rot);

		double kMaxSpeed = 12000;
		double kMaxSpin = 4000; // previously 7000, previously 8000, adjusted for Isaac
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
		var wheelSpeeds = kKinematics.toWheelSpeeds(new ChassisSpeeds(linear, 0.0, angular));
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

	public void updateOdometry() {
		kOdometry.update(getGyroRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters());
	}

	private void resetEncoders() {
		kFalconLeftFront.setSelectedSensorPosition(0);
		kFalconLeftBack.setSelectedSensorPosition(0);
		kFalconRightFront.setSelectedSensorPosition(0);
		kFalconRightBack.setSelectedSensorPosition(0);
	}

	public void resetGyro() {
		kPigeon.setYaw(0);
	}

	public double getLeftEncoderMeters() {
		return getLeftEncoderPos() * Constants.kMetersPerTick;
	}

	public double getRightEncoderMeters() {
		return getRightEncoderPos() * Constants.kMetersPerTick;
	}

	public double getLeftEncoderMeterRate() {
		return getLeftEncoderVel() * Constants.kMeterPerSecondsPerTicksPerDeciSecond;
	}

	public double getRightEncoderMeterRate() {
		return getRightEncoderVel() * Constants.kMeterPerSecondsPerTicksPerDeciSecond;
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
