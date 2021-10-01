
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.Constants.*;

import static frc.robot.Constants.*;
import static frc.robot.Debug.*;
import static frc.robot.RobotContainer.*;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class DriveTrainSub extends SubsystemBase {

	private final TalonFX kFalconLF;
	private final TalonFX kFalconLB;
	private final TalonFX kFalconRF;
	private final TalonFX kFalconRB;

	private boolean boostMode = false;
	/**
	 * Creates a new DriveTrainSub.
	 */
	public DriveTrainSub(TalonFX kFLF, TalonFX kFLB, TalonFX kFRF, TalonFX kFRB) {
		debug("driveTrain subsystem initializing: "+this.toString());

		this.kFalconLF = kFLF;
		this.kFalconLB = kFLB;
		this.kFalconRF = kFRF;
		this.kFalconRB = kFRB;

		kFalconLF.configFactoryDefault();
		kFalconLB.configFactoryDefault();
		kFalconRF.configFactoryDefault();
		kFalconRB.configFactoryDefault();

		kFalconLB.follow(kFalconLF);
		kFalconRB.follow(kFalconRF);

		kFalconLF.setInverted(false);
		kFalconLB.setInverted(false);
		kFalconRF.setInverted(true);
		kFalconRB.setInverted(true);

		/* Config sensor used for Primary PID [Velocity] */
		kFalconLF.configSelectedFeedbackSensor(
			FeedbackDevice.IntegratedSensor,
			PID.kPIDLoopIdx, 
			Config.kTimeoutMs);


		kFalconRF.configSelectedFeedbackSensor(
			FeedbackDevice.IntegratedSensor,
			PID.kPIDLoopIdx, 
			Config.kTimeoutMs);
		/*
		  Phase sensor accordingly.
		 Positive Sensor Reading should match Green (blinking) Leds on Talon
		*/
		kFalconLF.setSensorPhase(true);
		kFalconRF.setSensorPhase(false);

		/* Config the peak and nominal outputs */
		kFalconLF.configNominalOutputForward(0, Config.kTimeoutMs);
		kFalconLF.configNominalOutputReverse(0, Config.kTimeoutMs);
		kFalconLF.configPeakOutputForward(1, Config.kTimeoutMs);
		kFalconLF.configPeakOutputReverse(-1, Config.kTimeoutMs);

		kFalconRF.configNominalOutputForward(0, Config.kTimeoutMs);
		kFalconRF.configNominalOutputReverse(0, Config.kTimeoutMs);
		kFalconRF.configPeakOutputForward(1, Config.kTimeoutMs);
		kFalconRF.configPeakOutputReverse(-1, Config.kTimeoutMs);

		kFalconLF.configMotionAcceleration(Config.kDriveTrainMaxAcceleration);
		kFalconLF.configMotionCruiseVelocity(Config.kDriveTrainCruiseVelocity);

		kFalconRF.configMotionAcceleration(Config.kDriveTrainMaxAcceleration);
		kFalconRF.configMotionCruiseVelocity(Config.kDriveTrainCruiseVelocity);

		/* Config the Velocity closed loop gains in slot0 */
		
		kFalconLF.config_kP(
			PID.kPIDLoopIdx,
			PID.kGainsLeft.kP,
			Config.kTimeoutMs
		);
		kFalconLF.config_kI(
			PID.kPIDLoopIdx,
			PID.kGainsLeft.kI,
			Config.kTimeoutMs
		);
		kFalconLF.config_kD(
			PID.kPIDLoopIdx,
			PID.kGainsLeft.kD,
			Config.kTimeoutMs
		);
		kFalconLF.config_kF(
			PID.kPIDLoopIdx,
			PID.kGainsLeft.kF,
			Config.kTimeoutMs
		);
		
		kFalconRF.config_kP(
			PID.kPIDLoopIdx,
			PID.kGainsRight.kP,
			Config.kTimeoutMs
		);
		kFalconRF.config_kI(
			PID.kPIDLoopIdx,
			PID.kGainsRight.kI,
			Config.kTimeoutMs
		);
		kFalconRF.config_kD(
			PID.kPIDLoopIdx,
			PID.kGainsRight.kD,
			Config.kTimeoutMs
		);
		kFalconRF.config_kF(
			PID.kPIDLoopIdx,
			PID.kGainsRight.kF,
			Config.kTimeoutMs
		);

		kFalconLF.configClosedloopRamp(.75);
		kFalconRF.configClosedloopRamp(.75);

		resetEncoders();

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

	}

	public void enableInit() {
//		resetEncoders();
		resetNavigation();
	}

	public void teleOpInit() {
		boostMode = false;
	}

	public boolean getBoostMode() {
		return boostMode;
	}

	public void runDriveTrainStraight() {

		debug("LB: " + kFalconLB.getSelectedSensorPosition());
		debug("LF: " + kFalconLF.getSelectedSensorPosition());
		debug("RB: " + kFalconRB.getSelectedSensorPosition());
		debug("RF: " + kFalconRF.getSelectedSensorPosition());

		/* Teleop Drive with Velocity control */
		//get forward intention from driver left stick Y direction
		double driverForward = square(deadband(
				-kDriver.getY(GenericHID.Hand.kLeft),
				Config.kDefaultDeadband
		));

		if (!kDriver.getStartButton()) { // if not disabled by start button tool (this is for manual moving)
			velocityDriveStraight(driverForward, 0);
		}
	}

	public void runDriveTrain() {

		// reset encoders tool
		if (kDriver.getBackButton()) {
			resetEncoders();
		}

		debug("LB: " + kFalconLB.getSelectedSensorPosition());
		debug("LF: " + kFalconLF.getSelectedSensorPosition());
		debug("RB: " + kFalconRB.getSelectedSensorPosition());
		debug("RF: " + kFalconRF.getSelectedSensorPosition());

		/* Teleop Drive with Velocity control */
		//get forward intention from driver left stick Y direction
		double driverForward = square(deadband(
				-kDriver.getY(GenericHID.Hand.kLeft),
				Config.kDefaultDeadband
		));
		//get rotation intention from driver right stick X direction
		double driverTurn = square(deadband(
				kDriver.getX(GenericHID.Hand.kRight),
				Config.kDefaultDeadband
		));

		if (kDriver.getBumper(GenericHID.Hand.kLeft)) {
			velocityDrive(driverForward, -.5);
		} else if (kDriver.getBumper(GenericHID.Hand.kRight)) {
			velocityDrive(driverForward, .5);
		} else {
			if (!kDriver.getStartButton()) { // if not disabled by start button tool (this is for manual moving)
				velocityDrive(driverForward, driverTurn);
			}
		}

		if (kDriver.getBButtonPressed()) {
			boostMode = !boostMode;
		}

		// speed shooting challenge MM setpoint: -190000

//		if (kDriver.getPOV() == 90) {  // east
//			motionMagic(-190000);
//		}
//		if (kDriver.getPOV() == 270) {  // west
//			motionMagic(0);
//		}

	}

	/**
	 * @param fwd The Forward intention from -1 to 1
	 * @param rot The Rotation intention from -1 to 1
	 */
	private void velocityDrive(double fwd, double rot) {
		debug("Running velocityDrive(): forward: " + fwd 
				+ ", rotation: " + rot + ".");
		double kMaxSpeed = 12000;
		if (boostMode) { // if super speed button is activate
			kMaxSpeed = 16000; // previously 15000 previously 18000
		} else {
			kMaxSpeed = 12000; // previously 15000 previously 18000
		}
		double kMaxSpin = 4000; // previously 7000, previously 8000, adjusted for Isaac
		double targetVelocity_UnitsPer100ms = fwd * kMaxSpeed;
		double turnOffset = rot * kMaxSpin;
		/* 500 RPM in either direction */
		kFalconLF.set(ControlMode.Velocity,
			targetVelocity_UnitsPer100ms+turnOffset
		);
		kFalconRF.set(ControlMode.Velocity,
			targetVelocity_UnitsPer100ms-turnOffset
		);
	}

	public void velocityDriveStraight(double fwd, double rotarget) {

		final double kMaxSpeed = 18000;
		final double kMaxSpin = 8000; // previously 8000, adjusted for Isaac

		double angleDif = clamp(getGyroHeading() - rotarget, 2);
		double pValue = 0.03;
		kFalconLF.set(
				ControlMode.Velocity,
				fwd*kMaxSpeed + (pValue * angleDif) * kMaxSpin
		);
		kFalconRF.set(
				ControlMode.Velocity,
				fwd*kMaxSpeed - (pValue * angleDif) * kMaxSpin
		);

	}

	public void motionMagic(double target) {
		motionMagic(target, target);
//		kFalconLF.set(ControlMode.MotionMagic,
//			target
//		);
//		kFalconRF.set(ControlMode.MotionMagic,
//			target
//		);
	}

	public void motionMagic(double left, double right) {
		kFalconLF.set(ControlMode.MotionMagic,
			left
		);
		kFalconRF.set(ControlMode.MotionMagic,
			right
		);
	}

//	public void motionMagicStraight(double target) {
//		kFalconLF.set(
//				ControlMode.MotionMagic,
//				target,
//				DemandType.AuxPID,
//
//		);
//		kFalconRF.set(
//				ControlMode.MotionMagic,
//				target
//		);
//	}

	// this is very inaccurate and stupid
	public void motionMagicStraightButJank(double distanceTarget, double headingTarget) {
		double angleDif = getGyroHeading() - headingTarget;
		double pValue = 1;
		kFalconLF.set(
				ControlMode.MotionMagic,
				distanceTarget + pValue * angleDif
				);
		kFalconRF.set(
				ControlMode.MotionMagic,
				distanceTarget - pValue * angleDif
		);

	}

	public double getLeftEncoderPos() {
		return kFalconLF.getSelectedSensorPosition();
	}

	public double getRightEncoderPos() {
		return kFalconRF.getSelectedSensorPosition();
	}

	private void resetEncoders() {
		kFalconLF.setSelectedSensorPosition(0);
		kFalconLB.setSelectedSensorPosition(0);
		kFalconRF.setSelectedSensorPosition(0);
		kFalconRB.setSelectedSensorPosition(0);
	}

	private void resetNavigation() {
		resetEncoders();
		kGyro.setYaw(0, Config.kTimeoutMs);
	}

	public double getGyroHeading() {
		double[] ypr = new double[3];
		kGyro.getYawPitchRoll(ypr);
		return ypr[0];
	}

	public void debugEncoders() {
		debug("LB: " + kFalconLB.getSelectedSensorPosition());
		debug("LF: " + kFalconLF.getSelectedSensorPosition());
		debug("RB: " + kFalconRB.getSelectedSensorPosition());
		debug("RF: " + kFalconRF.getSelectedSensorPosition());
	}
}
