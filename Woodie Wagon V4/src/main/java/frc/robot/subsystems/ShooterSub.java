
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import static frc.robot.Constants.*;

import static frc.robot.Debug.*;
import static frc.robot.RobotContainer.kDriver;
import static frc.robot.RobotContainer.kOperator;

public class ShooterSub extends SubsystemBase {

	private final TalonFX kShooterFalcon;

	private double shooterSetSpeed;

	private double shooterSpeedModifier;

	boolean driverUpCurr = false;
	boolean driverUpPrev = false;
	boolean driverDownCurr = false;
	boolean driverDownPrev = false;

	/**
	 * Creates a new ShooterSub.
	 */
	public ShooterSub(TalonFX kSFalcon) {
		debug("shooter subsystem initializing: " + this.toString());

		this.kShooterFalcon = kSFalcon;

		shooterSetSpeed = 0;

		shooterSpeedModifier = 0;

		kShooterFalcon.configFactoryDefault();

		kShooterFalcon.setInverted(true);

		kShooterFalcon.configSelectedFeedbackSensor(
			FeedbackDevice.IntegratedSensor,
			PID.kPIDLoopIdx, 
			Config.kTimeoutMs);

		kShooterFalcon.setSensorPhase(false);

		kShooterFalcon.configNominalOutputForward(0, Config.kTimeoutMs);
		kShooterFalcon.configNominalOutputReverse(0, Config.kTimeoutMs);
		kShooterFalcon.configPeakOutputForward(1, Config.kTimeoutMs);
		kShooterFalcon.configPeakOutputReverse(-1, Config.kTimeoutMs);

		kShooterFalcon.setNeutralMode(NeutralMode.Coast);

		kShooterFalcon.configClosedloopRamp(0);

		kShooterFalcon.config_kF(PID.kPIDLoopIdx,
			PID.kGainsShooter.kF,
			Config.kTimeoutMs
		);
		kShooterFalcon.config_kP(PID.kPIDLoopIdx,
			PID.kGainsShooter.kP,
			Config.kTimeoutMs
		);
		kShooterFalcon.config_kI(PID.kPIDLoopIdx,
			PID.kGainsShooter.kI,
			Config.kTimeoutMs
		);
		kShooterFalcon.config_kD(PID.kPIDLoopIdx,
			PID.kGainsShooter.kD,
			Config.kTimeoutMs
		);

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void enableInit() {

	}

	public void shooterSpin() {

		switch(kOperator.getPOV()) {
			case 0: //North
				setRamp(0);
				spin(9925); // long range: actual was 11950,
				break;
			case 90: // West
			case 270: // East
				setRamp(0);
				spin(9400); // mid range: actual was 9777
				break;
			case 180: //South
				setRamp(0);
				spin(8750); // short range 8750
				break;
			// mid range again
			case -1: //not pressing
				spin(-1);
				break;
			default: //this means something is wrong
				spin(0);
				break;
		}

		//update button states on joystick
		this.driverDownPrev = this.driverDownCurr;
		this.driverUpPrev = this.driverUpCurr;
		this.driverUpCurr = false;
		this.driverDownCurr = false;

		this.driverUpCurr = (kDriver.getPOV() == 0); // North
		this.driverDownCurr = (kDriver.getPOV() == 180); // South

		//check for changes and update

		if (this.driverUpCurr && !this.driverUpPrev) {
			shooterSpeedModifier += 50;
		}

		if (this.driverDownCurr && !this.driverDownPrev) {
			shooterSpeedModifier -= 50;
		}

		/* Controller Haptic Feedback */

//		// this is being derpy so leave it alone
//		 if (kShooterSub.getRawSpeed()
//		 		>= shooterSetSpeed*Config.kUpToSpeedThreshold
//		 		&& shooterSetSpeed != 0) {
//		 	kOperator.setRumble(RumbleType.kLeftRumble, 1);
//		 } else {
//		 	kOperator.setRumble(RumbleType.kLeftRumble, 0);
//		 }

	}

	public void spin(double target) {
		if (target == -1) {
			this.kShooterFalcon.set(ControlMode.PercentOutput, 0);
			this.shooterSetSpeed = -1;
		}
		else if (target == 0) {
			this.kShooterFalcon.set(ControlMode.Velocity, 0);
			this.shooterSetSpeed = 0;
		}
		else {
			kShooterFalcon.set(
				ControlMode.Velocity,
				target + this.shooterSpeedModifier
			);
			this.shooterSetSpeed = target + this.shooterSpeedModifier;
		}
	}

	public void setRamp(double value) {
		kShooterFalcon.configClosedloopRamp(value);
	}

	public double getRawSpeed() {
		return kShooterFalcon.getSelectedSensorVelocity();
	}

	public double getShooterSetSpeed() {
		return shooterSetSpeed;
	}

	public double getShooterSpeedModifier() {
		return shooterSpeedModifier;
	}

}
