
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMath.Constants;

import static frc.robot.RobotContainer.*;
import static frc.robot.RobotMath.*;
//import static frc.robot.RobotMath.Constants.aimOffsetX; // Aim offset not being used
import static frc.robot.RobotMath.Constants.turretTurnFrictionConstant;

public class TurretSub extends SubsystemBase {

	private final TalonSRX kTurretTalon;

	/**
	 * Creates a new TurretSub.
	 */
	public TurretSub(TalonSRX kTTalon) {

		this.kTurretTalon = kTTalon;

		kTurretTalon.configFactoryDefault();

		kTurretTalon.setInverted(true);


		
		kTurretTalon.configForwardLimitSwitchSource(
			LimitSwitchSource.FeedbackConnector,
			LimitSwitchNormal.NormallyClosed
		);
		kTurretTalon.configReverseLimitSwitchSource(
			LimitSwitchSource.FeedbackConnector,
			LimitSwitchNormal.NormallyClosed
		);

		kTurretTalon.config_kP(
			PID.kPIDLoopIdx,
			PID.kGainsTurret.kP,
			Constants.kTimeoutMs
		);
		kTurretTalon.config_kI(
			PID.kPIDLoopIdx,
			PID.kGainsTurret.kI,
			Constants.kTimeoutMs
		);
		kTurretTalon.config_kD(
			PID.kPIDLoopIdx,
			PID.kGainsTurret.kD,
			Constants.kTimeoutMs
		);
		kTurretTalon.config_kF(
			PID.kPIDLoopIdx,
			PID.kGainsTurret.kF,
			Constants.kTimeoutMs
		);
	}

	

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void enableInit() {
		resetEncoder();
	}

	public void runTurret() {

		if (kLimelight.getPipeline() == LimelightValue.TeleOp.val()) {
			if (kLimelight.hasTarget()) {
				double targetX = kLimelight.getTargetX();
				turn(
						targetX/Constants.screenSizeX+Math.copySign(
								turretTurnFrictionConstant,
								targetX
						)
				);
			} else {
				turn(0); // I don't know if this is necessary but this is safe
			}

		}

		if (kLimelight.getPipeline() == LimelightValue.Default.val()) {
			double turretTurn = square(deadBand(
					kOperator.getX(GenericHID.Hand.kRight),
					Constants.kDefaultDeadband
			));
			turn(turretTurn);

		}

	}

	private void turn(double value) {
		final double kMaxSpeed = -1.0/2.0;
		turnRaw(value*kMaxSpeed); // TODO comment out for turret disable!
	}

	private void turnRaw(double value) {
		kTurretTalon.set(ControlMode.PercentOutput, value);
	}

	public int getRawEncoder() {
		return kTurretTalon.getSelectedSensorPosition();
	}

	public int getRawEncoderSped() {
		return kTurretTalon.getSelectedSensorVelocity();
	}

	private void resetEncoder() {
		kTurretTalon.setSelectedSensorPosition(0);
	}
}
