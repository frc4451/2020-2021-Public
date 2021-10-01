
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Config.*;
import static frc.robot.Constants.deadband;
import static frc.robot.Constants.square;
import static frc.robot.Debug.*;
import static frc.robot.RobotContainer.*;

import frc.robot.Constants.*;

public class TurretSub extends SubsystemBase {

	private final TalonSRX kTurretTalon;

	/**
	 * Creates a new TurretSub.
	 */
	public TurretSub(TalonSRX kTTalon) {
		debug("turret subsystem initializing: " + this.toString());

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
			Config.kTimeoutMs
		);
		kTurretTalon.config_kI(
			PID.kPIDLoopIdx,
			PID.kGainsTurret.kI,
			Config.kTimeoutMs
		);
		kTurretTalon.config_kD(
			PID.kPIDLoopIdx,
			PID.kGainsTurret.kD,
			Config.kTimeoutMs
		);
		kTurretTalon.config_kF(
			PID.kPIDLoopIdx,
			PID.kGainsTurret.kF,
			Config.kTimeoutMs
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

		if (kLimeLight.getPipeline() == 9) {
			if (kLimeLight.hasTarget()) {
				double xOffset = kLimeLight.getXOffset() + aimOffsetX;
				turn(
						xOffset/Config.screenSizeX+Math.copySign(
								turretTurnFrictionConstant,
								xOffset
						)
				);
			}

		}

		if (kLimeLight.getPipeline() == LimelightValue.Default.val()) {
			double turretTurn = square(deadband(
					kOperator.getX(GenericHID.Hand.kRight),
					Config.kDefaultDeadband
			));
			debug(" Op Right X input: " + kOperator.getX(GenericHID.Hand.kRight));
			debug("turning turret: " + turretTurn);
			turn(turretTurn);

		}
		else if (kLimeLight.getPipeline() == LimelightValue.Short.val())  {
			if (kLimeLight.hasTarget()) {
				double xOffset = kLimeLight.getXOffset();
				turn(
						xOffset/Config.screenSizeX+Math.copySign(
								turretTurnFrictionConstant,
								xOffset
						)
				);
			}

			// } else if (kLimeLight.getPipeline() == LimelightValue.Medium.val()) {

			// 	if ( kLimeLight.hasTarget()) {
			// 		double xOffset = kLimeLight.getXOffset();
			// 		turn(xOffset/screenSizeX/2+turnFrictionConstant);
			// 	}

		}
		else if (kLimeLight.getPipeline() == LimelightValue.Long.val()) {

			if (kLimeLight.hasTarget()) {
				double xOffset = kLimeLight.getXOffset();
				turn(
						xOffset/Config.screenSizeX/4+Math.copySign(
								turretTurnFrictionConstant,
								xOffset
						)
				);
			}

		}

	}

	private void turn(double value) {
		final double kMaxSpeed = -1.0/2.0;
		turnRaw(value*kMaxSpeed);
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
