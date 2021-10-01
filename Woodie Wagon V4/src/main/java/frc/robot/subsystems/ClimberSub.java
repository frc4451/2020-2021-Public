
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import static frc.robot.Debug.*;

import static frc.robot.Constants.*;
import static frc.robot.RobotContainer.*;

public class ClimberSub extends SubsystemBase {

	private final TalonFX kClimbFalcon;

	private final Solenoid kClimbSolenoid;

	private boolean climberBrakeState = SolenoidState.BrakeDisengaged.val();

	/**
	 * Creates a new ClimberSub.
	 */
	public ClimberSub(TalonFX kCFalcon, Solenoid kCSolenoid) {
		debug("climber subsystem initializing: " + this.toString());

		this.kClimbFalcon = kCFalcon;

		this.kClimbSolenoid = kCSolenoid;

		kClimbFalcon.configFactoryDefault();

		kClimbFalcon.setInverted(true);

		kClimbFalcon.setNeutralMode(NeutralMode.Brake);

		kClimbFalcon.config_kP(
			PID.kPIDLoopIdx,
			PID.kGainsClimber.kP,
			Config.kTimeoutMs
		);
		kClimbFalcon.config_kI(
			PID.kPIDLoopIdx,
			PID.kGainsClimber.kI,
			Config.kTimeoutMs
		);
		kClimbFalcon.config_kD(
			PID.kPIDLoopIdx,
			PID.kGainsClimber.kD,
			Config.kTimeoutMs
		);
		kClimbFalcon.config_kF(
			PID.kPIDLoopIdx,
			PID.kGainsClimber.kF,
			Config.kTimeoutMs
		);

		kClimbFalcon.configMotionAcceleration(Config.kClimberMaxAcceleration);
		kClimbFalcon.configMotionCruiseVelocity(Config.kClimberCruiseVelocity);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void enableInit() {
		resetEncoder();
		// sets brake solenoid to unlocked, this should either be when on the floor starting auto or when on the floor
		// of the shop after a match.
		climberBrakeState = SolenoidState.BrakeEngaged.val();
	}

	public void runClimb() {

		double climbSpeed = .7;

		int normalClimbTarget = -135500;

		if (kOperator.getStartButtonPressed()) {
			climberBrakeState = SolenoidState.BrakeDisengaged.val();
		}

		if (kOperator.getBButtonPressed()) {
			climberBrakeState = SolenoidState.BrakeEngaged.val();
		}

		if (kOperator.getXButton()) {

			if (climberBrakeState == SolenoidState.BrakeDisengaged.val()) {
				motionMagic(normalClimbTarget);
			}

		} else {

			kClimbSolenoid.set(climberBrakeState);

			if (climberBrakeState == SolenoidState.BrakeDisengaged.val()) { // brake is disengaged
				if (kOperator.getY(GenericHID.Hand.kLeft) < -.25) { // if joystick up
					climb(-climbSpeed); // extend climber
				} else if (kOperator.getY(GenericHID.Hand.kLeft) > .25) { // if joystick down
					climb(climbSpeed + (kOperator.getYButton() ? .25 : 0)); // retract climber
				} else {
					climb(0 + (kOperator.getYButton() ? .25 : 0)); // .25 is incremental
				}
			}

		}

	}

	private void climb(double value) { // value is percent of max speed
//		kClimbFalcon.set( // Why did I comment this out...
//			ControlMode.PercentOutput,
//			value * Config.kMaxClimbFF
//		);
	}

	private void motionMagic(int position) {
		kClimbFalcon.set(ControlMode.MotionMagic, position);
	}

	private void resetEncoder() {
		kClimbFalcon.setSelectedSensorPosition(0);
	}

	public int getRawPosition() {
		return kClimbFalcon.getSelectedSensorPosition();
	}

	public int getRawVelocity() {
		return kClimbFalcon.getSelectedSensorVelocity();
	}

	public boolean getClimberBrakeState() {
		return climberBrakeState;
	}

	public void lockClimberOnDisable() {
		climberBrakeState = SolenoidState.BrakeEngaged.val();
		kClimbSolenoid.set(climberBrakeState);
	}

}
