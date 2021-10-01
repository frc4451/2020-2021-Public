package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import static frc.robot.Debug.debug;
import static frc.robot.RobotContainer.*;

public class Climber extends SubsystemBase {

	private final TalonFX kClimbFalcon;

	private final Solenoid kClimbSolenoid;

	private boolean climberBrakeState = SolenoidState.BrakeDisengaged.val();

	public Climber(TalonFX _kClimbFalcon, Solenoid _kClimbSolenoid) {
		debug("climber subsystem initializing: " + this.toString());

		// We cut the climber off lol

		kClimbFalcon = _kClimbFalcon;
		kClimbSolenoid = _kClimbSolenoid;

		kClimbFalcon.configFactoryDefault();

		kClimbFalcon.setInverted(true);

		kClimbFalcon.setNeutralMode(NeutralMode.Brake);

		kClimbFalcon.config_kP(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsClimber.kP,
				Config.DEF_TIMEOUT_MS
		);
		kClimbFalcon.config_kI(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsClimber.kI,
				Config.DEF_TIMEOUT_MS
		);
		kClimbFalcon.config_kD(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsClimber.kD,
				Config.DEF_TIMEOUT_MS
		);
		kClimbFalcon.config_kF(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsClimber.kF,
				Config.DEF_TIMEOUT_MS
		);

		kClimbFalcon.configMotionAcceleration(Settings.CLIMBER_MAX_ACC);
		kClimbFalcon.configMotionCruiseVelocity(Settings.CLIMBER_CRUISE_VEL);

	}

	public void enableInit() {
		resetEncoder();
		// sets brake solenoid to unlocked, this should either be when on the floor starting auto or when on the floor
		// of the shop after a match.
		climberBrakeState = SolenoidState.BrakeEngaged.val();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void runClimb() {
//		debug("Running Climb", "debug");

		double climbSpeed = .7;

		int normalClimbTarget = -135500;

		if (IO.Operator.lCenterPressed()) {
			climberBrakeState = SolenoidState.BrakeDisengaged.val();
		}

		if (IO.Operator.bPressed()) {
			climberBrakeState = SolenoidState.BrakeEngaged.val();
		}

		kClimbSolenoid.set(climberBrakeState);

//		debug("climberBrakeState:" + climberBrakeState, "debug");

		if (climberBrakeState == SolenoidState.BrakeDisengaged.val()) { // if brake is disengaged, do inputs
			if (IO.Operator.x()) {

				motionMagic(normalClimbTarget);

			} else {

				if (IO.Operator.leftY() < -.25) { // if joystick up
					climb(-climbSpeed); // extend climber
				} else if (IO.Operator.leftY() > .25) { // if joystick down
					climb(climbSpeed + (IO.Operator.y() ? .25 : 0)); // retract climber
				} else {
					climb(0 + (IO.Operator.y() ? .25 : 0)); // .25 is incremental
				}

			}
		} else {
			climb(0);
		}

	}

	private void climb(double value) { // value is percent of max speed
		kClimbFalcon.set(
			ControlMode.PercentOutput,
			value * Settings.MAX_CLIMB_FF
		);
	}

	private void motionMagic(int position) {
		kClimbFalcon.set(ControlMode.MotionMagic, position);
	}

	private void resetEncoder() {
		kClimbFalcon.setSelectedSensorPosition(0);
	}

	public double getRawPosition() {
		return kClimbFalcon.getSelectedSensorPosition();
	}

	public double getRawVelocity() {
		return kClimbFalcon.getSelectedSensorVelocity();
	}

	public boolean getClimberBrakeState() {
		return climberBrakeState;
	}

//	public void lockClimberOnDisable() {
//		climberBrakeState = SolenoidState.BrakeEngaged.val();
//		kClimbSolenoid.set(climberBrakeState);
//	}

}
