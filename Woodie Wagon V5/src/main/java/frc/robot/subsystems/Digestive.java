package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static frc.robot.Debug.debug;
import static frc.robot.RobotContainer.*;
import static frc.robot.RobotMath.*;

public class Digestive extends SubsystemBase {

	private final VictorSPX kIntakeVictor;
	private final DoubleSolenoid kIntakeSolenoid;
	private final VictorSPX kFloorAgitator;
	private final TalonSRX kTowerBelt;
	private final TalonSRX kEggBeaterAgitator;
	private final DigitalInput kTopBeamBreak;
	private final DigitalInput kBottomBeamBreak;

	// Queuing delay: waits .25 seconds before queuing starts
	private final double qDelay = 0.25; // in seconds
	private double qMemory = -qDelay;
	private boolean qStatus = false;

	private final double intakeMotorOffDelay = 0.5; // in seconds
//	private double retractMemory = -intakeMotorOffDelay;
	private boolean intakeState = false; // true is extended
//	private boolean toggleMode = false; // for manual intake toggles



	public Digestive(
			VictorSPX _kIntakeVictor,
			DoubleSolenoid _kIntakeSolenoid,
			VictorSPX _kFloorAgitator,
			TalonSRX _kTowerBelt,
			TalonSRX _kEggBeaterAgitator,
			DigitalInput _kTopBeamBreak,
			DigitalInput _kBottomBeamBreak
	) {
		kIntakeVictor = _kIntakeVictor;
		kIntakeSolenoid = _kIntakeSolenoid;
		kFloorAgitator = _kFloorAgitator;
		kTowerBelt = _kTowerBelt;
		kEggBeaterAgitator = _kEggBeaterAgitator;
		kTopBeamBreak = _kTopBeamBreak;
		kBottomBeamBreak = _kBottomBeamBreak;


		kIntakeVictor.configFactoryDefault();
		kIntakeVictor.setInverted(true);

		kIntakeSolenoid.set(Value.kReverse);

		kFloorAgitator.configFactoryDefault();
		kTowerBelt.configFactoryDefault();
		kEggBeaterAgitator.configFactoryDefault();

		kFloorAgitator.setInverted(true);
		kTowerBelt.setInverted(true);
		kEggBeaterAgitator.setInverted(false);
	}

	public void enableInit() {
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
//		if (
//				(intakeState && !toggleMode)
//						|| getTimeSince(retractMemory)
//						< intakeMotorOffDelay
//		) {
//			kIntakeVictor.set(
//					ControlMode.PercentOutput,
//					Settings.DEF_INTAKE_FF
//			);
//		} else {
//			kIntakeVictor.set(ControlMode.PercentOutput, 0);
//		}
	}

	public void runDigestive() {

		// if operator doing stuff with digestive
		if (
				IO.Operator.lBump() // if priming
						|| IO.Operator.rBump() // or if reverse
						|| IO.Operator.rTrig() >= Config.DEF_DEADBAND // or if shooting
		) {

			// run prime if Left Operator bumper
			if (IO.Operator.lBump()) {
				primeOn();
			}

			// when firing, feed on. (definition of firing)
			if (IO.Operator.rTrig() >= Config.DEF_DEADBAND) {
				shootFeed();
			}

			// when operator is doing something, only do intake functions for driver, don't allow digestive
			if (IO.Driver.rTrig() >= Config.DEF_DEADBAND) {
				intakeExtend();
				intakeRoll();
			} else {
				intakeStop();

				// if driver wants to reverse intake (right now in any extend/retract position), do it
				if (IO.Driver.b() && intakeState) { // only if intake is down
					intakeReverse();
				}

				if (IO.Driver.rCenterPressed()) { // Intake Extend/Retract Toggle
					if (intakeState) {
						intakeRetract();
					} else {
						intakeExtend();
					}

				}
			}

//			// button above fire to reverse tower (REMOVED, this was hazard because of changed tower structure)
			if (IO.Operator.rBump()) {
				reverseOn();
			}
		}

		else { // if operator not doing anything:
			/*
			 * Intake extension and retraction with Driver right trigger
			 * and priming of tower
			 * If the Drivers right trigger, extend the intake
			 * and run agitator and tower unless the tower limit switch triggers.
			 */
			if (IO.Driver.rTrig() >= Config.DEF_DEADBAND) {
				primeOn();
				intakeExtend();
				intakeRoll();
			} else {
				feedOff();
				intakeStop();

				// if driver wants to reverse intake (right now in any extend/retract position), do it
				if (IO.Driver.b() && intakeState) { // only if intake is down
					intakeReverse();
				}

				if (IO.Driver.rCenterPressed()) { // Intake Extend/Retract Toggle
					if (intakeState) {
						intakeRetract();
					} else {
						intakeExtend();
					}

				}
			}
		}

	}


	/**
	 * Extends intake and turns intake motor on.
	 */
	public void intakeExtend() {
		kIntakeSolenoid.set(Value.kForward);
		intakeState = true;
	}

	/**
	 * Retracts intake and turns intake motor off.
	 */
	public void intakeRetract() {
		kIntakeSolenoid.set(Value.kReverse);
//		TODO: Might use this for edge cases where driver toggles intake up right as they are stopping roller
//		if (intakeState) {
//			retractMemory = getTime();
//		}
		intakeState = false;
	}

	public void intakeRoll() {
		kIntakeVictor.set(
				ControlMode.PercentOutput,
				Settings.DEF_INTAKE_FF
		);
	}

	public void intakeStop() {
		kIntakeVictor.set(
				ControlMode.PercentOutput,
				0
		);
	}

	public void intakeReverse() {
		kIntakeVictor.set(
				ControlMode.PercentOutput,
				-Settings.DEF_INTAKE_FF
		);
	}

	public void reverseOn() {
		if(!kTopBeamBreak.get()) {
			kTowerBelt.set(
					ControlMode.PercentOutput,
					-Settings.DEF_TOWER_PRIME_FF
			);
			kFloorAgitator.set(
					ControlMode.PercentOutput,
					-Settings.DEF_AGITATOR_FF
			);
		} else {
			feedOff();
		}
	}

	public void primeOn() {

		// .get() returns true if no ball is blocking it.
		if (kTopBeamBreak.get()) { // continue unless limit switch triggered
			if (!kBottomBeamBreak.get()) { // if bottom sees a ball, turn on
				qStatus = true;
				kTowerBelt.set(
						ControlMode.PercentOutput,
						Settings.DEF_TOWER_PRIME_FF
				);
//				kAgitatorVictor.set(
//						ControlMode.PercentOutput,
//						Config.kDefaultAgitatorFF
//				); // turn priming on TODO Find out why this was here lol
			} else {
				if (qStatus) { // if first time off, set memory
					qStatus = false;
					qMemory = getTime();
				} else if (getTimeSince(qMemory) > qDelay) {
					// else, if more than queueDelay seconds after, turn off
					feedOff(); // turn priming and feeding off
				}

			}
			kFloorAgitator.set(
					ControlMode.PercentOutput,
					Settings.DEF_AGITATOR_FF
			);
			kEggBeaterAgitator.set(
					ControlMode.PercentOutput,
					-Settings.DEF_EGGBEATER_FF
			);
		} else { // if top limit switch triggers, forget everything and turn off
			feedOff();
		}
	}

	public void shootFeed() {
		if (true /*TODO: isShooterUpToSpeed()*/) {
			kTowerBelt.set(
					ControlMode.PercentOutput,
					Settings.DEF_TOWER_SHOOT_FF
			);
			kFloorAgitator.set(
					ControlMode.PercentOutput,
					Settings.DEF_AGITATOR_FF
			);
			kEggBeaterAgitator.set(
					ControlMode.PercentOutput,
					-Settings.DEF_EGGBEATER_FF
			);
		} else {
			primeOn();
		}
	}

	public void safeShootFeed() {
		if (Subsystems.kLimeLight.hasTarget() && true) { // Subsystems.kShooterHead.isShooterUpToSpeed()
			kTowerBelt.set(
					ControlMode.PercentOutput,
					Settings.DEF_TOWER_SHOOT_FF
			);
			kFloorAgitator.set(
					ControlMode.PercentOutput,
					Settings.DEF_AGITATOR_FF
			);
			kEggBeaterAgitator.set(
					ControlMode.PercentOutput,
					-Settings.DEF_EGGBEATER_FF
			);
		} else {
			primeOn();
		}
	}

	public void feedOff() {
		kFloorAgitator.set(ControlMode.PercentOutput, 0);
		kTowerBelt.set(ControlMode.PercentOutput, 0);
		kEggBeaterAgitator.set(ControlMode.PercentOutput, 0);
	}

	public boolean getBottomLimitSwitch() {
		return kBottomBeamBreak.get();
	}

	public boolean getTopLimitSwitch() {
		return kTopBeamBreak.get();
	}

}
