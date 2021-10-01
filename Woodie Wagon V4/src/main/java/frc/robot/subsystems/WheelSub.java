
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants;

import static frc.robot.Debug.*;
import static frc.robot.RobotContainer.kDriver;

public class WheelSub extends SubsystemBase {

	private final TalonSRX kWheelTalon;

	/**
	 * Creates a new WheelSub.
	 */
	public WheelSub(TalonSRX kWTalon) {
		debug("turret subsystem initializing: " + this.toString());

		this.kWheelTalon = kWTalon;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void enableInit() {

	}

	/**
	 * Run Color Wheel iteration
	 */
	public void runWheel() {

		if (kDriver.getTriggerAxis(GenericHID.Hand.kLeft)
				>= Constants.Config.kDefaultDeadband) {
			spin(Constants.Config.kDefaultWheelFF);
		}
		else {
			spin(0);}

	}

	private void spin(double value) {
		kWheelTalon.set(ControlMode.PercentOutput, value);
	}
}
