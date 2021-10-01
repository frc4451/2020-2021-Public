package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Debug.*;

import static frc.robot.Constants.*;
import static frc.robot.RobotContainer.*;

public class ColorWheel extends SubsystemBase {

	private VictorSPX kColorWheelVictor;

	public ColorWheel(VictorSPX _kColorWheelVictor) {
		kColorWheelVictor = _kColorWheelVictor;

		kColorWheelVictor.configFactoryDefault();

		kColorWheelVictor.setInverted(true);

	}

	public void enableInit() {
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void runColorWheel() {
		debug("running Color Wheel", "debug");
		if (IO.Driver.lTrig() > Constants.Config.DEF_DEADBAND) {
			kColorWheelVictor.set(ControlMode.PercentOutput, Settings.DEF_WHEEL_FF);
			debug("setting Color Wheel", "debug");
		} else {
			kColorWheelVictor.set(ControlMode.PercentOutput, 0);
		}
	}

}
