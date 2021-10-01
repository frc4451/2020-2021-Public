package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMath;

public class Limelight extends SubsystemBase {

	NetworkTable table;

	Solenoid statusLight;

	private int _pipeline = RobotMath.LimelightValue.Default.val();

	/**
	 * Creates a new LimelightSubsystem.
	 */
	public Limelight(Solenoid _statusLight) {

		table = NetworkTableInstance.getDefault().getTable("limelight");

		statusLight = _statusLight;

//		setPipeDefault();
		setPipeAuto();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void teleOpInit() {
		setPipeTeleOp();
	}

	public void runLimelight() {
		statusLight.set(hasTarget());
	}

	/**
	 *
	 * @return Returns if the limelight is currently detecting a target.
	 */
	public boolean hasTarget() {
		return (table.getEntry("tv").getDouble(0) >= 1);
	}

	/**
	 *
	 * @return Returns the difference (from ~ -28 to 28) between the x position
	 * of the limelights crosshair and the x position of the midpoint of the
	 * target.
	 */
	public double getTargetX() {
		return table.getEntry("tx").getDouble(0);
	}

	/**
	 *
	 * @return Returns the difference (from ~ -24 to 24) between the y position
	 * of the limelights crosshair and the y position of the midpoint of the
	 * target.
	 */
	public double getTargetY() {
		return table.getEntry("ty").getDouble(0);
	}

	/**
	 *
	 * @return Returns the area in percentage of the screen size in 0-1.
	 */
	public double getArea() {
		return table.getEntry("ta").getDouble(0);
	}

	private void setPipeline(int value) {
		table.getEntry("pipeline").setNumber(value);
	}

	public void setPipeAuto() {
		System.out.println("setting auto pipe (4)");
		this._pipeline = RobotMath.LimelightValue.Auto.val();
		this.setPipeline(this._pipeline);
	}

	public void setPipeTeleOp() {
		System.out.println("setting teleOp pipe (9)");
		this._pipeline = RobotMath.LimelightValue.TeleOp.val();
		this.setPipeline(this._pipeline);
	}

	public void setPipeDefault() {
		System.out.println("setting default pipe (0)");
		this._pipeline = RobotMath.LimelightValue.Default.val();
		this.setPipeline(this._pipeline);
	}

	public int getPipeline() {
		return this._pipeline;
	}
}
