package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightPipeline;

import static frc.robot.Debug.debug;

public class LimeLight extends SubsystemBase {

	/* =================================================================================================================
	 * Internal Objects
	 */

	NetworkTable table;

	Solenoid statusLight;

	private int _pipeline = LimelightPipeline.Default.val();

	/* =================================================================================================================
	 * Constructor
	 */

	public LimeLight(String masterKey, Solenoid _statusLight) {

		table = NetworkTableInstance.getDefault().getTable(masterKey);

		statusLight = _statusLight;

	}

	public void enableInit() {
	}

	public void runLimelight() {
		statusLight.set(hasTarget());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	/* =================================================================================================================
	 * Limelight IO w/ NetworkTables
	 */

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

	public int getPipeline() {
		return this._pipeline;
	}

	public void setPipeDefault() {
		debug("setting Default pipeline (0)");
		this._pipeline = LimelightPipeline.Default.val();
		this.setPipeline(this._pipeline);
	}

	public void setPipeTeleOp() {
		debug("setting NormalAim pipeline (9)");
		this._pipeline = LimelightPipeline.NormalAim.val();
		this.setPipeline(this._pipeline);
	}

}
