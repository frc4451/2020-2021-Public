
/*
 *  Class that organizes gains used when assigning values to slots
 */
package frc.robot;

public class Gains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final int kIzone;
	public final double kPeakOutput;
	public final double kS;
	public final double kV;
	public final double kA;

	public Gains (
			double _kP,
			double _kI,
			double _kD,
			double _kF,
			double _kS,
			double _kV,
			double _kA,
			int _kIzone,
			double _kPeakOutput
			) {
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kS = _kS;
		kV = _kV;
		kA = _kA;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
	}
}
