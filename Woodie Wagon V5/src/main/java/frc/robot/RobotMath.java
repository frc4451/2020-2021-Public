package frc.robot;

import edu.wpi.first.wpilibj.Timer;

import static java.lang.Math.*;

public class RobotMath{

	// simple methods used for a lot of things

	// Clamping

	/**
	 * a
	 * @param input a
	 * @param limit a
	 * @return a
	 */
	public static double clampBand(double input, double limit) {
		return (abs(input) <= limit ? input : copySign(limit, input));
	}

	/**
	 *
	 * @param input number being put through
	 * @param center center of range
	 * @param limit radius of range
	 * @return clamped input range from -limit to limit
	 */
	public static double clampBand(double input, double center, double limit) {
		return (abs(input-center) <= limit ? input-center : copySign(limit, input));
	}

	public static double deadBand(double input, double limit) {
		return (abs(input) <= limit ? 0 : input);
	}

	public static double deadBand(double input, double center, double limit) {
		return (abs(input-center) <= limit ? 0 : input);
	}




	public static double square(double input) {
		return copySign(pow(input,2), input);
	}

	public static double pythag(double _a, double _b) {
		return pow(pow(_a,2)+pow(_b,2),.5);
	}


	// time methods

	public static double getTime() {
		return Timer.getFPGATimestamp();
	}

	public static double getTimeSince(double value) {
		return Timer.getFPGATimestamp()-value;
	}

	public static double getMatchTime() {
		return Timer.getMatchTime();
	} // only works in an actual match

}
