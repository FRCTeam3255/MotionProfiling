package frc.robot;

public class Constants {
	
	/**
	 * How many sensor units per rotation.
	 * Using CTRE Magnetic Encoder.
	 * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
	 */
	public final static int kSensorUnitsPerRot = 652;
	
	/**
	 * Motor neutral dead-band, set to the minimum 0.1%.
	 */
	public final static double kNeutralDeadband = 0.001;
	
	/**
	 * Pigeon will reports 8192 units per 360 deg (1 rotation)
	 * If using encoder-derived (left plus/minus right) heading, find this emperically.
	 */
	public final static double kTurnUnitsPerDeg = 8192.0 / 360.0;


	public final static String Digits     = "(\\p{Digit}+)";
	public final static String HexDigits  = "(\\p{XDigit}+)";
	// an exponent is 'e' or 'E' followed by an optionally 
	// signed decimal integer.
	public final static String Exp        = "[eE][+-]?"+Digits;
	public final static String fpRegex    =
		("[\\x00-\\x20]*"+ // Optional leading "whitespace"
		"[+-]?(" +         // Optional sign character
		"NaN|" +           // "NaN" string
		"Infinity|" +      // "Infinity" string
	
		// A decimal floating-point string representing a finite positive
		// number without a leading sign has at most five basic pieces:
		// Digits . Digits ExponentPart FloatTypeSuffix
		// 
		// Since this method allows integer-only strings as input
		// in addition to strings of floating-point literals, the
		// two sub-patterns below are simplifications of the grammar
		// productions from the Java Language Specification, 2nd 
		// edition, section 3.10.2.
	
		// Digits ._opt Digits_opt ExponentPart_opt FloatTypeSuffix_opt
		"((("+Digits+"(\\.)?("+Digits+"?)("+Exp+")?)|"+
	
		// . Digits ExponentPart_opt FloatTypeSuffix_opt
		"(\\.("+Digits+")("+Exp+")?)|"+
	
		// Hexadecimal strings
		"((" +
		// 0[xX] HexDigits ._opt BinaryExponent FloatTypeSuffix_opt
		"(0[xX]" + HexDigits + "(\\.)?)|" +
	
		// 0[xX] HexDigits_opt . HexDigits BinaryExponent FloatTypeSuffix_opt
		"(0[xX]" + HexDigits + "?(\\.)" + HexDigits + ")" +
	
		// 0[xX] HexDigits_opt . HexDigits BinaryExponent FloatTypeSuffix_opt
		"(0[xX]" + HexDigits + "?(\\.)" + HexDigits + ")" +
	
		")[pP][+-]?" + Digits + "))" +
		"[fFdD]?))" +
		"[\\x00-\\x20]*");// Optional trailing "whitespace"
	
			
	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop
	 * 	                                    			  kP   kI    kD     kF             Iz    PeakOut */
	public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/6800.0,  400,  1.00 ); /* measured 6800 velocity units at full motor output */
	
	public final static int kPrimaryPIDSlot = 0; // any slot [0,3]
	public final static int kAuxPIDSlot = 1; // any slot [0,3]
}
