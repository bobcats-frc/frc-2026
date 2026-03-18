package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The constants required for the intake subsystem.
 */
public class IntakeConstants {
	// Motor IDs
	public static final int kRollerMotorID = 25;
	public static final String kRollerMotorCANBus = ""; // Main RIO bus
	public static final int kArmMotorID = 26;
	public static final String kArmMotorCANBus = "";

	// Control parameters
	public static final boolean kIsFOC = false;
	// Voltage when intaking, ~2000-2500 RPM for the Kraken X60.
	public static final double kIntakeRollerVoltage = 4.5;
	public static final double kOuttakeRollerVoltage = -3.5;
	// Arm PID
	// Units: Volts, Degrees
	public static final double kP = 0.33, kD = 0.001, kG = 0.190229043;
	public static final double kPSim = 0.33, kDSim = 0, kGSim = 0.190229043;
	public static final double kMotionMagicMaxVelocityDegPerSec = 480;
	public static final double kMotionMagicAccelerationDegPerSecSq = 480 * 5.0;
	// The angle at which the encoders are zeroed, the angle at the start of the match
	public static final double kArmCalibrationAngleDeg = 90;
	public static final double kArmClosedAngle = 90, kArmOpenedAngle = 0, kArmPushFuelAngle = 45;
	public static final double kArmMinAngle = 0, kArmMaxAngle = 90;

	public static final double kArmMaxTemperature = 80;
	public static final double kRollerMaxTemperature = 80;

	// TODO exact values
	// Gearbox reduction
	public static final double kRollerGearboxReduction = 20.0 / 12.0; // A 1:1 ratio
	public static final double kArmGearboxReduction = 75; // A 75:1 ratio

	// Current limits for the motor
	public static final double kRollerMotorSupplyLimitAmps = 20.0;
	public static final double kRollerMotorStatorLimitAmps = 50.0;

	public static final double kArmMotorSupplyLimitAmps = 30.0;
	public static final double kArmMotorStatorLimitAmps = 60.0;

	// Inversions
	public static final boolean kRollerMotorInverted = false;
	public static final boolean kArmMotorInverted = true;

	// Simulation data
	// Roller simulation
	public static final DCMotor kRollerGearbox = DCMotor.getKrakenX60Foc(1);
	public static final double kRollerMomentOfInertia = 0.000175;
	// Arm simulation
	public static final DCMotor kArmGearbox = DCMotor.getKrakenX60Foc(1);
	public static final double kArmCenterOfGravityDistance = 0.254765557; // meters
	// MOI of the arm through the pivot axis, unit kg*m^2
	public static final double kArmMomentOfInertiaPivot = 0.310440464;
	// public static final Translation3d kIntakePivotPoseRobotRelative = Translation3d.kZero;
}
