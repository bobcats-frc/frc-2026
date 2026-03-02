package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;

// TODO

/**
 * The constants required for the hood subsystem.
 */
public class HoodConstants {
	// Motor IDs
	public static final int kMotorID = 21;
	public static final String kMotorCANBus = ""; // Main RIO bus

	// Control constants (PID-SVA)
	// Units: Volts, Degrees, deg/s
	public static final double kMinAngleDeg = 0, kMaxAngleDeg = 80;
	// M_arm = 0.953kg
	// Voltage gains
	// R: Resistance, kT: Torque coeff., M: Arm mass, r_com: CG distance (meters), G: Reduction
	// I = T / G*kT, I = M*g*r_com / G*kT, V = IR @ w=0,
	// Hence kG at horizontal is approx. R*M*r_com*g / G*kT, V(theta) = kG*cos(theta)
	public static final double kP = 0.14, kD = 0.0036948, kS = 0, kV = 0, kG = 0.3641;
	// public static final double kPSim = 0.18, kDSim = 0.006, kSSim = 0, kVSim = 0, kGSim =
	// 0.3641;
	public static final double kPSim = 0.14, kDSim = 0.0036948, kSSim = 0, kVSim = 0, kGSim = 0.337;// 0.3641;
	// MotionMagic values
	public static final double kMotionMagicMaxVelocityDegPerSec = 720 * 3.0;
	public static final double kMotionMagicAccelerationDegPerSecSq = 1440 * 3.0;
	public static final boolean kIsFOC = true;

	public static final double kMaxTemperature = 80;

	public static final double kExitAngleOffset = 43.02;

	public static final double kSysIdVoltageRampRate = 3;
	public static final double kSysIdVoltageStep = 0.9;
	public static final double kSysIdTimeout = 0.4;

	// The angle at which the hood rests at the start of the match
	// Recommended to set to 0 if possible, the minimum angle otherwise.
	public static final double kHoodCalibrationAngle = kMinAngleDeg;

	// Gearbox reduction
	public static final double kGearboxReduction = 4; // A 4:1 ratio (I/O)

	// Current limits for the motors
	public static final double kMotorSupplyLimitAmps = 50.0;
	public static final double kMotorStatorLimitAmps = 90.0;

	// Inversions
	public static final boolean kMotorInverted = false;

	// Simulation data
	public static final DCMotor kGearbox = DCMotor.getKrakenX44Foc(1);
	public static final double kCenterOfMassDistance = 0.169; // meters
	public static final double kMomentOfInertia = 0.0277 / 4.0;
	public static final Transform3d kHoodCentralPivotRobotRelative = Transform3d.kZero;
}
