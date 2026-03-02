package frc.robot.subsystems.shooter.rollers;

import edu.wpi.first.math.system.plant.DCMotor;

// TODO

/**
 * The constants required for the roller subsystem.
 */
public class RollerConstants {
	// Motor IDs
	public static final int kMotorID = 20;
	public static final String kMotorCANBus = ""; // Main RIO bus

	// Control constants (PID-SVA)
	// Units: Volts, RPM
	public static final double kP = 0.0104, kD = 5.28e-5, kS = 0.0, kV = 0.0015448;
	public static final double kPSim = 0.0104, kDSim = 5.28e-5, kSSim = 0.0, kVSim = 0.0015448;
	public static final boolean kIsFOC = true;

	public static final double kMaxTemperature = 80;

	public static final double kSysIdVoltageRampRate = 0.6;
	public static final double kSysIdVoltageStep = 3;
	public static final double kSysIdTimeout = 5;

	// Soft-max, not the hardware limit RPM
	public static final double kMaxAllowedRPM = 9000;

	// Gearbox reduction
	public static final double kGearboxReduction = 0.75; // A 3:4 ratio (I/O)

	// Current limits for the motors
	public static final double kMotorSupplyLimitAmps = 50.0;
	public static final double kMotorStatorLimitAmps = 90.0;

	// Inversions
	public static final boolean kMotorInverted = false;

	// Simulation data
	public static final DCMotor kGearbox = DCMotor.getKrakenX60Foc(1);
	public static final double kMomentOfInertia = 0.000774324522;
}
