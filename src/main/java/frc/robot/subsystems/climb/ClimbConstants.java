package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The constants required for the climb subsystem.
 */
public class ClimbConstants {
	// Motor ID
	public static final int kMotorID = 3;
	public static final String kMotorCANBus = ""; // Main RIO bus

	// Control parameters
	public static final boolean kIsFOC = true;
	// Units: Volts, RPM, seconds
	public static final double kForwardOutput = 11;
	// This is the motor RPM, not the output!
	public static final double kStallVelocityThreshold = 0.5;
	public static final double kStallTimeThreshold = 0.4;
	// Elevator hit output, must hold the entire robot
	// Use the formula V = R*M*g*r_drum / G*kT for manual calculations
	// Otherwise try until a valid voltage holds the robot without too much current
	public static final double kG = 0.252968528; // 0.79052665;
	// Wait a bit when extended to avoid any encoder/lag issues
	public static final double kAutonClimbSequenceRetractIntermission = 0.5;

	public static final double kMaxTemperature = 80;

	// Current limits for the motor
	public static final double kMotorSupplyLimitAmps = 60.0;
	public static final double kMotorStatorLimitAmps = 120.0;

	// Inversions
	public static final boolean kMotorInverted = false;

	// Simulation data
	public static final DCMotor kGearbox = DCMotor.getKrakenX60Foc(1);
	// Gearbox reduction
	public static final double kGearboxReduction = 250; // A 250:1 ratio (I/O)
	// 19mm spool diameter
	public static final double kSpoolRadius = 0.019 / 2.0;
	public static final double kMaxClimberHeight = 0.135;
	// public static final double kClimberMass = 0.5;
}
