package frc.robot.subsystems.shooter.feeder;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The constants required for the feeder subsystem.
 */
public class FeederConstants {
	// Motor IDs
	public static final int kMotorID = 22;

	// Control parameters
	// Voltage when intaking, ~2200 RPM for the NEO v1.1.
	public static final double kFeedVoltage = 4.5;
	public static final double kUnfeedVoltage = -3.5;

	// Gearbox reduction
	public static final double kGearboxReduction = 0.75; // A 3:4 ratio (I/O)

	// Current limit for the motor
	public static final int kMotorStatorLimitAmps = 25;

	// Inversions
	public static final boolean kMotorInverted = false;

	// Simulation data
	public static final DCMotor kGearbox = DCMotor.getNEO(1);
	public static final double kMomentOfInertia = 0.000175;
}
