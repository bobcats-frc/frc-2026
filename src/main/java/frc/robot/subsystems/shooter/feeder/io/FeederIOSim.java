package frc.robot.subsystems.shooter.feeder.io;

import static frc.robot.subsystems.shooter.feeder.FeederConstants.kGearbox;
import static frc.robot.subsystems.shooter.feeder.FeederConstants.kGearboxReduction;
import static frc.robot.subsystems.shooter.feeder.FeederConstants.kMomentOfInertia;
import static frc.robot.subsystems.shooter.feeder.FeederConstants.kMotorStatorLimitAmps;

import com.bobcats.lib.subsystem.rollers.io.GenericRollerIOSim;

/** The IO sim implementation for feeder hardware interacions without any hardware. */
public class FeederIOSim extends GenericRollerIOSim implements FeederIO {
	/**
	 * Constructs a new FeederIOSim.
	 */
	public FeederIOSim() {
		super(kGearbox, kMomentOfInertia, kGearboxReduction, 1, Double.POSITIVE_INFINITY, kMotorStatorLimitAmps, false);
	}
}
