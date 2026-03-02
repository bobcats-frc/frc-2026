package frc.robot.subsystems.shooter.feeder.io;

import static frc.robot.subsystems.shooter.feeder.FeederConstants.kMotorID;
import static frc.robot.subsystems.shooter.feeder.FeederConstants.kMotorInverted;
import static frc.robot.subsystems.shooter.feeder.FeederConstants.kMotorStatorLimitAmps;

import com.bobcats.lib.subsystem.rollers.io.GenericRollerIOSparkMax;

/** The IO hardware implementation for feeder hardware interacions with the TalonFX. */
public class FeederIOSparkMax extends GenericRollerIOSparkMax implements FeederIO {
	/**
	 * Constructs a new FeederIOKraken.
	 */
	public FeederIOSparkMax() {
		super(kMotorID, -1, kMotorInverted, false, kMotorStatorLimitAmps);
	}
}
