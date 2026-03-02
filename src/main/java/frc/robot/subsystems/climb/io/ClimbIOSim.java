package frc.robot.subsystems.climb.io;

import static edu.wpi.first.units.Units.Kilograms;
import static frc.robot.subsystems.climb.ClimbConstants.kGearbox;
import static frc.robot.subsystems.climb.ClimbConstants.kGearboxReduction;
import static frc.robot.subsystems.climb.ClimbConstants.kMaxClimberHeight;
import static frc.robot.subsystems.climb.ClimbConstants.kMotorStatorLimitAmps;
import static frc.robot.subsystems.climb.ClimbConstants.kMotorSupplyLimitAmps;
import static frc.robot.subsystems.climb.ClimbConstants.kSpoolRadius;

import com.bobcats.lib.subsystem.bangbangElevator.io.BangBangElevatorIOSim;
import frc.robot.constants.Constants.PhysicalParameters;

/** The IO sim implementation for climb hardware interacions without any hardware. */
public class ClimbIOSim extends BangBangElevatorIOSim implements ClimbIO {
	/**
	 * Constructs a new ClimbIOSim.
	 */
	public ClimbIOSim() {
		super(kGearbox, kGearboxReduction, 1, PhysicalParameters.kRobotMass.in(Kilograms), kSpoolRadius, 0,
				kMaxClimberHeight, 0, kMotorSupplyLimitAmps, kMotorStatorLimitAmps, false);
	}

}
