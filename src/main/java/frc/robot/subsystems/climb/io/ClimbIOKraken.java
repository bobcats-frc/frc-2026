package frc.robot.subsystems.climb.io;

import static frc.robot.subsystems.climb.ClimbConstants.kIsFOC;
import static frc.robot.subsystems.climb.ClimbConstants.kMotorCANBus;
import static frc.robot.subsystems.climb.ClimbConstants.kMotorID;
import static frc.robot.subsystems.climb.ClimbConstants.kMotorInverted;
import static frc.robot.subsystems.climb.ClimbConstants.kMotorStatorLimitAmps;
import static frc.robot.subsystems.climb.ClimbConstants.kMotorSupplyLimitAmps;

import com.bobcats.lib.subsystem.bangbangElevator.io.BangBangElevatorIOTalonFX;

/** The IO hardware implementation for climb hardware interacions with the TalonFX. */
public class ClimbIOKraken extends BangBangElevatorIOTalonFX implements ClimbIO {
	/**
	 * Constructs a new ClimbIOKraken.
	 */
	public ClimbIOKraken() {
		super(kMotorID, -1, kMotorInverted, false, kMotorSupplyLimitAmps, kMotorStatorLimitAmps, kMotorCANBus, "", 1,
				-1, kIsFOC);
	}
}
