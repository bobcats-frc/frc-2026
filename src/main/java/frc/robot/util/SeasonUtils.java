package frc.robot.util;

import com.bobcats.lib.utils.AllianceUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A 2026 season-specific utility class.
 */
public class SeasonUtils {
	/**
	 * Returns whether the blue alliance's hub is active.
	 *
	 * @return Whether the blue alliance's hub is active.
	 */
	public static boolean isBlueAllianceHubActive() {
		// Always enabled in autonomous
		if (DriverStation.isAutonomous()) return true;

		double time = DriverStation.getMatchTime();
		if (time < 0) return true;
		// Whether the blue alliance scored the most in auton
		boolean isBlueAlliance = DriverStation.getGameSpecificMessage().startsWith("B");

		if (DriverStation.getGameSpecificMessage().length() == 0)
			// No auton data; assume both hubs are active
			return true;

		// Transition period
		if (time > 130) return true;
		// Shift 1
		else if (time > 105) return isBlueAlliance;
		// Shift 2
		else if (time > 80) return !isBlueAlliance;
		// Shift 3
		else if (time > 55) return isBlueAlliance;
		// Shift 4
		else if (time > 30) return !isBlueAlliance;
		// Endgame
		else return true;
	}

	/**
	 * Returns whether the red alliance's hub is active.
	 *
	 * @return Whether the red alliance's hub is active.
	 */
	public static boolean isRedAllianceHubActive() {
		// Always enabled in autonomous
		if (DriverStation.isAutonomous()) return true;

		double time = DriverStation.getMatchTime();
		if (time < 0) return true;
		// Whether the blue alliance scored the most in auton
		boolean isBlueAlliance = DriverStation.getGameSpecificMessage().startsWith("B");

		if (DriverStation.getGameSpecificMessage().length() == 0)
			// No auton data; assume both hubs are active
			return true;

		// Transition period
		if (time > 130) return true;
		// Shift 1
		else if (time > 105) return !isBlueAlliance;
		// Shift 2
		else if (time > 80) return isBlueAlliance;
		// Shift 3
		else if (time > 55) return !isBlueAlliance;
		// Shift 4
		else if (time > 30) return isBlueAlliance;
		// Endgame
		else return true;
	}

	/**
	 * Returns the time remaining until the current alliance's hub becomes activated.
	 *
	 * @return The time remaining until the current alliance's hub becomes activated.
	 */
	public static double getTimeUntilHubShift() {
		// Always enabled in autonomous
		if (DriverStation.isAutonomous()) return 0;

		double time = DriverStation.getMatchTime();
		if (time < 0) return 0;
		// Whether the blue alliance scored the most in auton
		boolean isBlueAlliance = DriverStation.getGameSpecificMessage().startsWith("B");

		if (DriverStation.getGameSpecificMessage().length() == 0)
			// No auton data; assume both hubs are active
			return 0;

		if (AllianceUtil.isRedAlliance()) {
			// Transition period
			if (time > 130) return 0;
			// Shift 1
			else if (time > 105) return isBlueAlliance ? time - 105 : 0;
			// Shift 2
			else if (time > 80) return !isBlueAlliance ? time - 80 : 0;
			// Shift 3
			else if (time > 55) return isBlueAlliance ? time - 55 : 0;
			// Shift 4
			else if (time > 30) return !isBlueAlliance ? time - 30 : 0;
			// Endgame
			else return 0;
		} else {
			// Transition period
			if (time > 130) return 0;
			// Shift 1
			else if (time > 105) return !isBlueAlliance ? time - 105 : 0;
			// Shift 2
			else if (time > 80) return isBlueAlliance ? time - 80 : 0;
			// Shift 3
			else if (time > 55) return !isBlueAlliance ? time - 55 : 0;
			// Shift 4
			else if (time > 30) return isBlueAlliance ? time - 30 : 0;
			// Endgame
			else return 0;
		}
	}

	/**
	 * Returns whether the current alliance's hub is active.
	 *
	 * @return Whether the current alliance's hub is active.
	 */
	public static boolean isAllianceHubActive() {
		return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? isBlueAllianceHubActive()
				: isRedAllianceHubActive();
	}
}
