package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class holding the Operator Interface (OI) constants. */
public class OIConstants {
	// OI Ports
	public static final int kDriverControllerPort = 0;
	public static final int kOperatorControllerPort = 1;

	// OI Instances
	public static final CommandPS5Controller kDriverController = new CommandPS5Controller(kDriverControllerPort);
	public static final CommandPS5Controller kOperatorController = new CommandPS5Controller(kOperatorControllerPort);

	public static final double kDriverControllerDeadband = 0.05;

	// Driver Keybinds
	public static final Trigger kDriverZeroHeadingKeybind = kDriverController.options();
	public static final Trigger kDriverSlowModeKeybind = kDriverController.circle();
	public static final Trigger kDriverXBrakeKeybind = kDriverController.cross();
	public static final Trigger kDriverIntakeKeybind = kOperatorController.R1();
	public static final Trigger kDriverOuttakeKeybind = kDriverController.L1();
	public static final Trigger kDriverCloseIntakeKeybind = kDriverController.square();

	// Operator Keybinds
	public static final Trigger kOperatorToggleObjectiveKeybind = kOperatorController.triangle();
	public static final Trigger kOperatorScoreHubKeybind = kOperatorController.R1();
	public static final Trigger kOperatorPassFuelKeybind = kOperatorController.L1();
	public static final Trigger kOperatorCorralKeybind = kOperatorController.options();
}
