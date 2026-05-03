package frc.robot;


import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  // ---------------- SUBSYSTEMS ----------------
  private final Drivetrain m_drivetrain = new Drivetrain();

  // ---------------- CONTROLLER ----------------
  private final CommandXboxController m_driverController =
      new CommandXboxController(0); // USB port 0

  /** The container for the robot. */
  public RobotContainer() {

    // Configure button bindings (optional for now)
    configureBindings();

    // ---------------- DEFAULT DRIVE COMMAND ----------------
    // This runs continuously and controls the drivetrain
    m_drivetrain.setDefaultCommand(
        new DriveCommand(
            m_drivetrain,
            () -> -m_driverController.getLeftY(),   // left side
            () -> -m_driverController.getRightY()   // right side
        )
    );
  }

  /** Define button bindings here */
  private void configureBindings() {

    // Example (optional):
    // m_driverController.a().onTrue(new InstantCommand(() -> System.out.println("A pressed")));

  }

  /** Autonomous command (none for now) */
  public Command getAutonomousCommand() {
    return null;
  }

  /** Getter if you need drivetrain elsewhere */
  public Drivetrain getDrivetrain() {
    return m_drivetrain;
  }
}