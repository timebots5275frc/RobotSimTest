package frc.robot;


import frc.robot.commands.DriveCommand;
import frc.robot.commands.TeleopJoystickDrive;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Input.Input;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  CANDriveSubsystem driveTrain;
  GenericHID bBoard;
  Input input;
  Joystick joy;
  TeleopJoystickDrive teleJoyDrive;

  /** The container for the robot. */
  public RobotContainer() {

    bBoard = new GenericHID(1);
    joy = new Joystick(0);
    input = new Input(joy);
    driveTrain = new CANDriveSubsystem();

    // Configure button bindings (optional for now)
    configureBindings();

    // ---------------- DEFAULT DRIVE COMMAND ----------------
    // This runs continuously and controls the drivetrain
   
  }

  /** Define button bindings here */
  private void configureBindings() {
    teleJoyDrive = new TeleopJoystickDrive(driveTrain, input, false, 1);
    driveTrain.setDefaultCommand(teleJoyDrive);

    // Example (optional):
    // m_driverController.a().onTrue(new InstantCommand(() -> System.out.println("A pressed")));

  }

  /** Autonomous command (none for now) */
  public Command getAutonomousCommand() {
    return null;
  }

  
  
}