package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



public class Robot extends LoggedRobot {

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  @Override
  public void robotInit() {
    // Start camera (optional)
    CameraServer.startAutomaticCapture();

    Logger.addDataReceiver(new WPILOGWriter());
    Logger.addDataReceiver(new NT4Publisher());

    Logger.start();

    // Create robot container (this sets up subsystems + commands)
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // Runs command-based framework
    CommandScheduler.getInstance().run();
  }

  // ---------------- DISABLED ----------------

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  // ---------------- AUTONOMOUS ----------------

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  // ---------------- TELEOP ----------------

  @Override
  public void teleopInit() {
    // Stop auton when teleop starts
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  // ---------------- TEST ----------------

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  // ---------------- SIMULATION ----------------

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}