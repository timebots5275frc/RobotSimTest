package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;


import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;

public class CANDriveSubsystem extends SubsystemBase {

    // ---------------- MOTORS ----------------
    public static SparkFlex leftLeader;
    private SparkFlex leftFollower;
    public static SparkFlex rightLeader;
    private SparkFlex rightFollower;

    private final DifferentialDrive drive;

    // ---------------- SIM ----------------
    private DifferentialDrivetrainSim sim;
    private DifferentialDriveOdometry odometry;

    public CANDriveSubsystem() {

        leftLeader = new SparkFlex(Constants.DriveConstants.LEFT_LEADER_ID, MotorType.kBrushless);
        leftFollower = new SparkFlex(Constants.DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushless);
        rightLeader = new SparkFlex(Constants.DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushless);
        rightFollower = new SparkFlex(Constants.DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushless);

        drive = new DifferentialDrive(leftLeader, rightLeader);

        // ---------------- CONFIG ----------------
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.voltageCompensation(12);
        config.smartCurrentLimit(
            Constants.DriveConstants.DRIVE_MOTOR_STALL_LIMIT,
            Constants.DriveConstants.DRIVE_MOTOR_FREE_LIMIT
        );
        config.closedLoopRampRate(Constants.DriveConstants.DRIVE_MOTOR_RAMP_RATE);

        config.follow(leftLeader);
        leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(rightLeader);
        rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.disableFollowerMode();
        rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.inverted(true);
        leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ---------------- SIM SETUP ----------------
        if (RobotBase.isSimulation()) {

            sim = new DifferentialDrivetrainSim(
                DCMotor.getNEO(2),   // using NEO motors
                10.71,
                7.5,
                60.0,
                0.0762,
                0.6,
                null
            );

            odometry = new DifferentialDriveOdometry(sim.getHeading(), 0, 0);
        }
    }

    // ---------------- DRIVE ----------------
    public void driveArcade(double xSpeed, double zRotation) {
        drive.arcadeDrive(xSpeed, zRotation);
    }

    // ---------------- POSE ----------------
    public Pose2d getPose() {
        return odometry != null ? odometry.getPoseMeters() : new Pose2d();
    }

    // ---------------- PERIODIC ----------------
    @Override
    public void periodic() {

        if (RobotBase.isSimulation()) {

            odometry.update(
                sim.getHeading(),
                sim.getLeftPositionMeters(),
                sim.getRightPositionMeters()
            );

            // Log to AdvantageScope
            Logger.recordOutput("Drivetrain/Pose", getPose());
            
        }
    }

    // ---------------- SIMULATION ----------------
    @Override
    public void simulationPeriodic() {

        if (!RobotBase.isSimulation()) return;

        // Apply motor outputs → volts
        sim.setInputs(
            leftLeader.get() * 12.0,
            rightLeader.get() * 12.0
        );

        // Advance physics
        sim.update(0.02);
    }

    public DifferentialDrive getDifferentialDrive() {
      return drive;
    }
}