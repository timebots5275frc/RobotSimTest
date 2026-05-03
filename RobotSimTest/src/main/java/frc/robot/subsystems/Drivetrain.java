package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    // ---------------- MOTORS ----------------
    private final SparkMax leftMotor = new SparkMax(1, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(2, MotorType.kBrushless);

    // ---------------- SENSORS ----------------
    private final Encoder leftEncoder = new Encoder(0, 1);
    private final Encoder rightEncoder = new Encoder(2, 3);
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    // ---------------- SIM OBJECTS ----------------
    private final EncoderSim leftEncoderSim;
    private final EncoderSim rightEncoderSim;
    private final ADXRS450_GyroSim gyroSim;

    private final DifferentialDrivetrainSim sim;

    // ---------------- ODOMETRY ----------------
    private final DifferentialDriveOdometry odometry;

    public Drivetrain() {

        // Simulation physics model
        sim = new DifferentialDrivetrainSim(
            DCMotor.getCIM(2),   // motors per side
            10.71,               // gear ratio
            7.5,                 // moment of inertia
            60.0,                // mass (kg)
            0.0762,              // wheel radius (meters)
            0.6,                 // track width (meters)
            null
        );

        // Sim wrappers for sensors
        leftEncoderSim = new EncoderSim(leftEncoder);
        rightEncoderSim = new EncoderSim(rightEncoder);
        gyroSim = new ADXRS450_GyroSim(gyro);

        // Odometry
        odometry = new DifferentialDriveOdometry(
            gyro.getRotation2d(),
            leftEncoder.getDistance(),
            rightEncoder.getDistance()
        );
    }

    // ---------------- DRIVE ----------------
    public void tankDrive(double left, double right) {
        leftMotor.set(left);
        rightMotor.set(right);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // ---------------- PERIODIC ----------------
    @Override
    public void periodic() {

        // Update odometry
        odometry.update(
            gyro.getRotation2d(),
            leftEncoder.getDistance(),
            rightEncoder.getDistance()
        );

        // Log for AdvantageScope
        Logger.recordOutput("Drivetrain/Pose", getPose());
        Logger.recordOutput("Drivetrain/LeftSpeed", leftMotor.get());
        Logger.recordOutput("Drivetrain/RightSpeed", rightMotor.get());
    }

    // ---------------- SIMULATION ----------------
    @Override
    public void simulationPeriodic() {

        // Apply motor outputs as voltages
        sim.setInputs(
            leftMotor.get() * 12.0,
            rightMotor.get() * 12.0
        );

        // Advance physics
        sim.update(0.02);

        // Update simulated sensors
        leftEncoderSim.setDistance(sim.getLeftPositionMeters());
        rightEncoderSim.setDistance(sim.getRightPositionMeters());

        gyroSim.setAngle(sim.getHeading().getDegrees());
    }
}