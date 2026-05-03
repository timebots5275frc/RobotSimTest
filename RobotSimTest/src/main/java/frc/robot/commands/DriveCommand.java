package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {

    private final Drivetrain drivetrain;
    private final DoubleSupplier left;
    private final DoubleSupplier right;

    public DriveCommand(Drivetrain drivetrain, DoubleSupplier left, DoubleSupplier right) {
        this.drivetrain = drivetrain;
        this.left = left;
        this.right = right;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.tankDrive(left.getAsDouble(), right.getAsDouble());
    }
}