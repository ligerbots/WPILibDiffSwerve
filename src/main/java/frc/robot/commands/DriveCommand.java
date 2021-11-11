package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    /** Creates a new DriveCommand. */
    Drivetrain driveTrain;

    DoubleSupplier xSpeed;
    DoubleSupplier ySpeed;
    DoubleSupplier rot;
    BooleanSupplier turtleMode;

    public DriveCommand(
            Drivetrain driveTrain,
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier rot,
            BooleanSupplier turtleMode) {
        this.driveTrain = driveTrain;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rot = rot;
        this.turtleMode = turtleMode;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Use squared inputs
        // Might want to change that for competition
        driveTrain.driveWithJoystick(
                xSpeed.getAsDouble(),
                ySpeed.getAsDouble(),
                rot.getAsDouble(),
                turtleMode.getAsBoolean(),
                true
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
