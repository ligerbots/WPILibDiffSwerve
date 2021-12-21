// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
    private static final double kMaxSpeed = Units.feetToMeters(14.7); // 14.7 feet per second
    private static final double kMaxSpeedTurtle = Units.feetToMeters(5.0); // 5.0 feet per second
    private static final double kMaxAngularSpeed = 2 * Math.PI; // rotations per second

    private static final double offsetX = 11.075;
    private static final double offsetY = 12.575;

    /* From swerve.toml */
    private final Translation2d m_frontLeftLocation = new Translation2d(Units.inchesToMeters(offsetX), Units.inchesToMeters(offsetY));
    private final Translation2d m_frontRightLocation = new Translation2d(Units.inchesToMeters(offsetX), Units.inchesToMeters(-offsetY));
    private final Translation2d m_backLeftLocation = new Translation2d(Units.inchesToMeters(-offsetX), Units.inchesToMeters(offsetY));
    private final Translation2d m_backRightLocation = new Translation2d(Units.inchesToMeters(-offsetX), Units.inchesToMeters(-offsetY));

    private final SwerveModuleWrapper m_frontLeft = new SwerveModuleWrapper("frontLeft", 12, 11, 10, 42.7, offsetX, offsetY);
    private final SwerveModuleWrapper m_frontRight = new SwerveModuleWrapper("frontRight", 9, 8, 7, 3.07, offsetX, -offsetY);
    private final SwerveModuleWrapper m_backLeft = new SwerveModuleWrapper("backLeft", 3, 2, 1, -52.38, -offsetX, offsetY);
    private final SwerveModuleWrapper m_backRight = new SwerveModuleWrapper("backRight", 6, 5, 4, -50.54, -offsetX, -offsetY);

    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
            m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

    // Control Objects
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    public Drivetrain() {
        m_gyro.reset();
    }

    @Override
    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), m_frontLeft.getState(), m_frontRight.getState(),
                m_backLeft.getState(), m_backRight.getState());

        // for debug, have the modules populate NetworkTables
        m_frontLeft.populateNetworkTable();
        m_frontRight.populateNetworkTable();
        m_backLeft.populateNetworkTable();
        m_backRight.populateNetworkTable();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    // @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        var swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void driveWithJoystick(double controllerx, double controllery, double controllerrotation, boolean turtlemode,
            boolean fieldRelative) {
        double maxSpeed = turtlemode ? kMaxSpeedTurtle : kMaxSpeed;
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.

        SmartDashboard.putNumber("ControllerX", controllerx);
        SmartDashboard.putNumber("ControllerY", controllery);
        SmartDashboard.putNumber("ControllerRot", controllerrotation);
       
        final var xSpeed = -m_xspeedLimiter.calculate(applyDeadband(controllery, 0.1)) * maxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_yspeedLimiter.calculate(applyDeadband(controllerx, 0.1)) * maxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -m_rotLimiter.calculate(applyDeadband(controllerrotation, 0.1)) * kMaxAngularSpeed;

        this.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    private static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }
}