// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.util.Units;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private static final double offsetX = 11.075;
  private static final double offsetY = 12.575;
  /*From swerve.toml*/
  private final Translation2d m_frontLeftLocation = new Translation2d(Units.inchesToMeters(offsetX), Units.inchesToMeters(offsetY));
  private final Translation2d m_frontRightLocation = new Translation2d(Units.inchesToMeters(offsetX), Units.inchesToMeters(-offsetY));
  private final Translation2d m_backLeftLocation = new Translation2d(Units.inchesToMeters(-offsetX), Units.inchesToMeters(offsetY));
  private final Translation2d m_backRightLocation = new Translation2d(Units.inchesToMeters(-offsetX), Units.inchesToMeters(-offsetY));

  private final SwerveModuleWrapper m_frontLeft = new SwerveModuleWrapper(
          12, 11, 10, 42.7, offsetX, offsetY
  );
  private final SwerveModuleWrapper m_frontRight = new SwerveModuleWrapper(
          9,8,7, 3.07, offsetX, -offsetY
  );
  private final SwerveModuleWrapper m_backLeft = new SwerveModuleWrapper(
          3,2,1,-52.38, -offsetX, offsetY
  );
  private final SwerveModuleWrapper m_backRight = new SwerveModuleWrapper(
          6,5,4, -50.54, -offsetX, -offsetY
  );

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

      
  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }
}
