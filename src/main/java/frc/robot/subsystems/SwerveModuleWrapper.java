// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.electronwill.nightconfig.core.Config;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import frc.team88.swerve.configuration.subconfig.Falcon500Configuration;
import frc.team88.swerve.configuration.subconfig.SensorTransmissionConfiguration;
import frc.team88.swerve.configuration.subconfig.SwerveModuleConfiguration;
import frc.team88.swerve.module.SwerveModule;
import frc.team88.swerve.module.motor.Falcon500;
import frc.team88.swerve.module.motor.SwerveMotor;
import frc.team88.swerve.module.sensor.PositionSensor;
import frc.team88.swerve.module.sensor.SensorTransmission;
import frc.team88.swerve.module.sensor.SwerveCANcoder;
import frc.team88.swerve.util.WrappedAngle;

import java.util.List;

public class SwerveModuleWrapper {

  SwerveModule swerveModule;
  public SwerveModuleWrapper(
          int canID0,
          int canID1,
          int canIDAzimuth,
          double offsetAzimuth,
          double location_x,
          double location_y
  ) {
    Falcon500Configuration falconConfig = new Falcon500Configuration(
            makeFalcon500Config(false, 101.0)
    );

    SwerveMotor motors[] =
            new SwerveMotor[] {
                    new Falcon500(canID0, falconConfig), // motors/0
                    new Falcon500(canID1, falconConfig)  // motors/1
            };
    PositionSensor azimuthSensor =
            new SensorTransmission(
                    new SwerveCANcoder(canIDAzimuth),
                    new SensorTransmissionConfiguration(
                            makeSensorTransmissionConfig(false,1,offsetAzimuth)
                    )
            );

    Config config = Config.inMemory();
    config.set("location-inches.x", location_x);
    config.set("location-inches.y", location_y);
    config.set("differential-matrix", List.of(
            List.of(0.041666666, -0.041666666),
            List.of(0.069444444, 0.09722222)
    ));
    config.set("wheel-diameter-inches",3.5);

    config.set("azimuth-controller.kP", 20.0);
    config.set("azimuth-controller.kI", 0.0);
    config.set("azimuth-controller.kD", 0.0);
    config.set("azimuth-controller.kF", 0.0);
    config.set("azimuth-controller.i-zone", 0.0);
    config.set("azimuth-controller.i-max", 0.0);
    config.set("azimuth-controller.max-speed", 1440);
    config.set("azimuth-controller.max-acceleration", 1080);

    config.set("wheel-controller.kP", 0.0);
    config.set("wheel-controller.kI", 0.1);
    config.set("wheel-controller.kD", 0.0);
    config.set("wheel-controller.i-zone", 1.5);
    config.set("wheel-controller.i-max", 0.0);

    SwerveModuleConfiguration swerveModuleConfig = new SwerveModuleConfiguration(config);

    swerveModule = new SwerveModule(motors, azimuthSensor, swerveModuleConfig);
  }

  private Config makeFalcon500Config(boolean inverted, double maxSpeed){
    Config config = Config.inMemory();
    config.set("inverted", inverted);
    config.set("max-speed-rps", maxSpeed);
    return config;
  }
  private Config makeSensorTransmissionConfig(boolean inverted, double ratio, double offset){
    Config config = Config.inMemory();
    config.set("inverted", inverted);
    config.set("ratio", ratio);
    config.set("offset", offset);
    return config;
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // getWheelVelocity returns speed in feet per second but SwerveModuleState wants meters per second
    // WrappedAngle.asDouble returns angle in degrees
    SwerveModuleState swerveModuleState = new SwerveModuleState(
            Units.feetToMeters(swerveModule.getWheelVelocity()),
            Rotation2d.fromDegrees(swerveModule.getAzimuthPosition().asDouble())
    );

    return swerveModuleState;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // wheelVelocity is in feet per second
    double wheelVelocity = Units.metersToFeet(desiredState.speedMetersPerSecond);
    // WrappedAngle accepts degrees in constructor
    WrappedAngle azimuthPosition = new WrappedAngle(desiredState.angle.getDegrees());
    swerveModule.set(wheelVelocity, azimuthPosition);
  }
}
