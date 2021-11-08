// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.electronwill.nightconfig.core.Config;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.team88.swerve.configuration.subconfig.Falcon500Configuration;
import frc.team88.swerve.configuration.subconfig.SensorTransmissionConfiguration;
import frc.team88.swerve.configuration.subconfig.SwerveModuleConfiguration;
import frc.team88.swerve.module.SwerveModule;
import frc.team88.swerve.module.motor.Falcon500;
import frc.team88.swerve.module.motor.SwerveMotor;
import frc.team88.swerve.module.sensor.PositionSensor;
import frc.team88.swerve.module.sensor.SensorTransmission;
import frc.team88.swerve.module.sensor.SwerveCANcoder;

public class SwerveModuleWrapper {

  public SwerveModuleWrapper(int canID0, int canID1, int canIDAzimuth, double offsetAzmuth) {
    Falcon500Configuration falconConfig = new Falcon500Configuration(false, 101.0);
    SwerveMotor motors[] =
            new SwerveMotor[] {
                    new Falcon500(canID0, falconConfig), // motors/0
                    new Falcon500(canID1, falconConfig)  // motors/1
            };
    PositionSensor azimuthSensor =
            new SensorTransmission(new SwerveCANcoder(canIDAzimuth), new SensorTransmissionConfiguration(false,1,offsetAzmuth));
    SwerveModuleConfiguration swerveModuleConfig = new SwerveModuleConfiguration();

    SwerveModule swerveModule = new SwerveModule(motors, azimuthSensor, swerveModuleConfig);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return null;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
  }
}
