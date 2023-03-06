// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class GripperSubsystem extends SubsystemBase {
  /** Creates a new GripperSubsystem. */
  CANCoder gripperAbsolute = new CANCoder(ClawConstants.GripperEncoder);

  public GripperSubsystem() {
    gripperAbsolute.configFactoryDefault();
    gripperAbsolute.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    gripperAbsolute.configMagnetOffset(-121.734); // WARNING: DO NOT CHANGE THIS UNLESS ENCODER HAS BEEN REMOVED AND PUT BACK ON
  }// This value sets zero to ~2 degrees 

  PWMVictorSPX gripperMotor = new PWMVictorSPX(0);

// Postive value = close
// Negative value = open
public void driveGripper(double speed){
  //WARNING: Gripper continues traveling 15-30 degrees past
  //max and min position.  Zero is set to where the gripper CANNOT
  //close anymore.  Max is set to 330 so it does not go past 360 which
  //loops back around to zero and breaks the follow logic
  //double maxSpeed = 1;
  double maxClosed = 20;
  double maxOpen = 320;
  //double position = gripperAbsolute.getAbsolutePosition();
  boolean isMaxClosed = gripperAbsolute.getAbsolutePosition() <= maxClosed;
  boolean isMaxOpen = gripperAbsolute.getAbsolutePosition() >= maxOpen;
  boolean gripperClosing = speed > 0;
  boolean gripperOpening = speed < 0;

  //  if(isMaxClosed && gripperClosing){
  //    gripperMotor.set(0);
  //  } else if(isMaxOpen && gripperOpening){
  //    gripperMotor.set(0);
  //    } else {
  //      gripperMotor.set(maxSpeed * speed);
  //      }

   gripperMotor.set(speed);

  SmartDashboard.putBoolean("gripperClosing", gripperClosing);
  SmartDashboard.putBoolean("gripperOpening", gripperOpening);
  SmartDashboard.putBoolean("isMaxClosed", isMaxClosed);
  SmartDashboard.putBoolean("isMaxOpen", isMaxOpen);
  SmartDashboard.putNumber("Gripper Absolute", gripperAbsolute.getAbsolutePosition());

  };

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
