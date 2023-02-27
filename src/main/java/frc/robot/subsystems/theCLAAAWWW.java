// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Arm;
import frc.robot.Wrist;
import frc.robot.Constants.Clawstants;

public class theCLAAAWWW extends SubsystemBase {
  /** Creates a new theCLAAAWWW. */
  public enum ClawState {
  
    TRANSPORT, LOADING, LOW, MEDIUM, HIGH
  }

  Arm arm = Arm.getInstance();
  Wrist wrist = Wrist.getInstance();
  public ClawState clawState = getState();


  // TODO Dont forget that EVERY thing needs a god forbidden PID.


  public theCLAAAWWW() {
    System.out.println("CONSTRUCTOR STATE: " + clawState.name());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Reset Encoder", arm.isResetEncoderPushed());
    if(arm.isResetEncoderPushed()){
      arm.zeroEncoder();
    }

    clawState = getState();
    
    SmartDashboard.putString("ClawState", clawState.name());

    SmartDashboard.putNumber("Arm Angle", arm.getAngle());
    SmartDashboard.putNumber("Arm Encoder", arm.getRawEncoderUnits());

    SmartDashboard.putNumber("Wrist Absolute", wrist.getWristAbsolute());
    SmartDashboard.putNumber("Wrist Angle", wrist.getWristAngle());
    SmartDashboard.putNumber("Wrist Encoder", wrist.getRawEncoderUnits());
    
  }

public void syncEncoders(){
  wrist.syncEncoders();
}

public void drive(double speed, double wristSpeed){
  arm.drive(speed * 0.25);
  wrist.drive(wristSpeed * 0.25);

}

public void setArmAngle(double armAngle){
  arm.setAngle(armAngle);
}

public void setWristAngle(double wristAngle){
  wrist.setAngle(wristAngle);
}

public double getWristAngle(){
  return wrist.getWristAngle();
}

public double getArmAngle(){
  return arm.getAngle();
}

public ClawState getState(){

  if(arm.getAngle() <= 10 && wrist.getWristAbsolute() > Clawstants.wristMedium){
    return ClawState.LOADING;
  } 
  else if (arm.getAngle() <= 10 && wrist.getWristAbsolute() < Clawstants.wristMedium){
    return ClawState.TRANSPORT;
  } 
  else if (arm.getAngle() <= 43){
    return ClawState.LOW;
  }
  else if(arm.getAngle() > 43 && arm.getAngle() < 82.5){
    return ClawState.MEDIUM;
  }
  else if(arm.getAngle() >= 82.5){
    return ClawState.HIGH;
  }

  return null;
}





}