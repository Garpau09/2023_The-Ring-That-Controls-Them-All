// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ClawConstants.ClawPoses;
import frc.robot.Objects.Arm;
import frc.robot.Objects.Wrist;

public class ClawSubsystem extends SubsystemBase {

  public HashMap<ClawPoses, double[]> clawStates = ClawConstants.kClawStates;

  /** Used to track the state of the claw as a whole */
  ClawPoses targetClawState;
  /** Allows us to set wrist and arm states individually */
  ClawPoses targetWristState;
  ClawPoses targetArmState;

  private Wrist mWrist;
  private Arm mArm;

  public ClawSubsystem() {

    for(ClawPoses pose : ClawPoses.values()){
      try {
        double x = 0;
        x = x + clawStates.get(pose)[0];
        x = x + clawStates.get(pose)[1];
      } catch(Exception exception) {
        throw new IndexOutOfBoundsException(
          "NOT ALL CLAW POSES HAVE A VALUE IN THE HASHMAP!  THIS WILL RESULT IN CRASHING IF NOT RESOLVED \nCHECK CLAW CONSTANTS");
      }
    }

    double[] wristPID = {.5, 0, 0};
    double[] armPID = {.5, 0, 0};

    mWrist = new Wrist(wristPID, 0.5, 6000, 7500);
    mArm = new Arm(armPID, .4, 6000, 7000);
  }

  public void setWristState(final ClawPoses state){
    targetWristState = state;
    targetClawState = state; // Keeps track of claw as a whole

    mWrist.setTargetTheta(clawStates.get(targetClawState)[0]);
  }

  public void setArmState(final ClawPoses state){
    targetArmState = state;
    targetClawState = state; // Keeps track of claw as a whole

    mArm.setTargetTheta(clawStates.get(targetClawState)[1]);
  }

  public boolean getAtWristTarget(double deadBand){
    return mWrist.getAtTarget(deadBand);
  }

  public boolean getAtArmTarget(double deadband){
    return mArm.getAtTarget(deadband);
  }

  public void syncWristEncoders(){
    mWrist.syncEncoders();
  }

  public ClawPoses getClawState(){
    return targetClawState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("targetClawState", targetClawState.toString());
    SmartDashboard.putString("targetArmState", targetArmState.toString());
    SmartDashboard.putString("targetWristState", targetWristState.toString());
  }
}
