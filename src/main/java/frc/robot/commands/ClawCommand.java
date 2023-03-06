// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClawConstants.ClawPoses;
import frc.robot.subsystems.ClawSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClawCommand extends SequentialCommandGroup {
  /** Creates a new ClawCommand. */
  public ClawCommand(ClawSubsystem clawSubsystem, ClawPoses targetPose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    ClawPoses currentPose = clawSubsystem.getClawState();
    
    //TODO: WaitCommands() are added here for safety.  Remove if (doubtfully) it works

    if(
      (currentPose == ClawPoses.LOW_SCORE || currentPose == ClawPoses.TRANSPORT) 
      && 
      (targetPose == ClawPoses.LOADING || targetPose == ClawPoses.TRANSPORT)){
      addCommands(
        //Arm out -> rotate wrist -> arm in
        new ClawDashboardCommand("Moving to TRANSITION"),
        new PrintCommand("Current Pose: " + currentPose.toString()),
        new PrintCommand("Target Pose: " + targetPose.toString()),
        new PrintCommand("RUNNING OPTION 1 \nWaiting 20s"),
        new WaitCommand(20),
        new PrintCommand("Moving Arm to TRANSITION"),
          new ArmCommand(clawSubsystem, ClawPoses.TRANSITION), 
        new ClawDashboardCommand("Moving to " + targetPose.toString()),
        new PrintCommand("Moving Wrist to " + targetPose.toString()),
          new WristCommand(clawSubsystem, targetPose),
        new PrintCommand("Moving Arm to " + targetPose.toString()),
          new ArmCommand(clawSubsystem, targetPose),
        new PrintCommand("Finished Sequence"),
        new ClawDashboardCommand(targetPose.toString()));
    }
    else if(
      (currentPose == ClawPoses.LOADING) && (targetPose == ClawPoses.LOW_SCORE || targetPose == ClawPoses.TRANSPORT)){
      addCommands(
        // Wrist grab -> arm out -> rotate wrist -> arm in
        new ClawDashboardCommand("Moving to " + targetPose.toString()),
        new PrintCommand("Current Pose: " + currentPose.toString()),
        new PrintCommand("Target Pose: " + targetPose.toString()),
        new PrintCommand("RUNNING OPTION 2 \nWaiting 20s"),
        new WaitCommand(20),
        new PrintCommand("Moving Wrist to GRABBING"),
          new WristCommand(clawSubsystem, ClawPoses.GRABBING),
        new PrintCommand("Moving Arm to TRANSITION"),
          new ArmCommand(clawSubsystem, ClawPoses.TRANSITION), 
        new PrintCommand("Moving Wrist to " + targetPose.toString()),
          new WristCommand(clawSubsystem, targetPose),
        new PrintCommand("Moving Arm to " + targetPose.toString()),
          new ArmCommand(clawSubsystem, targetPose),
        new PrintCommand("Finished Sequence"),
        new ClawDashboardCommand(targetPose.toString()));
    }
    else if(currentPose == ClawPoses.LOADING){
      addCommands(
        //
        new ClawDashboardCommand("Moving to " + targetPose.toString()),
        new PrintCommand("Current Pose: " + currentPose.toString()),
        new PrintCommand("Target Pose: " + targetPose.toString()),
        new PrintCommand("RUNNING OPTION 3 \nWaiting 20s"),
        new WaitCommand(20),
        new PrintCommand("Moving Wrist to GRABBING"),
          new WristCommand(clawSubsystem, ClawPoses.GRABBING),
        new PrintCommand("Moving Arm to " + targetPose.toString()),
          new ArmCommand(clawSubsystem, targetPose), 
        new PrintCommand("Moving Wrist to " + targetPose.toString()),
          new WristCommand(clawSubsystem, targetPose),
        new PrintCommand("Finished Sequence"),
        new ClawDashboardCommand(targetPose.toString()));

    } 
    else {
      addCommands(
        new ClawDashboardCommand("Moving to " + targetPose.toString()),
        new PrintCommand("Current Pose: " + currentPose.toString()),
        new PrintCommand("Target Pose: " + targetPose.toString()),
        new PrintCommand("RUNNING OPTION 4 \nWaiting 20s"),
        new WaitCommand(20),
        new PrintCommand("Moving Arm to " + targetPose.toString()),
          new ArmCommand(clawSubsystem, targetPose),
        new PrintCommand("Moving Wrist to " + targetPose.toString()),
          new WristCommand(clawSubsystem, targetPose),
        new PrintCommand("Finished Sequence"),
        new ClawDashboardCommand(targetPose.toString()));

    }
  }
}
