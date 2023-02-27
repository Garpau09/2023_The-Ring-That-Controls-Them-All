// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Clawstants;
import frc.robot.subsystems.theCLAAAWWW;
import frc.robot.subsystems.theCLAAAWWW.ClawState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClawCommand extends SequentialCommandGroup {
  /** Creates a new ClawCommand. */
  public ClawCommand(theCLAAAWWW clawSubsystem, ClawState targetState) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double targetWristAngle;
    double targetArmAngle;


    if (targetState == ClawState.LOW){
      targetWristAngle = Clawstants.wristLow;
      targetArmAngle = Clawstants.armLow;
    } else if (targetState == ClawState.MEDIUM){
      targetWristAngle = Clawstants.wristMedium;
      targetArmAngle = Clawstants.armMedium;
    } else if (targetState == ClawState.HIGH){
      targetWristAngle = Clawstants.wristHigh;
      targetArmAngle = Clawstants.armHigh;
    } else if (targetState == ClawState.TRANSPORT){
      targetWristAngle = Clawstants.wristTransport;
      targetArmAngle = Clawstants.armTransport;
    } else{
      targetWristAngle = Clawstants.wristLoading;
      targetArmAngle = Clawstants.armLoading;
    }
    
    // addCommands(
    //   new PrintCommand("RUNNING ONLY OPTION"),
    //   new InstantCommand(() -> clawSubsystem.syncEncoders()),
    //   new InstantCommand(()-> clawSubsystem.setState(targetState)),
    //   new PrintCommand("TWERKING WRIST"),
    //   new WristCommand(clawSubsystem, clawSubsystem.getWristAngle() + 10),
    //   new PrintCommand("ARM TO TRANSITION"),
    //   new ArmCommand(clawSubsystem, Clawstants.armMedium), 
    //   new PrintCommand("Moving Wrist"),
    //   new WristCommand(clawSubsystem, Clawstants.wristLoading),
    //   new PrintCommand("Moving Arm"),
    //   new ArmCommand(clawSubsystem, Clawstants.armLoading),
    //   new PrintCommand("Finished Sequence"));


    if(
      (clawSubsystem.getState() == ClawState.LOW || clawSubsystem.getState() == ClawState.TRANSPORT) 
      && 
      (targetState == ClawState.LOADING || targetState == ClawState.TRANSPORT)){
      addCommands(
        new PrintCommand("RUNNING OPTION 1"),
        new InstantCommand(() -> clawSubsystem.syncEncoders()),
        new PrintCommand("Moving Arm"),
        new ArmCommand(clawSubsystem, Clawstants.armTransition), 
        new PrintCommand("Moving Wrist"),
        new WristCommand(clawSubsystem, Clawstants.wristLoading),
        new PrintCommand("Moving Arm"),
        new ArmCommand(clawSubsystem, Clawstants.armLoading),
        new PrintCommand("Finished Sequence"));
    }
    else if(
      (clawSubsystem.getState() == ClawState.LOADING) && (targetState == ClawState.LOW || targetState == ClawState.TRANSPORT)){
      addCommands(
        new PrintCommand("RUNNING OPTION 2"),
        new InstantCommand(() -> clawSubsystem.syncEncoders()),
        new WristCommand(clawSubsystem, Clawstants.wristGrabbed),
        new ArmCommand(clawSubsystem, Clawstants.armTransition), 
        new WristCommand(clawSubsystem, targetWristAngle),
        new ArmCommand(clawSubsystem, targetArmAngle));
    }
    else if(clawSubsystem.getState() == ClawState.LOADING){
      addCommands(
        new PrintCommand("RUNNING OPTION 3"),
        new PrintCommand("clawState: " + clawSubsystem.clawState.name()),
        new PrintCommand("targetState: " + targetState),
        new InstantCommand(() -> clawSubsystem.syncEncoders()),
        new WristCommand(clawSubsystem, Clawstants.wristGrabbed),
        new ArmCommand(clawSubsystem, targetArmAngle), 
        new WristCommand(clawSubsystem, targetWristAngle));
    } else{
      addCommands(
        new PrintCommand("RUNNING OPTION 4"),
        new InstantCommand(() -> clawSubsystem.syncEncoders()),
        new ArmCommand(clawSubsystem, targetArmAngle),
        new WristCommand(clawSubsystem, targetWristAngle));
    }

    
    
  // }
  
  // public ClawCommand(theCLAAAWWW clawSubsystem, double angle){
  //   addCommands(new ArmCommand(clawSubsystem, angle));
  // }
  }
}
