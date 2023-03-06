// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants.ClawPoses;
import frc.robot.subsystems.ClawSubsystem;

public class ArmCommand extends CommandBase {
  private ClawSubsystem clawSubsystem;
  private ClawPoses clawPose;
  
  /** Creates a new ArmCommand. */
  public ArmCommand(ClawSubsystem clawSubsystem, ClawPoses clawPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.clawSubsystem = clawSubsystem;
    this.clawPose = clawPose;
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawSubsystem.setArmState(clawPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return clawSubsystem.getAtArmTarget(8);
  }
}
