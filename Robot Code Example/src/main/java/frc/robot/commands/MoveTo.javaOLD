// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainOld;

public class MoveTo extends CommandBase {
  /** Creates a new MoveTo. */
  double[] target;
  int sustain;
  public MoveTo(double[] target) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveTrainOld.getInstance());
    this.target = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveTrainOld.getInstance().lockDriveTrain();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveTrainOld.getInstance().toPose(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveTrainOld.getInstance().unlockDriveTrain();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(DriveTrainOld.getInstance().atSetpoint()) {
      sustain++;
    }
    else {
      sustain = 0;
    }
    return sustain >= DriveTrainOld.getInstance().getCurrentSpeedMultiplier() * 10;
  }
}
