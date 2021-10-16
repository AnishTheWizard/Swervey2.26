// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveForward extends SequentialCommandGroup {
  /** Creates a new MoveForward. */
  public MoveForward() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveTo(new double[]{0.0, 0.0, 0.0}),
      new WaitCommand(0.75),
      new MoveTo(new double[]{0.0, 0.0, -0.75*Math.PI}),
      new LimeLightLineUp(),
      new MoveTo(new double[]{0.0, 10.0, -Math.PI/2}),
      new MoveTo(new double[]{2.0, 10.0, -Math.PI/2}),
      new MoveTo(new double[]{2.0, 10.0, -Math.PI})
      // new MoveTo(new double[]{2.0, 3.0, -Math.PI})
    );
  }
}
