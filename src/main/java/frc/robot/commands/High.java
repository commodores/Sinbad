// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class High extends SequentialCommandGroup {
  public final Elevator m_Elevator;
  public final Extender m_Extender;
  public final Wrist m_Wrist;
  /** Creates a new Ground. */
  public High(Extender extenderSusbsystem, Elevator elevatorSubsystem, Wrist wristSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_Extender = extenderSusbsystem;
    m_Elevator = elevatorSubsystem;
    m_Wrist = wristSubsystem;
    addCommands(
      new HighElevator(m_Elevator).withTimeout(1.5),
        new ParallelCommandGroup(
          new HighExtender(m_Extender),
          new HighWrist(m_Wrist)
        )
    );
  }
}
