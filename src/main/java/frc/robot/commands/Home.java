// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Home extends SequentialCommandGroup {
  public final Elevator m_Elevator;
  public final Extender m_Extender;
  public final Wrist m_Wrist;
  public final Intake m_Intake;
  /** Creates a new Home. */
  public Home(Extender extenderSusbsystem, Elevator elevatorSubsystem, Wrist wristSubsystem, Intake intakeSubsystem) {
     // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    m_Extender = extenderSusbsystem;
    m_Elevator = elevatorSubsystem;
    m_Wrist = wristSubsystem;
    m_Intake = intakeSubsystem;
   
    addCommands(
      new ParallelCommandGroup(
          new HomeExtender(m_Extender),
          new HomeWrist(m_Wrist, m_Intake)),
      new ParallelCommandGroup(
          new HomeElevator(m_Elevator),
          new ZeroWrist(m_Wrist))
    );
  }
}
