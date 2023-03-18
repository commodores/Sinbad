// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Ground extends SequentialCommandGroup {
  public final Elevator m_Elevator;
  public final Arm m_Arm;
  public final Wrist m_Wrist;
  /** Creates a new Ground. */
  public Ground(Arm armSusbsystem, Elevator elevatorSubsystem, Wrist wristSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_Arm = armSusbsystem;
    m_Elevator = elevatorSubsystem;
    m_Wrist = wristSubsystem;
    addCommands(new GroundElevator(m_Elevator).withTimeout(2), new ParallelCommandGroup(new GroundArm(m_Arm), new GroundWrist(m_Wrist)));
  }
}
