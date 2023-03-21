// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extender;


public class ManualExtender extends CommandBase {
  private final Extender m_Extender;
  double speed;

  /** Creates a new ShelfArm. */
  public ManualExtender(Extender subsystem, double speedManual) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Extender = subsystem;
    addRequirements(m_Extender);
    speed = speedManual;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // m_Extender.seedEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Extender.manualExtender(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Extender.manualExtender(0);
   // m_Extender.seedEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
