// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class ManualWrist extends CommandBase {
  private final Wrist m_Wrist;
  double speed;

  /** Creates a new ShelfArm. */
  public ManualWrist(Wrist subsystem, double speedManual) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Wrist = subsystem;
    addRequirements(m_Wrist);
    speed = speedManual;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Wrist.manualWrist(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Wrist.manualWrist(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
