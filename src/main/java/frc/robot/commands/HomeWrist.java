// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class HomeWrist extends CommandBase {
  private final Wrist m_Wrist;
  private final Intake m_Intake;
  /** Creates a new GroundArm. */
  public HomeWrist(Wrist subsystem, Intake intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Wrist = subsystem;
    m_Intake = intakeSubsystem;

    addRequirements(m_Wrist, m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Wrist.setEncoder(-54);

    m_Intake.runIntakeSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Wrist.manualWrist(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_Wrist.manualWrist(0);
    m_Wrist.resetEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Wrist.getLimitSwitch();
  }
}
