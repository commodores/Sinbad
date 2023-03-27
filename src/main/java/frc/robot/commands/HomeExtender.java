// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.subsystems.Extender;



public class HomeExtender extends CommandBase {
  private final Extender m_Extender;
  /** Creates a new HomeExtender. */
  public HomeExtender(Extender subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Extender = subsystem;

    addRequirements(m_Extender);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_Extender.setEncoder(Units.inchesToMeters(22)*ExtenderConstants.KExtenderMetersToNeoRotationsFactor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_Extender.manualExtender(-.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Extender.manualExtender(0);
    m_Extender.resetEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Extender.getLimitSwitch();
  }
}
