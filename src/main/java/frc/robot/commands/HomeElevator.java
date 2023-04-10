// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;


public class HomeElevator extends CommandBase {

  private final Elevator m_Elevator;
  /** Creates a new HomeElevator. */

  public HomeElevator( Elevator subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_Elevator = subsystem;

    addRequirements(m_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_Elevator.setEncoder(Units.inchesToMeters(20)*ElevatorConstants.KElevatorMetersToNeoRotationsFactor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_Elevator.manualElevator(-0.85);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.manualElevator(0);
    m_Elevator.resetEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_Elevator.getLimitSwitch();
  }
}
