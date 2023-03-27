package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AlignToTarget extends CommandBase {
  double Kp = 0.000001;
  double min_command = .9;

  public AlignToTarget() {
        // Use requires() here to declare subsystem dependencies
        addRequirements(RobotContainer.s_Swerve);
        addRequirements(RobotContainer.m_LimeLight);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
      
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

      double heading_error = - RobotContainer.m_LimeLight.getYAngle();
      double steering_adjust = 0.0;
      
      if (RobotContainer.m_LimeLight.getYAngle() > 0.0)
      {
              steering_adjust = -(Kp*heading_error + min_command);
      }
      else if (RobotContainer.m_LimeLight.getYAngle() < 0.0)
      {
              steering_adjust = Kp*heading_error + min_command;
      }

      RobotContainer.s_Swerve.drive(new Translation2d(0,0), steering_adjust, true, true);

      
     
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
      return (RobotContainer.m_LimeLight.getYAngle() < .5 && RobotContainer.m_LimeLight.getYAngle() > -.5)||!RobotContainer.m_LimeLight.isTargetVisible();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      
    }

    
}