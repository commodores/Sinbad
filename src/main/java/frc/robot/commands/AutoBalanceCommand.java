package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class AutoBalanceCommand extends CommandBase {
    
    private final Swerve m_Swerve;
    double elevationAngle;
    double errorThreshold;
    int stopCheck;

    public AutoBalanceCommand(Swerve subsystem) {
        m_Swerve = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_Swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stopCheck = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevationAngle = m_Swerve.getElevationAngle();
        System.out.println(elevationAngle);
        if (elevationAngle > AutoConstants.maxPlatformPositivePitch) {
            m_Swerve.drive(new Translation2d(Constants.Swerve.balanceSpeedMod,0), 0, true, true);
        } else if (elevationAngle < AutoConstants.maxPlatformNegativePitch) {
            m_Swerve.drive(new Translation2d(-Constants.Swerve.balanceSpeedMod,0), 0, true, true);
        } else {
            m_Swerve.drive(new Translation2d(0,0), 0, true, true);
            stopCheck++;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
                /* Get Values, Deadband*/
                double translationVal = 0;
                double strafeVal = 0;
                double rotationVal = 0.05;
                m_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), rotationVal * Constants.Swerve.maxAngularVelocity, true, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return stopCheck > 50;

    }
}