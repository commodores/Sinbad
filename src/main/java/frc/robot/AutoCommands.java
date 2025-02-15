package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;;

public class AutoCommands {

    private final Swerve swerve;
    public final Map<String, SequentialCommandGroup> autos;
    public final Map<String, Command> eventMap;
    private SwerveAutoBuilder autoBuilder;
    private PathPlannerTrajectory trajectory;

    //Example Multi-Path
    

    public AutoCommands(Swerve swerve) {
        
        this.swerve = swerve;

        //Build Autos
        autos = new HashMap<String, SequentialCommandGroup>();
        eventMap = new HashMap<String, Command>();
        
        /////Do Nothing//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        autos.put("nothing", new SequentialCommandGroup(
          new Nothing()
        ));

        /////Auto Balance Tester//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        List<PathPlannerTrajectory> AutoBalanceTest = PathPlanner.loadPathGroup("AutoBalanceTest", new PathConstraints(1, 1));
        autos.put("AutoBalanceTest", new SequentialCommandGroup(
            getCommand(AutoBalanceTest),
            new AutoBalanceCommand(RobotContainer.s_Swerve),
            new AutoLock(RobotContainer.s_Swerve)
        )); 
       
        /////Charge Auto//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        List<PathPlannerTrajectory> Charge = PathPlanner.loadPathGroup("Charge", new PathConstraints(1.4, 1.4));
        autos.put("Charge", new SequentialCommandGroup(
            new High(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(2.5),
            new AutoRelease(RobotContainer.m_Intake).withTimeout(.5),
            new StowAuto(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(2),
            getCommand(Charge),
            new AutoBalanceCommand(RobotContainer.s_Swerve),
            new AutoLock(RobotContainer.s_Swerve)
        ));       

        /////Charge + Game Piece//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        List<PathPlannerTrajectory> ChargeGamePiece = PathPlanner.loadPathGroup("ChargeGamePiece", new PathConstraints(1.25, 1.25));
        autos.put("ChargeGamePiece", new SequentialCommandGroup(
            new Stowed(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(.5),
            new AutoRelease(RobotContainer.m_Intake).withTimeout(.5),
            getCommand(ChargeGamePiece),
            new AutoBalanceCommand(RobotContainer.s_Swerve),
            new AutoLock(RobotContainer.s_Swerve)
        ));

       

        /////Two Piece High//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        List<PathPlannerTrajectory> TwoPieceHigh = PathPlanner.loadPathGroup("TwoPieceHigh", new PathConstraints(2.5, 2));
        autos.put("TwoPieceHigh", new SequentialCommandGroup(
            new High(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(2.5),
            new AutoRelease(RobotContainer.m_Intake).withTimeout(.5),
            new StowAuto(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(2),
            getCommand(TwoPieceHigh),
            new High(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(2),
            new AutoRelease(RobotContainer.m_Intake).withTimeout(.5),
            new StowAuto(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(2)
        ));

        /////Gather//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        List<PathPlannerTrajectory> Gather = PathPlanner.loadPathGroup("Gather", new PathConstraints(2.5, 2));
        autos.put("Gather", new SequentialCommandGroup(
            new Stowed(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(.5),
            new AutoRelease(RobotContainer.m_Intake).withTimeout(.5),
            getCommand(Gather),
            new AutoRelease(RobotContainer.m_Intake).withTimeout(.5)
          
        ));

         /////High Mid Cube//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         List<PathPlannerTrajectory> HighMidCube = PathPlanner.loadPathGroup("HighMidCube", new PathConstraints(2.5, 2));
         autos.put("HighMidCube", new SequentialCommandGroup(
             new Stowed(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(.5),
             new High(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(2.5),
             new AutoRelease(RobotContainer.m_Intake).withTimeout(.1),
             new StowAuto(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(2),
             getCommand(HighMidCube),
             new Mid(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(2),
             new AutoRelease(RobotContainer.m_Intake).withTimeout(.1),
             new StowAuto(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(2)
         ));

         /////Three Cube Score//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        List<PathPlannerTrajectory> ThreeCubeInOut = PathPlanner.loadPathGroup("ThreeCubeInOut", new PathConstraints(3, 3));
        autos.put("ThreeCubeInOut", new SequentialCommandGroup(
            new Stowed(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(.5),
            new AutoRelease(RobotContainer.m_Intake).withTimeout(.5),
            getCommand(ThreeCubeInOut),
            new StowAuto(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(2)
        ));

        /////Three Cube Score Number Two//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        List<PathPlannerTrajectory> ThreeCubeOutIn = PathPlanner.loadPathGroup("ThreeCubeOutIn", new PathConstraints(3, 3));
        autos.put("ThreeCubeOutIn", new SequentialCommandGroup(
            new Stowed(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(.5),
            new AutoRelease(RobotContainer.m_Intake).withTimeout(.5),
            getCommand(ThreeCubeOutIn),
            new StowAuto(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist).withTimeout(2)
        ));

        //Events////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        eventMap.put("runIntake", new AutoIntake(RobotContainer.m_Intake));
        eventMap.put("release", new AutoRelease(RobotContainer.m_Intake));
        eventMap.put("stopIntake", new StopIntake(RobotContainer.m_Intake));
        eventMap.put("ground", new Ground(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist));
        eventMap.put("groundUp", new GroundUp(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist));
        eventMap.put("stow", new Stowed(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist));
        eventMap.put("high", new High(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist));
        eventMap.put("mid", new Mid(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist));
        eventMap.put("groundAuto", new GroundAuto(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist)); 
        eventMap.put("stowAuto", new StowAuto(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist));
        eventMap.put("nothing", new Nothing().withTimeout(2));
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        

    }

    private Command getCommand(List<PathPlannerTrajectory>pathGroup) {                
        
        autoBuilder = new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetOdometry,
            Constants.Swerve.swerveKinematics,
            new PIDConstants(Constants.AutoConstants.kPXandYControllers, 0, 0),
            new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0),
            swerve::setModuleStates,
            eventMap,
            true,
            swerve);

        return autoBuilder.fullAuto(pathGroup);
    }

    

}