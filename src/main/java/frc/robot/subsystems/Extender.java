// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ExtenderConstants;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;


public class Extender extends TrapezoidProfileSubsystem {

  public enum ExtenderState {HOME, FLOOR_CUBE, FLOOR_CONE, SHELF, MID, HIGH};
  private final CANSparkMax extenderMotor = new CANSparkMax(ExtenderConstants.extenderMotorID, MotorType.kBrushless);;
  private final RelativeEncoder extenderEncoder;
  private SparkMaxPIDController extenderPIDController;
  private final int m_pidSlot = 0;
  private ExtenderState m_extenderState = ExtenderState.HOME;
  private TrapezoidProfile.State m_extenderPosVel = new TrapezoidProfile.State(0, 0);;

  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
    ExtenderConstants.kSVolts, ExtenderConstants.kGVolts, 
    ExtenderConstants.kVVoltSecondPerMeter, ExtenderConstants.kAVoltSecondSquaredPerMeter);

  private final TimeOfFlight extenderDistanceSensor = new TimeOfFlight(ExtenderConstants.extenderDistanceSensorID);  

  
  /** Creates a new Arm. */
  public Extender() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(ExtenderConstants.kMaxVelMetersPerSec, ExtenderConstants.kMaxAccelMetersPerSecSquared),
        // The initial position of the mechanism
        ExtenderConstants.kInitialPosMeters);

    extenderMotor.restoreFactoryDefaults();
    extenderMotor.setSmartCurrentLimit(80);
    extenderMotor.setIdleMode(IdleMode.kBrake);

    extenderMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    extenderMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    extenderMotor.setSoftLimit(SoftLimitDirection.kForward, ExtenderConstants.kForwardSoftlimit);
    extenderMotor.setSoftLimit(SoftLimitDirection.kReverse, ExtenderConstants.kReverseSoftLimit);

    extenderEncoder = extenderMotor.getEncoder();
    extenderDistanceSensor.setRangingMode(RangingMode.Short, 25);

    // Set the PID coefficients for the Elevator motor
    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    extenderPIDController = extenderMotor.getPIDController();
    extenderPIDController.setP(ExtenderConstants.kP, m_pidSlot);
    extenderPIDController.setI(ExtenderConstants.kI, m_pidSlot);
    extenderPIDController.setD(ExtenderConstants.kD, m_pidSlot);
    extenderPIDController.setIZone(0, m_pidSlot);
    extenderPIDController.setFF(0, m_pidSlot);
    extenderPIDController.setOutputRange(ExtenderConstants.kMinOutput, ExtenderConstants.kMaxOutput, m_pidSlot);    

  }

  public void homeExtender() {
    if (m_extenderState == ExtenderState.HOME) {
      return;
    } else {
      setGoal(ExtenderConstants.kHomePos_m);
      m_extenderState = ExtenderState.HOME;
    }
  }

  public void floorCubesExtender() {
    if (m_extenderState == ExtenderState.FLOOR_CUBE) {
      return;
    } else {
      setGoal(ExtenderConstants.kFloorCubePos_m);
      m_extenderState = ExtenderState.FLOOR_CUBE;
    }
  }

  public void floorConesExtender() {
    if (m_extenderState == ExtenderState.FLOOR_CONE) {
      return;
    } else {
      setGoal(ExtenderConstants.kFloorConePos_m);
      m_extenderState = ExtenderState.FLOOR_CONE;
    }
  }

  public void shelfExtender() {
    if (m_extenderState == ExtenderState.SHELF) {
      return;
    } else {
      setGoal(ExtenderConstants.kShelfPos_m);
      m_extenderState = ExtenderState.SHELF;
    }
  }

  public void midExtender() {
    if (m_extenderState == ExtenderState.MID) {
      return;
    } else {
      setGoal(ExtenderConstants.kMidPos_m);
      m_extenderState = ExtenderState.MID;
    }
  }

  public void highExtender() {
    if (m_extenderState == ExtenderState.HIGH) {
      return;
    } else {
      setGoal(ExtenderConstants.kHighPos_m);
      m_extenderState = ExtenderState.HIGH;
    }
  }

  public ExtenderState getExtenderState() {
    return m_extenderState;
  }

  public TrapezoidProfile.State getExtenderPosVel() {
    return m_extenderPosVel;
  }

  public double getDistanceSensor(){
    return extenderDistanceSensor.getRange()*10;
  }

  public double getEncoderDistance(){
    return extenderEncoder.getPosition()*ExtenderConstants.KExtenderMetersToNeoRotationsFactor;
  }

  public void seedEncoder(){
    extenderEncoder.setPosition(getDistanceSensor()/ExtenderConstants.KExtenderMetersToNeoRotationsFactor);
  }

  @Override
  protected void useState(TrapezoidProfile.State state) {
    // Use the computed profile state here.
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(state.position, state.velocity);
    m_extenderPosVel = state;
    extenderPIDController.setReference(state.position * ExtenderConstants.KExtenderMetersToNeoRotationsFactor, 
        ControlType.kPosition, m_pidSlot, feedforward, ArbFFUnits.kVoltage);
  }

  @Override
  public void periodic(){
    super.periodic();

    SmartDashboard.putNumber("Extender Distance Sensor Position", getDistanceSensor());
    SmartDashboard.putNumber("Extender Encoder Position", getEncoderDistance());
    SmartDashboard.putString("Extender State", getExtenderState().toString());


  }
}