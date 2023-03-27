// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Elevator extends SubsystemBase {

  private final CANSparkMax elevatorMotor;

  private SparkMaxPIDController elevatorPIDController;
  private RelativeEncoder elevatorEncoder;
 // private final TimeOfFlight extenderDistanceSensor;
  private final double forwardLimit, reverseLimit;
  private final DigitalInput reverseLimitSwitch;
  
  
  double kP = Constants.ElevatorConstants.elevatorKP,
    kI = Constants.ElevatorConstants.elevatorKI,
    kD = Constants.ElevatorConstants.elevatorKD,
    kIz = Constants.ElevatorConstants.elevatorKIz,
    kFF = Constants.ElevatorConstants.elevatorKFF, 
    kMinOutput = Constants.ElevatorConstants.elevatorKMinOutput,
    kMaxOutput = Constants.ElevatorConstants.elevatorKMaxOutput,
    minVel = Constants.ElevatorConstants.elevatorMinVel,
    maxVel = Constants.ElevatorConstants.elevatorMaxVel,
    maxAcc = Constants.ElevatorConstants.elevatorMaxAcc,
    allowedErr = Constants.ElevatorConstants.elevatorAllowedErr;

  
  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new CANSparkMax(Constants.ElevatorConstants.elevatorMotorID, MotorType.kBrushless);

    //extenderDistanceSensor = new TimeOfFlight(ElevatorConstants.elevatorDistanceSensorID);  

    elevatorMotor.restoreFactoryDefaults();
    elevatorMotor.setSmartCurrentLimit(80);
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    reverseLimit = 0;
    forwardLimit = Units.inchesToMeters(19)*ElevatorConstants.KElevatorMetersToNeoRotationsFactor;


    elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, ((float)forwardLimit));
    elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, ((float)reverseLimit));

    elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // initialze PID controller and encoder objects
    elevatorPIDController = elevatorMotor.getPIDController();
    elevatorEncoder = elevatorMotor.getEncoder();
    reverseLimitSwitch = new DigitalInput(1);
    

    // set PID coefficients
    elevatorPIDController.setP(kP);
    elevatorPIDController.setI(kI);
    elevatorPIDController.setD(kD);
    elevatorPIDController.setIZone(kIz);
    elevatorPIDController.setFF(kFF);
    elevatorPIDController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    elevatorPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    elevatorPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    elevatorPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    elevatorPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    //seedEncoder();
  }

  public void setPosition(double targetPosition){
    elevatorPIDController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);
  }

  public void manualElevator(double speed){
    elevatorMotor.set(speed);
  }

  public double getOutputCurrent() {
    return elevatorMotor.getOutputCurrent();
  }

  public double getPosition() {
    return Units.metersToInches(elevatorEncoder.getPosition()/ElevatorConstants.KElevatorMetersToNeoRotationsFactor);
  }

  public void resetEncoder(){
    elevatorMotor.getEncoder().setPosition(0);
  }

  public void setEncoder(double position){
    elevatorMotor.getEncoder().setPosition(position);
  }


  //public double getDistanceSensor(){
   // return Units.metersToInches((extenderDistanceSensor.getRange()*.001)-.05588);
  //}

  //public void seedEncoder(){
    //elevatorEncoder.setPosition(Units.inchesToMeters(getDistanceSensor()*ElevatorConstants.KElevatorMetersToNeoRotationsFactor));
  //}

  public boolean getLimitSwitch(){
    return !reverseLimitSwitch.get();
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Current", getOutputCurrent());
    SmartDashboard.putNumber("Elevator Position", getPosition());
    //SmartDashboard.putNumber("Elevator Distance Sensor Position", getDistanceSensor());
    SmartDashboard.putBoolean("Elevator Reverse Limit Switch", getLimitSwitch());

    
  }

  
}