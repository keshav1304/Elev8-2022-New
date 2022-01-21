// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {

  /** Creates a new ShooterSubsystem. */
  //private final WPI_TalonSRX FWL;
  //private final WPI_TalonSRX FWR;
  //private final MotorControllerGroup flyWheel;
  private final WPI_TalonSRX Shooter;
  private final Servo Hood;


  public ShooterSubsystem() {


    //FWL = new WPI_TalonSRX(Constants.FWL_port);
    //FWR = new WPI_TalonSRX(Constants.FWR_port);
    //flyWheel = new MotorControllerGroup(FWL, FWR);
    Shooter = new WPI_TalonSRX(Constants.Shooter_port);

    Hood = new Servo(Constants.Hood_port);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(double speed, double seconds)
  {
    //FWR.setInverted(false);

    double startTime = System.currentTimeMillis();
    while ((startTime - System.currentTimeMillis())/1000 <= seconds)
    {
      Shooter.set(speed);
    }
    Shooter.set(0);
    
  }

  public void setHood (double angle)
  {
    /*double startTime = System.currentTimeMillis();
    while ((startTime - System.currentTimeMillis())/1000 <= seconds)
    {
      Hood.set(speed);
    }
    Hood.set(0);*/
    Hood.setAngle(angle);
  }

}
