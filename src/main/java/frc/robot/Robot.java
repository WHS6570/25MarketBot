// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive;
  private final XboxController driveController = new XboxController(0);
  private final Solenoid launcher = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  private final Solenoid reloader = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
   private final Spark m_fleftMotor = new Spark(3);
   private final Spark m_frightMotor = new Spark(5);
   private final Spark m_bleftMotor = new Spark(4);
   private final Spark m_brightMotor = new Spark(6);
  

  public Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
     m_fleftMotor.addFollower(m_bleftMotor);
     m_frightMotor.addFollower(m_brightMotor);
     m_frightMotor.setInverted(true);

     m_robotDrive = new DifferentialDrive(m_fleftMotor::set, m_frightMotor::set);
    
     SendableRegistry.addChild(m_robotDrive, m_fleftMotor);
     SendableRegistry.addChild(m_robotDrive, m_frightMotor);
  }

  @Override
  public void teleopPeriodic() {
    //BELOW HERE<---------------------------------------------

     m_robotDrive.arcadeDrive(
       ((-driveController.getLeftY() * Math.abs(-driveController.getLeftY())) * (1-(-driveController.getRightTriggerAxis()*0.8))), 
       ((-driveController.getRightX() * Math.abs(-driveController.getRightX())) * (1-(-driveController.getRightTriggerAxis()*0.8)))
     );
    if (driveController.getRightBumperButtonPressed()){
     launcher.set(true);
    } else {
      launcher.set(false);
    }
    if (driveController.getLeftBumperButtonPressed()){
      reloader.set(true);
     } else {
       reloader.set(false);
     }
    
    //ABOVE HERE<---------------------------------------------
}


}   
