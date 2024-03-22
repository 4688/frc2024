// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  SwervBase drivebase;
  EveryBot kitBot;
  Limelight limey;
  Myah driveController;
  
  private int lastPOV = -1;
  private int turnToAng = -1;
  private boolean isAuto = false;
  public double[] aprilTags;
  public boolean isShooting = false;
  public boolean isIntaking = false;
  public boolean isRed = true;
  double saveZ = 0;
  double saveX = 0;
  double contID = 0;


  double x;
  double y;
  double z;
  double m;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    drivebase = new SwervBase();
    kitBot = new EveryBot();
    limey = new Limelight();
    driveController = new Myah();
    drivebase.resetNavx();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    drivebase.resetNavx();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
    
  }

  

  @Override
  public void teleopPeriodic() {
   
    x = driveController.getX();
    y = driveController.getY();
    z = driveController.getZ(drivebase.getNavX());


    if (xBox.getRawButtonPressed(1)){
      isAuto = !isAuto;
    }

    if(isAuto){
      if(isRed){
        if(contID == 4){
          if(!johnathan.shoot()){
            isAuto = false;
            drivebase.setTurnOffset(0);
            contID = 0;
          }
        }
        else if(closestId == 4){
          double dist = Math.sqrt(Math.pow(txValue, 2) + Math.pow(tzValue, 2));
          double angle = Math.toDegrees(Math.atan2(tzValue, txValue)) - 180;
          if (angle < 0) angle += 360;
          if (txValue > 1) txValue = 1;
          if (txValue < -1) txValue = -1;
          if (tzValue > 1) tzValue = 1;
          if (tzValue < -1) tzValue = -1;
          drivebase.setTurnOffset(180);
          turnToAng = (int) angle;
          if (dist > 2.25){
            x = -txValue;
            y = -tzValue;
          }else if(dist < 1.2){
            x = txValue;
            y = tzValue;
          }else{
            turnToAng = (int) (angle + 26.5) % 360;
            contID = 4;
          }
        }

        else if(closestId == 3){
          double angle = Math.toDegrees(Math.atan2(tzValue, txValue - 0.616)) - 180;
          if (angle < 0) angle += 360;
          turnToAng = (int) angle % 360;
        }

        if(closestId == 9 || closestId == 10){
          if (johnathan.getActiveShooter()){
            txValue = txValue - 0.1;
          }else{
            txValue = txValue + 0.1;
          }
          
          if ((Math.abs(txValue) < 0.02 && Math.abs(tzValue) < 0.02) || tzValue > 200){
            contID = 9;
            drivebase.resetDistance();
          }else{
            turnToAng = 300;
            drivebase.setTurnOffset(300);
            if (txValue > 1) txValue = 1;
            if (txValue < -1) txValue = -1;
            if (tzValue > 1) tzValue = 1;
            if (tzValue < -1) tzValue = -1;
            x = -txValue;
            y = -tzValue;
            saveZ = txValue;
            saveX = tzValue;
          }
        }
        else if(contID == 9){
          double distToTarget = Math.sqrt(Math.pow(saveX, 2) + Math.pow(saveZ, 2)) - drivebase.getDistance();
          if(distToTarget < 0.01 && distToTarget > -0.01){
            contID = 0;
            drivebase.resetDistance();
          }else{
            double regy = saveZ /(Math.abs(saveX) + Math.abs(saveZ));
            double regx = saveX / (Math.abs(saveX) + Math.abs(saveZ));
            y = -regy * distToTarget;
            x = -regx * distToTarget;
          }
        }else if(closestId == 5){
          txValue = txValue + 0.1;
          if ((Math.abs(txValue) < 0.02 && Math.abs(tzValue) < 0.02) || tzValue > 200){
            contID = 5;
            drivebase.resetDistance();
          }else{
            turnToAng = 90;
            drivebase.setTurnOffset(90);
            if (txValue > 1) txValue = 1;
            if (txValue < -1) txValue = -1;
            if (tzValue > 1) tzValue = 1;
            if (tzValue < -1) tzValue = -1;
            x = -txValue;
            y = -tzValue;
            saveZ = txValue;
            saveX = tzValue;
          }
        }
        else if(contID == 5){
          double distToTarget = Math.sqrt(Math.pow(saveX, 2) + Math.pow(saveZ, 2)) - drivebase.getDistance();
          if(distToTarget < 0.01 && distToTarget > -0.01){
            contID = 0;
            drivebase.resetDistance();
          }else{
            double regy = saveZ /(Math.abs(saveX) + Math.abs(saveZ));
            double regx = saveX / (Math.abs(saveX) + Math.abs(saveZ));
            y = -regy * distToTarget;
            x = -regx * distToTarget;
          }
        }else{
          isAuto = false;
          drivebase.setTurnOffset(0);
        }
      }else{
        if(contID == 7){
          if(!johnathan.shoot()){
            isAuto = false;
            drivebase.setTurnOffset(0);
            contID = 0;
          }
        }
        else if(closestId == 7){
          double dist = Math.sqrt(Math.pow(txValue, 2) + Math.pow(tzValue, 2));
          double angle = Math.toDegrees(Math.atan2(tzValue, txValue)) - 180;
          if (angle < 0) angle += 360;
          if (txValue > 1) txValue = 1;
          if (txValue < -1) txValue = -1;
          if (tzValue > 1) tzValue = 1;
          if (tzValue < -1) tzValue = -1;
          turnToAng = (int) angle;
          drivebase.setTurnOffset(180);
          if (dist > 2.25){
            x = -txValue;
            y = -tzValue;
          }else if(dist < 1.2){
            x = txValue;
            y = tzValue;
          }else{
            turnToAng = (int) (angle + 26.5) % 360;
            contID = 7;
          }
        }

        else if(closestId == 8){
          double angle = Math.toDegrees(Math.atan2(tzValue, txValue + 0.616)) - 180;
          if (angle < 0) angle += 360;
          turnToAng = (int) angle % 360;
        }

        if(closestId == 1 || closestId == 2){
          if (johnathan.getActiveShooter()){
            txValue = txValue - 0.1;
          }else{
            txValue = txValue + 0.1;
          }
          
          if ((Math.abs(txValue) < 0.02 && Math.abs(tzValue) < 0.02) || tzValue > 200){
            contID = 1;
            drivebase.resetDistance();
          }else{
            turnToAng = 60;
            drivebase.setTurnOffset(60);
            if (txValue > 1) txValue = 1;
            if (txValue < -1) txValue = -1;
            if (tzValue > 1) tzValue = 1;
            if (tzValue < -1) tzValue = -1;
            x = -txValue;
            y = -tzValue;
            saveZ = txValue;
            saveX = tzValue;
          }
        }
        else if(contID == 1){
          double distToTarget = Math.sqrt(Math.pow(saveX, 2) + Math.pow(saveZ, 2)) - drivebase.getDistance();
          if(distToTarget < 0.01 && distToTarget > -0.01){
            contID = 0;
            drivebase.resetDistance();
          }else{
            double regy = saveZ /(Math.abs(saveX) + Math.abs(saveZ));
            double regx = saveX / (Math.abs(saveX) + Math.abs(saveZ));
            y = -regy * distToTarget;
            x = -regx * distToTarget;
          }
        }else if(closestId == 6){
          txValue = txValue + 0.1;
          if ((Math.abs(txValue) < 0.02 && Math.abs(tzValue) < 0.02) || tzValue > 200){
            contID = 6;
            drivebase.resetDistance();
          }else{
            turnToAng = 270;
            drivebase.setTurnOffset(270);
            if (txValue > 1) txValue = 1;
            if (txValue < -1) txValue = -1;
            if (tzValue > 1) tzValue = 1;
            if (tzValue < -1) tzValue = -1;
            x = -txValue;
            y = -tzValue;
            saveZ = txValue;
            saveX = tzValue;
          }
        }
        else if(contID == 6){
          double distToTarget = Math.sqrt(Math.pow(saveX, 2) + Math.pow(saveZ, 2)) - drivebase.getDistance();
          if(distToTarget < 0.01 && distToTarget > -0.01){
            contID = 0;
            drivebase.resetDistance();
          }else{
            double regy = saveZ /(Math.abs(saveX) + Math.abs(saveZ));
            double regx = saveX / (Math.abs(saveX) + Math.abs(saveZ));
            y = -regy * distToTarget;
            x = -regx * distToTarget;
          }
        }else{
          isAuto = false;
          drivebase.setTurnOffset(0);
        }
      }
    }
        











    if (driveController.getStartButton()){
      drivebase.resetNavx();
    }

    if (driveController.getSelectButton()){
      drivebase.xToggle();
    }

    kitBot.handleNoteIn(driveController.getYButton());
    kitBot.handleNoteOut(driveController.getXButton());
    

    if (driveController.getBButton){
      kitBot.switchMode();
    }

    kitBot.handleClimb(driveController.getClimbButton(), driveController.getRaiseButton(), driveController.getSafteyButton());


    kitBot.displayDiagnostics();
    drivebase.getEncoders();


    m =  driveController.getBrake();
    drivebase.fieldCentric(y * m, x * m, z * m);
    

  }

  

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
