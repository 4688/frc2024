// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {

  SwervBase drivebase;
  EveryBot kitBot;
  Limelight limey;
  Myah driveController;

  private SendableChooser<String> autoChooser = new SendableChooser<>();
  
  private boolean isAuto = false;
  private int curAuto = 0;
  private int curAutoStep = 0;


  /*DA AUTO MASTER PLAN (TOP SECRET NO ONE CAN KNOW) 
   * 1 - when A is pressed, log the current ID, SKEW, X and Z, if ID = 0, END
   * 2 - Set Acentric to the current Apriltag scew and drive the X vector only till x = 0
   * 3 - Turn the robot to the saved skew + Cur Position
   * 4 - Recapture Apriltag to update desired X and Z if ID is correct while driving the desired X and Z distance in Acentric to updated scew. 
   * 5 - Once found, stop. (Maybe do something else);
   * 
   * AutoChecker -> WHen A is pressed, if ID is good for alliance, set Autohandler to current ID 
   * AutoHandler -> Always running, if variable is not set to 0, call handler for specific ID
   * SpecificHandler -> Keep track of autostep, when currentID is reset, so should the step, run the code based on the step.
  */

  double x;
  double y;
  double autoX;
  double autoZ;
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
    drivebase.setToGlideMode();
    limey.checkField();

    autoChooser.setDefaultOption("The Classic Shoot", "turnShoot");
    autoChooser.addOption("MR CLEAN!!!", "cleanUp");
    SmartDashboard.putData("Autonomous Modes", autoChooser);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    drivebase.resetNavx();
    limey.checkField();
    drivebase.setToGlideMode();
    drivebase.resetDistance();
  }

  @Override
  public void autonomousPeriodic() {
    String selectedMode = autoChooser.getSelected();
    
    switch (selectedMode) {
        case "turnShoot":
            turnAndShoot();
            break;
        case "cleanUp":
            wipeIT();
            break;
    }
  }

  public void turnAndShoot(){
    if(curAutoStep == 0){
      if(NavXDriveTo(0, 1.1)){
        curAutoStep = 1;
        drivebase.resetDistance();
      }
    }else if(curAutoStep == 1){
      if(autoTurn(180)){
        curAutoStep = 2;
        drivebase.resetDistance();
      }
    }else if(curAutoStep == 2){
      if(NavXDriveTo(0, -0.9)){
        curAutoStep = 3;
        drivebase.resetDistance();
      }
    }else if(curAutoStep == 3){
      if(!kitBot.shoot()){
        curAutoStep = 4;
      }
    }
  }

  public void wipeIT() {
    if (limey.areWeRed()) {
      if (curAutoStep == 0) {
        if (NavXDriveTo(-1.4, 7.87)) {
          curAutoStep = 1;
          drivebase.resetDistance();
        }
      } else if (curAutoStep == 1) {
        if (autoTurn(135)) {
          curAutoStep = 2;
          drivebase.resetDistance();
        }
      } else if (curAutoStep == 2) {
        if (NavXDriveTo(6.75, 0)) {
          curAutoStep = 3;
          drivebase.resetDistance();
        }
      } 

    } else {
      if (curAutoStep == 0) {
        if (NavXDriveTo(1.4, 7.87)) {
          curAutoStep = 1;
          drivebase.resetDistance();
        }
      } else if (curAutoStep == 1) {
        if (autoTurn(135)) {
          curAutoStep = 2;
          drivebase.resetDistance();
        }
      } else if (curAutoStep == 2) {
        if (NavXDriveTo(-6.75, 0)) {
          curAutoStep = 3;
          drivebase.resetDistance();
        }
      } 
    }

  }

  @Override
  public void teleopInit() {
    limey.checkField();
    drivebase.setToBrakeMode();
    curAutoStep = 0;
  }

  

  @Override
  public void teleopPeriodic() {
   if (driveController.getAButton()){
      isAuto = !isAuto;
      drivebase.resetDistance();
  }

    if(isAuto){
      limey.updateLimelight();
      if(limey.canSee()){
        if(limey.getID() == 9 || limey.getID() == 10){
          driveController.turnTo(300);
        }else if(limey.getID() == 1 || limey.getID() == 2){
          driveController.turnTo(60);
        }else if(limey.getID() == 5){
          driveController.turnTo(90);
        }else if(limey.getID() == 6){
          driveController.turnTo(270);
        }
      }
      isAuto = false;
    }else{
      teleopDrive();
      curAutoStep = 0;
    }

    kitBot.displayDiagnostics();
    drivebase.getEncoders();
    limey.updateLimelight();
    kitBot.handleLights(limey.canSee());
    
  }

  @Override
  public void disabledInit() {
    drivebase.setToCoastMode();
  }

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

  public void teleopDrive(){
    x = driveController.getX();
    y = driveController.getY();
    z = driveController.getZ(drivebase.getNavX());

    if (driveController.getStartButton()){
      drivebase.resetNavx();
    }

    if (driveController.getSelectButton()){
      drivebase.xToggle();
    }

    if(driveController.getYButton()){
      kitBot.handleNoteIn(driveController.getYButton());
    }else{
      kitBot.handleNoteOut(driveController.getXButton());
    }
    
    if (driveController.getBButton()){
      kitBot.switchMode();
    }

    kitBot.handleClimb(driveController.getClimbButton(), driveController.getRaiseButton(), driveController.getSafteyButton());

    m =  driveController.getBrake();
    if(driveController.getRobotCentric()){
      drivebase.robotCentric(x * m, y * m, z * m);
    }else{
      drivebase.fieldCentric(x * m, y * m, z * m);
    }
  }










  public boolean LimelightDriveTo(double localx, double localz){
    double startDist = drivebase.calculateMagnitude(localx, localz);
    if (startDist < 0.05){
      drivebase.robotCentric(0, 0, 0);
      return true;
    }
    double distance = startDist - drivebase.getDistance();
    if(Math.abs(distance) < 0.05){
      drivebase.robotCentric(0, 0, 0);
      return true;
    }
    drivebase.robotCentric(-(localx/startDist)*distance, (localz/startDist)*distance, 0);
    return false;
  }

  public boolean NavXDriveTo(double localx, double localz){
    double startDist = drivebase.calculateMagnitude(localx, localz);
    if (startDist < 0.05){
      drivebase.robotCentric(0, 0, 0);
      return true;
    }
    double distance = startDist - drivebase.getDistance();
    if(Math.abs(distance) < 0.05){
      drivebase.robotCentric(0, 0, 0);
      return true;
    }
    drivebase.fieldCentric((localx/startDist)*distance, (localz/startDist)*distance, 0);
    return false;
  }  

  public boolean autoTurn(int a){
    driveController.turnTo(a);
    double turnZ = driveController.getAutoZ(drivebase.getNavX());
    drivebase.robotCentric(0, 0, turnZ);
    if(driveController.getTurning()){
      drivebase.robotCentric(0, 0, 0);
      return true;
    }else{
      return false;
    }
  }
}
