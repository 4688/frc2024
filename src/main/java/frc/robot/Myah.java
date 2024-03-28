package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Myah {
    private static final int USB_PORT = 0;
    private static final double DEADBAND = 0.15;

    private Joystick xBox;
    private int autoTurn = -1;

    public Myah() {
        xBox = new Joystick(USB_PORT);
    }

    public boolean getTurning(){
        return (autoTurn == -1);
    }

    public double getX() {
        return deadband(xBox.getRawAxis(0));
    }

    public double getY() {
        return -deadband(xBox.getRawAxis(1));
    }

    public double getZ() {
        double z = deadband(xBox.getRawAxis(4));
        if (z != 0) {
            autoTurn = -1;
            return z;
        }
        return z;
    }

    public double getZ(double curPos) {
        double z = deadband(xBox.getRawAxis(4))* 0.8;
        if (z != 0) {
            autoTurn = -1;
            return z;
        }
        if (autoTurn == -1) {
            int currentPOV = xBox.getPOV();
            if (currentPOV == 45) {
                autoTurn = 60;
            } else if (currentPOV == 315) {
                autoTurn = 300;
            } else {
                autoTurn = currentPOV;
            }
        }

        if (autoTurn != -1) {
            if (Math.abs(curPos - autoTurn) < 0.2) {
                autoTurn = -1;
            } else {
                double clockwise = (autoTurn - curPos + 360) % 360;
                double counterCwise = (curPos - autoTurn + 360) % 360;
                if (clockwise > counterCwise) {
                    z = -Math.max(counterCwise / 180, 0.03);
                } else {
                    z = Math.max(clockwise / 180, 0.03);
                }
            }
        }
        return z;
    }

    public double getAutoZ(double curPos) {
        if (autoTurn != -1) {
            if (Math.abs(curPos - autoTurn) < 0.2) {
                autoTurn = -1;
            } else {
                double clockwise = (autoTurn - curPos + 360) % 360;
                double counterCwise = (curPos - autoTurn + 360) % 360;
                if (clockwise > counterCwise) {
                    return -Math.max(counterCwise / 180, 0.03);
                } else {
                    return Math.max(clockwise / 180, 0.03);
                }
            }
        }
        return 0;
    }

    public void turnTo(int turnAngle){
        autoTurn = turnAngle;
    }

    public boolean getStartButton(){
        return xBox.getRawButtonPressed(7);
    }

    public boolean getSelectButton(){
        return xBox.getRawButtonPressed(8);
    }

    public boolean getYButton(){
        return xBox.getRawButton(4);
    }

    public boolean getXButton(){
        if (xBox.getRawButton(4)){return false;}
        return xBox.getRawButton(3);
    }

    public boolean getBButton(){
        return xBox.getRawButtonPressed(2);
    }

    public boolean getAButton(){
        return xBox.getRawButtonPressed(1);
    }

    public boolean getClimbButton(){
        return xBox.getRawButton(6);
    }

    public boolean getRaiseButton(){
        return xBox.getRawButton(5);
    }

    public boolean getSafteyButton(){
        return xBox.getRawButton(8);
    }

    public boolean getRobotCentric(){
        return (xBox.getRawAxis(2) > 0.4);
    }

    public double getBrake(){
        return 1 - (xBox.getRawAxis(3) * 0.8);
    }

    private double deadband(double a) {
        if (Math.abs(a) < DEADBAND) {
            return 0;
        }
        return a;
    }

}