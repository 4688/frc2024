package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents the base of a swerve drive system, incorporating four swerve corners and a NavX for orientation.
 */
public class SwervBase {

    // Constants for initial rotation offsets for each SwervCorner
    private static final double ROTATE_OFFSET_FR = 0.067139;
    private static final double ROTATE_OFFSET_FL = 0.442383;
    private static final double ROTATE_OFFSET_BR = 0.370850;
    private static final double ROTATE_OFFSET_BL = 0.041748;

    // Swerve Corners
    private final SwervCorner CornerFR;
    private final SwervCorner CornerFL;
    private final SwervCorner CornerBR;
    private final SwervCorner CornerBL;
    private double angleFL = 0;
    private double angleFR = 0;
    private double angleBL = 0;
    private double angleBR = 0;
    private boolean xMode = false;
    private double turnOffset = 0;

    // NavX (ROBOT GPS)
    private final AHRS NavX;

    /**
     * Initializes the SwerveBase with predefined corner configurations and a NavX sensor.
     */
    public SwervBase() {
        // Initialize Swerve Corners with IDs and rotation offsets
        CornerFR = new SwervCorner(1, 2, 3, ROTATE_OFFSET_FR);
        CornerFL = new SwervCorner(4, 5, 6, ROTATE_OFFSET_FL);
        CornerBR = new SwervCorner(10,11, 12, ROTATE_OFFSET_BR);
        CornerBL = new SwervCorner(7, 8, 9, ROTATE_OFFSET_BL);

        // Initialize NavX
        NavX = new AHRS();
    }

    /**
     * Resets the NavX orientation and RPM of each swerve corner.
     */
    public void reset() {
        NavX.reset();
    }

    public void resetDistance() {
        CornerFL.resetDrive();
        CornerFR.resetDrive();
        CornerBL.resetDrive();
        CornerBR.resetDrive();
    }

    public double getNavX(){
        return (NavX.getYaw() + 360) % 360;
    }

    public void xToggle(){
        xMode = !xMode;
    }

    /**
     * Publishes encoder values and actual wheel angles to the SmartDashboard.
     */
    public void getEncoders() {
        SmartDashboard.putNumber("FL Encoder", CornerFL.getTurnEnc());
        SmartDashboard.putNumber("FR Encoder", CornerFR.getTurnEnc());
        SmartDashboard.putNumber("BR Encoder", CornerBR.getTurnEnc());
        SmartDashboard.putNumber("BL Encoder", CornerBL.getTurnEnc());

        SmartDashboard.putNumber("FL DISTANCE", CornerFL.getDistance());
        SmartDashboard.putNumber("FR DISTANCE", CornerFR.getDistance());
        SmartDashboard.putNumber("BL DISTANCE", CornerBL.getDistance());
        SmartDashboard.putNumber("BR DISTANCE", CornerBR.getDistance());
    }

    public double getDistance(){
        return Math.min(Math.min(CornerFL.getDistance(),CornerFR.getDistance()) , Math.min(CornerBL.getDistance(),CornerBR.getDistance()));
    }

    public void setTurnOffset(double a){
        turnOffset = a;
    }

    /**
     * Drives the swerve base using input x, y, and z values for movement and rotation.
     * @param x The x-component of the movement vector.
     * @param y The y-component of the movement vector.
     * @param z The z-component (rotation) of the movement vector.
     */
    public void liveMove(double x, double y, double z) {

        
        // Calculate Turn based on NavX
        double roboturn = (NavX.getYaw() + 360) % 360;
        roboturn = (roboturn - turnOffset + 360) % 360;
        SmartDashboard.putNumber("NavX", roboturn);
        double angle = Math.toDegrees(Math.atan2(y, x));
        if (angle < 0) angle += 360; 
        angle = Math.toRadians((angle + roboturn) % 360);
        double mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        x = mag * Math.cos(angle);
        y = mag * Math.sin(angle);
        
        
        // Normalize magnitudes and calculate angles directly within the method
        double frx = x + z;
        double fry = y + z;
        double frmag = Math.sqrt(Math.pow(frx, 2) + Math.pow(fry, 2));
        double flx = x - z;
        double fly = y + z;
        double flmag = Math.sqrt(Math.pow(flx, 2) + Math.pow(fly, 2));
        double blx = x - z;
        double bly = y - z;
        double blmag = Math.sqrt(Math.pow(blx, 2) + Math.pow(bly, 2));
        double brx = x + z;
        double bry = y - z;
        double brmag = Math.sqrt(Math.pow(brx, 2) + Math.pow(bry, 2));
    
        // Calculate angles
        if (flmag != 0){
            angleFL = Math.toDegrees(Math.atan2(fly, flx));
            if (angleFL < 0) angleFL += 360;
        }
        if (frmag != 0){
            angleFR = Math.toDegrees(Math.atan2(fry, frx));
            if (angleFR < 0) angleFR += 360;
        }
        if (blmag != 0){
            angleBL = Math.toDegrees(Math.atan2(bly, blx));
            if (angleBL < 0) angleBL += 360;
        }
        if (brmag != 0){
            angleBR = Math.toDegrees(Math.atan2(bry, brx));
            if (angleBR < 0) angleBR += 360;
        }

        //Checking if we need to turn first without moving (Stops browning out)
        double flDist = calcDist(CornerFL.getWheelAngle(),angleFL);
        double frDist = calcDist(CornerFR.getWheelAngle(),angleFR);
        double brDist = calcDist(CornerBR.getWheelAngle(),angleBR);
        double blDist = calcDist(CornerBL.getWheelAngle(),angleBL);
        if((flDist + frDist + brDist + blDist)/4 > 45){
            flmag = 0;
            frmag = 0;
            blmag = 0;
            brmag = 0;
        }else{

            // Normalize magnitudes
            double bigMag = Math.max(Math.max(flmag, frmag), Math.max(blmag, brmag));
            if (bigMag > 1) {
                flmag /= bigMag;
                frmag /= bigMag;
                blmag /= bigMag;
                brmag /= bigMag;
            }
        }

        if (x == 0 && y == 0 && z == 0 && xMode){
            CornerFL.liveDrive(45, flmag);
            CornerFR.liveDrive(315, frmag);
            CornerBL.liveDrive(135, blmag);
            CornerBR.liveDrive(225, brmag);
        }else{
            // Drive each corner with the calculated angles and magnitudes
            CornerFL.liveDrive(angleFL, flmag);
            CornerFR.liveDrive(angleFR, frmag);
            CornerBL.liveDrive(angleBL, blmag);
            CornerBR.liveDrive(angleBR, brmag);
        }
    }

    public double calcDist(double starting, double ending){
        double clockwise = (ending - starting + 360) % 360;
        double counterCwise = (starting - ending + 360) % 360;
        if (clockwise <= counterCwise) {
            return clockwise;
        } else {           
            return counterCwise;
        }
    }
    
}
