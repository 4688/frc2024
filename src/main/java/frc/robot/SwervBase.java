package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents the base of a swerve drive system, incorporating four swerve corners and a NavX for orientation.
 */
public class SwervBase {

    // Constants for initial rotation offsets for each SwervCorner
    private static final double ROTATE_OFFSET_FR = 0.794678;
    private static final double ROTATE_OFFSET_FL = 0.543213;
    private static final double ROTATE_OFFSET_BR = 0.273430;
    private static final double ROTATE_OFFSET_BL = 0.298096;

    // Swerve Corners
    private final SwervCorner CornerFR;
    private final SwervCorner CornerFL;
    private final SwervCorner CornerBR;
    private final SwervCorner CornerBL;
    private double angleFL = 0;
    private double angleFR = 0;
    private double angleBL = 0;
    private double angleBR = 0;

    // NavX (ROBOT GPS)
    private final AHRS NavX;

    /**
     * Initializes the SwerveBase with predefined corner configurations and a NavX sensor.
     */
    public SwervBase() {
        // Initialize Swerve Corners with IDs and rotation offsets
        CornerFR = new SwervCorner(2, 1, 9, ROTATE_OFFSET_FR);
        CornerFL = new SwervCorner(5, 6, 12, ROTATE_OFFSET_FL);
        CornerBR = new SwervCorner(4, 3, 10, ROTATE_OFFSET_BR);
        CornerBL = new SwervCorner(7, 8, 11, ROTATE_OFFSET_BL);

        // Initialize NavX
        NavX = new AHRS();
    }

    /**
     * Resets the NavX orientation and RPM of each swerve corner.
     */
    public void reset() {
        NavX.reset();
        CornerFL.resetRPM();
        CornerFR.resetRPM();
        CornerBL.resetRPM();
        CornerBR.resetRPM();
    }

    /**
     * Publishes encoder values and actual wheel angles to the SmartDashboard.
     */
    public void getEncoders() {
        SmartDashboard.putNumber("FL Encoder", CornerFL.getTurnEnc());
        SmartDashboard.putNumber("FR Encoder", CornerFR.getTurnEnc());
        SmartDashboard.putNumber("BR Encoder", CornerBR.getTurnEnc());
        SmartDashboard.putNumber("BL Encoder", CornerBL.getTurnEnc());

        SmartDashboard.putNumber("FL Actual Angle", CornerFL.getWheelAngle());
    }

    /**
     * Drives the swerve base using input x, y, and z values for movement and rotation.
     * @param x The x-component of the movement vector.
     * @param y The y-component of the movement vector.
     * @param z The z-component (rotation) of the movement vector.
     */
    public void liveMove(double x, double y, double z) {
        // Normalize magnitudes and calculate angles directly within the method
        double frx = x + z;
        double fry = y - z;
        double frmag = Math.sqrt(Math.pow(frx, 2) + Math.pow(fry, 2));
        double flx = x + z;
        double fly = y + z;
        double flmag = Math.sqrt(Math.pow(flx, 2) + Math.pow(fly, 2));
        double blx = x - z;
        double bly = y + z;
        double blmag = Math.sqrt(Math.pow(blx, 2) + Math.pow(bly, 2));
        double brx = x - z;
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

        // Calculate Turn based on NavX
        double roboturn = (NavX.getYaw() + 360) % 360;
        SmartDashboard.putNumber("NavX", roboturn);

        //Checking if we need to turn first without moving (Stops browning out)
        double flDist = calcDist(CornerFL.getWheelAngle(),(angleFL + roboturn) % 360);
        double frDist = calcDist(CornerFR.getWheelAngle(),(angleFR + roboturn) % 360);
        double brDist = calcDist(CornerBR.getWheelAngle(),(angleBR + roboturn) % 360);
        double blDist = calcDist(CornerBL.getWheelAngle(),(angleBL + roboturn) % 360);
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
    
        // Drive each corner with the calculated angles and magnitudes
        CornerFL.liveDrive((angleFL + roboturn) % 360, flmag);
        CornerFR.liveDrive((angleFR + roboturn) % 360, frmag);
        CornerBL.liveDrive((angleBL + roboturn) % 360, blmag);
        CornerBR.liveDrive((angleBR + roboturn) % 360, brmag);
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
