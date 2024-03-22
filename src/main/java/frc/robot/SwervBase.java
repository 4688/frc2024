package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents the base of a swerve drive system, incorporating four swerve
 * corners and a NavX for orientation.
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

    // NavX (ROBOT GPS)
    private final AHRS NavX;

    /**
     * Initializes the SwerveBase with predefined corner configurations and a NavX
     * sensor.
     */
    public SwervBase() {
        // Initialize Swerve Corners with IDs and rotation offsets
        CornerFR = new SwervCorner(1, 2, 3, ROTATE_OFFSET_FR);
        CornerFL = new SwervCorner(4, 5, 6, ROTATE_OFFSET_FL);
        CornerBR = new SwervCorner(10, 11, 12, ROTATE_OFFSET_BR);
        CornerBL = new SwervCorner(7, 8, 9, ROTATE_OFFSET_BL);

        // Initialize NavX
        NavX = new AHRS();
    }

    /**
     * Resets the NavX orientation and RPM of each swerve corner.
     */
    public void resetNavx() {
        NavX.reset();
    }

    public void resetDistance() {
        CornerFL.resetDrive();
        CornerFR.resetDrive();
        CornerBL.resetDrive();
        CornerBR.resetDrive();
    }

    public double getNavX() {
        return (NavX.getYaw() + 360) % 360;
    }

    public void xToggle() {
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

    public double getDistance() {
        return Math.min(Math.min(CornerFL.getDistance(), CornerFR.getDistance()),
                Math.min(CornerBL.getDistance(), CornerBR.getDistance()));
    }

    /**
     * Drives the swerve base using input x, y, and z values for movement and
     * rotation.
     * 
     * @param x The x-component of the movement vector.
     * @param y The y-component of the movement vector.
     * @param z The z-component (rotation) of the movement vector.
     */
    public void fieldCentric(double x, double y, double z) {

        // Obtain the current orientation of the robot from the NavX sensor and
        // normalize it to [0, 360)
        double robotOrientation = (NavX.getYaw() + 360) % 360;

        // Log the orientation for debugging purposes
        SmartDashboard.putNumber("NavX", robotOrientation);

        // Convert the target direction (given by x, y) into polar coordinates
        // (magnitude and angle)
        double mag = calculateMagnitude(x, y); // More efficient than sqrt(pow(x, 2) + pow(y, 2))
        double targetAngle = Math.toDegrees(Math.atan2(y, x));

        // Adjust the target angle based on the robot's current orientation
        targetAngle += robotOrientation;
        double adjustedAngleRad = Math.toRadians(targetAngle);

        // Convert the adjusted polar coordinates back to Cartesian coordinates
        x = mag * Math.cos(adjustedAngleRad);
        y = mag * Math.sin(adjustedAngleRad);

        x = Math.round(x * 100.0) / 100.0;
        y = Math.round(y * 100.0) / 100.0;

        robotCentric(x, y, z);

    }

    public void robotCentric(double x, double y, double z) {
        // Normalize magnitudes and calculate angles directly within the method
        double frx = x + z;
        double fry = y + z;
        double flx = x - z;
        double fly = y + z;
        double blx = x - z;
        double bly = y - z;
        double brx = x + z;
        double bry = y - z;

        double frmag = calculateMagnitude(frx, fry);
        double flmag = calculateMagnitude(flx, fly);
        double blmag = calculateMagnitude(blx, bly);
        double brmag = calculateMagnitude(brx, bry);

        // Calculate angles
        if (flmag != 0) {
            angleFL = Math.toDegrees(Math.atan2(flx, fly));
            if (angleFL < 0)
                angleFL += 360;
        }
        if (frmag != 0) {
            angleFR = Math.toDegrees(Math.atan2(frx, fry));
            if (angleFR < 0)
                angleFR += 360;
        }
        if (blmag != 0) {
            angleBL = Math.toDegrees(Math.atan2(blx, bly));
            if (angleBL < 0)
                angleBL += 360;
        }
        if (brmag != 0) {
            angleBR = Math.toDegrees(Math.atan2(brx, bry));
            if (angleBR < 0)
                angleBR += 360;
        }

        // Checking if we need to turn first without moving (Stops browning out)
        double flDist = calcDist(CornerFL.getWheelAngle(), angleFL);
        double frDist = calcDist(CornerFR.getWheelAngle(), angleFR);
        double brDist = calcDist(CornerBR.getWheelAngle(), angleBR);
        double blDist = calcDist(CornerBL.getWheelAngle(), angleBL);
        if ((flDist + frDist + brDist + blDist) / 4 > 45) {
            flmag = 0;
            frmag = 0;
            blmag = 0;
            brmag = 0;
        } else {

            // Normalize magnitudes
            double bigMag = Math.max(Math.max(flmag, frmag), Math.max(blmag, brmag));
            if (bigMag > 1) {
                flmag /= bigMag;
                frmag /= bigMag;
                blmag /= bigMag;
                brmag /= bigMag;
            }
        }

        if (x == 0 && y == 0 && z == 0 && xMode) {
            CornerFL.liveDrive(45, flmag);
            CornerFR.liveDrive(315, frmag);
            CornerBL.liveDrive(135, blmag);
            CornerBR.liveDrive(225, brmag);
        } else {
            // Drive each corner with the calculated angles and magnitudes
            CornerFL.liveDrive(angleFL, flmag);
            CornerFR.liveDrive(angleFR, frmag);
            CornerBL.liveDrive(angleBL, blmag);
            CornerBR.liveDrive(angleBR, brmag);
        }
    }

    public double calcDist(double starting, double ending) {
        double clockwise = (ending - starting + 360) % 360;
        double counterCwise = (starting - ending + 360) % 360;
        if (clockwise <= counterCwise) {
            return clockwise;
        } else {
            return counterCwise;
        }
    }

    private double calculateMagnitude(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

}
