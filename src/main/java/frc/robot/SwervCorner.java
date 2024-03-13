//                                   ,█▌ ▄█, █▀█ █▀█
//                     ;▄▄▄▄████▀▀═ ╘▀▀█^█▄█J█▄█ █▄█ `▀▀████▄▄▄▄▄▄╓
//             ,▄▄███▀▀▀"`       ,                          `¬█▀ ▐▌
//         ╓▄██▀▀'          ,▄▄█▀▀█▌    ;;                    └▀▀▀'     █▀▀█L
//      ▄██▀▀          ███▀▀▀▀▀▀█▄▀^  j█'└█▄▄▄▄▄▄▄▄▄▄▄▄;                ▀█▄██▄
//    ▄█▀-          ▄▄▄█⌐              ▀&█▀      `-`└▀▀▀▀▀▀███▄▄▄▄          "▀█▄
//   ██└      ╓▄▄██▀▀▀¬                                        ¬▀▀▀██▄▄▄      └██
//  ██    ╓▄██▀▀                                                      ▀▀██▄ç    ██
// ▐█   ▄█▀▀                                                              '▀█▄  ██▌
// █▌ ▄█▀                    *** Base Line Swerv Code ***                    ▀███▐█
// ███▀                          By: Brandon Mailloux                        ▄█▀ ▐█
// ▐██       ▄▄                                                           ▄██▀   █▌
//  ▐█▄     █"'█▄                                                    ,▄▄██▀¬   ,█▀
//   ▀█▄    ▀▀▀▀▀███▄▄ç                                        ╓▄▄███▀▀¬      ▄█▀
//    '██▀█⌐    ▄▄▄;╘▀▀▀▀████▄▄▄▄▄▄;,,,        ,,,;▄▄▄▄▄▄▄███▀▀▀▀╘  ▄▄▄   ▄█▀██▀
//     █▄ ▄▌    █▄"▀█   ▄⌐    "-'▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀'-¬     ▄▄▄ ▐▀██╘    █▄,▄▌
//      ¬└¬     ▄▀▀█▄  ███   █  █⌐  █ ▄▄▄▄▄  ╓▄▄     ████▄ ╓█▀▀▀▌  ▐█      --`
//              ▀█▄▄█⌐▐█ ▐▌  █  ██.j█ ▀▀█▀▀^██▀▀█    █  ▄█ █▌  ▐█  ▐█
//                 `  █▀▀██µ █  █▐█d█   █▌  ██▄▄     ██▀▀█ █▌  █▌  └▀
//                        ▐█ █  █ ▀██   █▌   "▀▀█L   █ ,▄█ ▀██▀▀
//                           ▀  ▀  ▀█   █▌  ██▄▄█-   █▀▀▀


//~~~~~~~~~~~~~~IMPORTS~~~~~~~~~~~~~~~~~
package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;

public class SwervCorner {

    // Constants for calculations
    private static final double WHEEL_TURN_RATIO = 6.778564; // Conversion factor for wheel velocity
    private static final double MOTOR_SPEED_SCALING = 100; // Scaling factor for motor speed adjustment
    private static final double MINIMUM_TURN_THRESHOLD = 0.25; // Minimum threshold for distance adjustment

    // Defining Swerve components
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final CANcoder encoder;
    private final RelativeEncoder driveEncoder;

    // Storing Corner Specific Settings
    private double rotateOffset; // Offset for rotation calibration
    private int curDirection; // Current direction of the wheel, 1 for forward, -1 for reverse
    private double RPMPosition; // Position of the wheel in RPM for resetting

    /**
     * Initializes a Swerve Corner with specified IDs and rotation offset.
     *
     * @param driveID       ID for the drive motor
     * @param turnID        ID for the turn motor
     * @param encoderID     ID for the encoder
     * @param rotateOffset  Initial rotation offset for calibration
     */
    public SwervCorner(int driveID, int turnID, int encoderID, double rotateOffset) {
        // Defining Motors
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

        // Defining Encoders
        driveEncoder = driveMotor.getEncoder();
        encoder = new CANcoder(encoderID);

        // Setting Corner Specific Settings
        this.rotateOffset = rotateOffset;
        this.curDirection = 1; // Initial direction set to forward
        RPMPosition = 0; // Initial RPM position reset
    }

    /**
     * Resets the RPM position to the current position of the drive encoder.
     */
    public void resetRPM() {
        RPMPosition = driveEncoder.getPosition();
    }

   
    /**
     * Calculates and returns the current wheel velocity.
     *
     * @return Wheel velocity adjusted by conversion factor.
     */
    public double WheelVelocity() {
        return Math.abs(driveEncoder.getVelocity() / WHEEL_TURN_RATIO * 31.9 * 60 / 100000);
    }

    /**
     * Calculates and returns the number of drive rotations since last reset.
     *
     * @return Number of rotations since last RPM reset.
     */
    public double getDriveRotations() {
        return (driveEncoder.getPosition() - RPMPosition) / WHEEL_TURN_RATIO * 0.319278;
    }

    /**
     * Flips the turn direction and adjusts the rotation offset.
     */
    public void flipTurn() {
        curDirection *= -1; // Reverse the current direction
        rotateOffset = (rotateOffset + 1.5) % 1; // Adjust the rotation offset
    }

    /**
     * Calculates and returns the current wheel angle considering the rotation offset.
     *
     * @return Current wheel angle in degrees.
     */
    public double getWheelAngle() {
        return ((this.getTurnEnc() - rotateOffset + 1) % 1) * 360;
    }

    /**
     * Retrieves the normalized encoder value adjusted for full rotation scale.
     *
     * @return Normalized encoder value adjusted to 0-1 scale.
     */
    public double getTurnEnc() {
        return (encoder.getAbsolutePosition().getValue() + 0.5);
    }

    /**
     * Calculates the shortest path to turn the corner to the desired angle.
     *
     * @param desAngle Desired angle to turn the corner to.
     */
    public void turnCorner(double desAngle) {
        double curAngle = getWheelAngle();
        double clockwise = (desAngle - curAngle + 360) % 360;
        double counterCwise = (curAngle - desAngle + 360) % 360;
        double shortestDistance;

        if (clockwise <= counterCwise) {
            if (clockwise < 90) {
                shortestDistance = -clockwise;
            } else {
                flipTurn();
                shortestDistance = counterCwise - 180;
            }
        } else {
            if (counterCwise < 90) {
                shortestDistance = counterCwise;
            } else {
                flipTurn();
                shortestDistance = -(clockwise - 180);
            }
        }

        if (Math.abs(shortestDistance) < MINIMUM_TURN_THRESHOLD) {
            shortestDistance = 0;
        }

        turnMotor.set(shortestDistance / MOTOR_SPEED_SCALING);
    }

    /**
     * Controls the live driving of the swerve corner, adjusting both angle and power.
     *
     * @param angle Desired angle for the turn.
     * @param power Power level for the drive motor.
     */
    public void liveDrive(double angle, double power) {
        turnCorner(angle);
        driveMotor.set(power * curDirection);
    }
}
