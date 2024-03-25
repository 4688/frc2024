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

public class SwervCorner {

    // Constants for calculations
    private static final double WHEEL_TURN_RATIO = 6.778564; // Conversion factor for wheel velocity
    private static final double MOTOR_SPEED_SCALING = 100; // Scaling factor for motor speed adjustment
    private static final double MINIMUM_TURN_THRESHOLD = 0.25; // Minimum threshold for distance adjustment
    private static final double CIRCUMFRENCE_OF_WHEEL = 0.319278;

    // Defining Swerve components
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final CANcoder encoder;
    private final RelativeEncoder driveEncoder;

    // Storing Corner Specific Settings
    private double rotateOffset; // Offset for rotation calibration
    private int curDirection; // Current direction of the wheel, 1 for forward, -1 for reverse
    private double last_drive_position; // Position of the wheel for resetting



    

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
        last_drive_position = 0; // Initial RPM position reset
    }

    public void resetDrive() {
        last_drive_position = driveEncoder.getPosition();
    }

    public double WheelVelocity() {
        return Math.abs(driveEncoder.getVelocity() / WHEEL_TURN_RATIO * 31.9 * 60 / 100000);
    }

    public double getDistance() {
        return Math.abs((driveEncoder.getPosition() - last_drive_position) / WHEEL_TURN_RATIO * CIRCUMFRENCE_OF_WHEEL);
    }

    public void flipTurn() {
        curDirection *= -1; // Reverse the current direction
        rotateOffset = (rotateOffset + 1.5) % 1; // Adjust the rotation offset
    }

    public double getWheelAngle() {
        return ((this.getTurnEnc() - rotateOffset + 1) % 1) * 360;
    }

    public double getTurnEnc() {
        return (encoder.getAbsolutePosition().getValue() + 0.5);
    }

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

    public void liveDrive(double angle, double power) {
        turnCorner(angle);
        driveMotor.set(power * curDirection);
    }

    public void setToBrakeMode(){
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setToGlideMode(){
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setToCoastMode(){
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

}