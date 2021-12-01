package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class DriveSubsystem extends SubsystemBase {

    private final Motor leftFrontMotor;
    private final Motor leftRearMotor;
    private final Motor rightFrontMotor;
    private final Motor rightRearMotor;
    private final MotorGroup leftMotorGroup;
    private final MotorGroup rightMotorGroup;
    private final DifferentialDrive drive;
    private boolean isInverted = false;
    private RevIMU imu;

    private double gearRatio = 12 / 1;
    private double ticksPerRotation = 305;
    private double inchesPerRotation = 3.54331 * Math.PI;
    private double inchesPerTick = inchesPerRotation / ticksPerRotation;

    private double autonMaxPower = 0.7;

    Telemetry telemetry;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftFrontMotor = new Motor(hardwareMap, "leftFrontMotor");
        leftRearMotor = new Motor(hardwareMap, "leftRearMotor");
        rightFrontMotor = new Motor(hardwareMap, "rightFrontMotor");
        rightRearMotor = new Motor(hardwareMap, "rightRearMotor");

        leftMotorGroup = new MotorGroup(leftFrontMotor, leftRearMotor);
        rightMotorGroup = new MotorGroup(rightFrontMotor, rightRearMotor);

        leftMotorGroup.setInverted(true);
        rightMotorGroup.setInverted(true);
        leftFrontMotor.encoder.setDirection(Motor.Direction.REVERSE);
        leftRearMotor.encoder.setDirection(Motor.Direction.REVERSE);

        drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

        imu = new RevIMU(hardwareMap, "imu");
        imu.init();

        this.telemetry = telemetry;
    }

    private void setInverted(Boolean isInverted) {
        this.isInverted = isInverted;
    }

    public void manualDrive(double power, double turn){
        power = isInverted ? power * -1 : power;
        drive.arcadeDrive(power, turn);
        //telemetry.addData("turn power", turn);
        //telemetry.update();
    }

    public void stop() {
        drive.stop();
    }

    public void resetEncoders() {
        leftFrontMotor.resetEncoder();
        rightFrontMotor.resetEncoder();
    }

    public int getCurrentPosition() {
       return (leftFrontMotor.getCurrentPosition() + rightFrontMotor.getCurrentPosition()) / 2;
    }

    public int getLeftPosition() {
        return leftFrontMotor.getCurrentPosition();
    }

    public int getRightPosition() {
        return rightFrontMotor.getCurrentPosition();
    }

    public double getAutonMaxPower() {
        return autonMaxPower;
    }

    public double setAutonMaxPower(double power) {
        return this.autonMaxPower = power;
    }

    public double getCurrentPositionInInches() {
        return getCurrentPosition() * inchesPerTick;
    }


    private enum State {
        FORWARD,
        BACKWARD
    }
    private State currentState = State.FORWARD;

    public String getState() {
        return this.currentState.name();
    }

    public void setState(String newState) {
        switch(newState) {
            case "FORWARD" :
                setInverted(false);
                this.currentState = State.FORWARD;
                break;
            case "BACKWARD" :
                setInverted(true);
                this.currentState = State.BACKWARD;
                break;
        }
    }

    public double getAbsoluteAngle() {
        return imu.getAbsoluteHeading();
    }

}
