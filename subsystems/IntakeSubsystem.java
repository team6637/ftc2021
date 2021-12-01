package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem extends SubsystemBase {
    private Motor intakemotor;
    private ServoEx servo;
    private boolean sensorSeesObject = false;
    private Rev2mDistanceSensor sensor;

    public  IntakeSubsystem(HardwareMap hardwareMap) {
        intakemotor = new Motor(hardwareMap, "intake");
        servo = new SimpleServo(hardwareMap, "flapper", -200, 5);
        servo.setInverted(true);
        sensor = hardwareMap.get(Rev2mDistanceSensor.class, "intakeSensor");
    }

    @Override
    public void periodic() {
        double sensorDistance = sensor.getDistance(DistanceUnit.INCH);

        if(sensorDistance < 4.5) {
            sensorSeesObject = true;
        } else {
            sensorSeesObject = false;
        }
    }

    public boolean getSensorSeesObject() {
        return sensorSeesObject;
    }

    public void move(double power) {
        intakemotor.set(power);
    }

    public void openFlapper() {
        servo.turnToAngle(-60.0);
        move(-0.3);
    }

    public void closeFlapper() {
        servo.turnToAngle(0.0);
    }

    public void flapperHoldObject() {
        servo.turnToAngle(20.0);
    }

    public double getFlapperAngle() {
        return servo.getAngle();
    }

}
