package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystem extends SubsystemBase {
    private final Motor armMotor;
    private final double kP = 0.001;
    private double setPoint = 0.0;
    private final PIDController pidController;

    public ArmSubsystem(HardwareMap hardwareMap) {
        armMotor = new Motor(hardwareMap, "arm");
        armMotor.setRunMode(Motor.RunMode.RawPower);
        armMotor.setInverted(false);
        armMotor.encoder.setDirection(Motor.Direction.FORWARD);
        pidController = new PIDController(kP, 0.0, 0.00001);
    }

    public void move(double power) {
        armMotor.set(power);
    }

    public int getCurrentPosition() {
        return armMotor.getCurrentPosition();
    }

    @Override
    public void periodic() {
        double output = pidController.calculate(getCurrentPosition(), setPoint);

        if(getState() == "FLOOR" && getCurrentPosition() < 2000 && Math.abs(output) > 0.1) {
            //output = -0.08;
        }
        move(output * 0.9);

    }

    private enum State {
        FLOOR,
        FORWARD_BOTTOM,
        FORWARD_MIDDLE,
        FORWARD_TOP,
        BACKWARD_TOP,
        BACKWARD_MIDDLE
    }
    private State currentState = State.FLOOR;

    public String getState(){
        return this.currentState.name();
    }

    public void setState(String newState) {
        switch(newState) {
            case "FLOOR":
                setPoint = 0.0;
                currentState = State.FLOOR;
                break;
            case "FORWARD_BOTTOM" :
                setPoint = 850.0;
                currentState = State.FORWARD_BOTTOM;
                break;
            case "FORWARD_MIDDLE" :
                setPoint = 1400.0;
                currentState = State.FORWARD_MIDDLE;
                break;
            case "FORWARD_TOP" :
                setPoint = 2100.0;
                currentState = State.FORWARD_TOP;
                break;
            case "BACKWARD_TOP" :
                setPoint = 5000.0;
                currentState = State.BACKWARD_TOP;
                break;
            case "BACKWARD_MIDDLE" :
                setPoint = 5600.0;
                currentState = State.BACKWARD_MIDDLE;
                break;
        }
    }
}
