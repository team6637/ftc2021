package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SpinnerSubsystem extends SubsystemBase {
    private Motor spinnermotor;
    public static double power = 0.7;

    public SpinnerSubsystem(HardwareMap hardwareMap) {
        spinnermotor = new Motor(hardwareMap, "spinner");
    }

    public void move() {
        spinnermotor.set(power);
    }
    public void stop() {
        spinnermotor.set(0.0);
    }
}
