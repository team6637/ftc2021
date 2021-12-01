package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class TurnToAngle extends CommandBase {

    private DriveSubsystem drive;
    //private double offsetAngle;
    private double maxPower = 0.7;
    private double turnKp = 0.004;
    private double turnKs = 0.54;
    private double offsetAngle = 0.0;
    private PIDController pidController;
    private double tolerance = 2.0;
    Telemetry telemetry;

    Rotation2d setPoint;

    public TurnToAngle(DriveSubsystem drive, double offsetAngle, Telemetry telemetry) {
        this.drive = drive;
        this.offsetAngle = offsetAngle;
        this.telemetry = telemetry;
        addRequirements(drive);

        pidController = new PIDController(turnKp, 0.0, 0.0);
        pidController.setTolerance(tolerance);

    }

    @Override
    public void initialize() {
        // rotation2d of original heading plus the offset target: gives degrees within pi bounds
        setPoint = Rotation2d.fromDegrees(drive.getAbsoluteAngle()).plus(Rotation2d.fromDegrees(offsetAngle));
        pidController.setSetPoint(setPoint.getDegrees());
    }

    @Override
    public void execute() {
        pidController.setP(turnKp);

        // current offset in degrees from setpoint. kept withing pi bounds
        double offsetFromSetPoint = Math.IEEEremainder(drive.getAbsoluteAngle() - setPoint.getDegrees(), 180);

        // error: orginal setpoint plus current offset
        double error = setPoint.getDegrees() + offsetFromSetPoint;

        double power = pidController.calculate(error);

        if(Math.abs(power) > 0.02) power = power + Math.signum(power) * turnKs;

        if(Math.abs(power) > maxPower) power = maxPower * Math.signum(power);

        drive.manualDrive(0.0, -power);

//        telemetry.addData("setPoint angle", setPoint.getDegrees());
   //       telemetry.addData("turn error", error);
//
//        telemetry.addData("power", power);
//        telemetry.addData("absolute angle", drive.getAbsoluteAngle());
//        telemetry.addData("turnKp", turnKp);
//        telemetry.addData("error", pidController.getPositionError());
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
