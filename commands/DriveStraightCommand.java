package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
@Config
public class DriveStraightCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private Telemetry telemetry;
    private PIDController pidController;
    private double setPoint;
    private double DRIVE_FORWARD_KP = 0.06;
    private double DRIVE_FORWARD_KS = .26;

    private double startingAngle;
    private Rotation2d currentAngleOffset;
    public static double DRIVE_TURN_KP = 0.065;

    public DriveStraightCommand(DriveSubsystem driveSubsystem, double setPoint, Telemetry telemetry) {
        this.driveSubsystem = driveSubsystem;
        this.setPoint = setPoint;
        this.telemetry = telemetry;

        pidController = new PIDController(DRIVE_FORWARD_KP, 0.0, 0.0);
        pidController.setTolerance(0.8);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.resetEncoders();
        pidController.setSetPoint(setPoint);

        startingAngle = driveSubsystem.getAbsoluteAngle();
    }

    @Override
    public void execute() {
        double maxPower = driveSubsystem.getAutonMaxPower();

        //pidController.setP(DRIVE_FORWARD_KP);
        //pidController.setSetPoint(setPoint);

        currentAngleOffset =
         Rotation2d.fromDegrees(driveSubsystem.getAbsoluteAngle()).minus(Rotation2d.fromDegrees(startingAngle));
        double turn = currentAngleOffset.getDegrees() * DRIVE_TURN_KP;

        double power = pidController.calculate(driveSubsystem.getCurrentPositionInInches());

        // deadband kS
        if(Math.abs(power) > .05) {
            power = power + DRIVE_FORWARD_KS * Math.signum(power);
        }

        // limit max power to 0.7
        if(Math.abs(power) > maxPower) power = maxPower * Math.signum(power);

        driveSubsystem.manualDrive(power, turn);

//        telemetry.addData("ticks", driveSubsystem.getCurrentPosition());
//        telemetry.addData("currentPosition", driveSubsystem.getCurrentPositionInInches());
          telemetry.addData("error", pidController.getPositionError());
          telemetry.addData("power", power);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetPoint();
    }
}
