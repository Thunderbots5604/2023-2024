package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Values;
import org.firstinspires.ftc.teamcode.mecanumdrive.AutoDriveTrain;
import org.firstinspires.ftc.teamcode.robotParts.Arm;
import org.firstinspires.ftc.teamcode.robotParts.Camera;
//import org.firstinspires.ftc.teamcode.robotParts.Claw;
//import org.firstinspires.ftc.teamcode.robotParts.Intake;


@Autonomous(name = "RedClosePark", group = "Autonomous")

public class RedClosePark extends LinearOpMode{
    private AutoDriveTrain drive;

    @Override
    public void runOpMode() {
        drive = new AutoDriveTrain(hardwareMap, "fl", "fr", "bl", "br", new double[] {Values.LATERAL_ERROR, Values.LONGITUDINAL_ERROR, Values.ANGLE_ERROR}, Values.INCHES_PER_TICK_LATERAL, Values.INCHES_PER_TICK_LONGITUDINAL, Values.RADIANS_PER_TICK);

        waitForStart();
        sleep(500);

        double[] targetPosition = new double[]{-40, 0, 0};
        drive.setTargetLocation(targetPosition);
        while (!drive.moveToTargetLocation(.5)) {
            drive.updateCurrentLocation();
        }
    }
}
