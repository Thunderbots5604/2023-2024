package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Values;
import org.firstinspires.ftc.teamcode.mecanumdrive.AutoDriveTrain;
import org.firstinspires.ftc.teamcode.robotParts.Arm;
import org.firstinspires.ftc.teamcode.robotParts.Camera;
//import org.firstinspires.ftc.teamcode.robotParts.Claw;
//import org.firstinspires.ftc.teamcode.robotParts.Intake;


@Autonomous(name = "RedFar", group = "Autonomous")

public class RedFar extends LinearOpMode{
    private AutoDriveTrain drive;
    private Arm arm;
    //    private Intake intake;
//    private Claw claw;
    private Camera camera;
    private int location;
    private double[][] positions ={{0, 16, 0}, {0, 2, 0}, {80, 2, 0}};
    private double ARM_HEIGHT;

    @Override
    public void runOpMode() {
        drive = new AutoDriveTrain(hardwareMap, "fl", "fr", "bl", "br", new double[] {Values.LATERAL_ERROR, Values.LONGITUDINAL_ERROR, Values.ANGLE_ERROR}, Values.INCHES_PER_TICK_LATERAL, Values.INCHES_PER_TICK_LONGITUDINAL, Values.RADIANS_PER_TICK);
        arm = new Arm(hardwareMap, "am1", "am2", 10, 1100, 0.05);
//        claw = new Claw(hardwareMap, "servo", 0, 0, 0);
//        intake = new Intake(hardwareMap, "intMotor");
        camera = new Camera(hardwareMap, telemetry, this);
        camera.init();
        double[] targetPosition = new double[3];

        waitForStart();
        location = camera.findRegion();
        sleep(500);

        for(int i = 0; i < positions.length; i++) {
            drive.setTargetLocation(positions[i]);
            while (!drive.moveToTargetLocation(.5)) {
                drive.updateCurrentLocation();
            }
            sleep(1000);
        }

//        arm.setTargetPosition(ARM_HEIGHT);
//        while(!arm.moveToTarget()){}
//        sleep(1000);

//        intake.setState("outputting");
//        sleep(2000);
//        intake.setState("off");
    }
}
