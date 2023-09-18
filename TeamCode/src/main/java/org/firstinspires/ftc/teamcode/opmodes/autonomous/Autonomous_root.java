package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Vision;
import org.firstinspires.ftc.teamcode.components.teaminfo.InitialSide;
import org.firstinspires.ftc.teamcode.components.teaminfo.TeamColor;
import org.firstinspires.ftc.teamcode.components.teaminfo.TeamInfo;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name="ADVANCED_STOP_AUTONOMOUS")
public class Autonomous_root extends LinearOpMode {
    //sleeve
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    @Override
    public void runOpMode() {
        // init chassis
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        Chassis chassis = new Chassis(motorFL, motorFR, motorBL, motorBR, imu, telemetry);
        chassis.init();

        // init arms
        DcMotor leftLift = hardwareMap.get(DcMotor.class, "leftArm");
        DcMotor rightLift = hardwareMap.get(DcMotor.class, "rightArm");
        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        Servo cam = hardwareMap.get(Servo.class, "cam");
        Arm arm = new Arm(leftLift, rightLift, gripper, cam);
        arm.init();
        arm.armTarget = 0;

        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Vision vision = new Vision(camera, telemetry, distanceSensor);
        vision.init();

        telemetry.addLine("waiting to start!");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            vision.searchTags();

            if(gamepad1.b || gamepad2.b) TeamInfo.teamColor = TeamColor.RED;
            if(gamepad1.x || gamepad2.x) TeamInfo.teamColor = TeamColor.BLUE;
            if(gamepad1.dpad_right || gamepad2.dpad_right) TeamInfo.initialSide = InitialSide.RIGHT;
            if(gamepad1.dpad_left || gamepad2.dpad_left) TeamInfo.initialSide = InitialSide.LEFT;

            if(vision.tagId() > 0) telemetry.addData("Detected tag ID:", vision.tagId());
            else telemetry.addLine("Don't see tag of interest :(");

            telemetry.addData("Team color", TeamInfo.teamColor);
            telemetry.addData("Initial side", TeamInfo.initialSide);
            telemetry.addData("distance", vision.distance());
            telemetry.update();

            sleep(20);
        }

        boolean parked = false;

        while(opModeIsActive() && !parked) {
            arm.closeGripper();
            chassis.runToPosition(-100, -100, -100, -100);

            chassis.resetEncoder();
            chassis.runToPosition(-2450, -2450, -2450, -2450);

            vision.setDetector("pole");

            chassis.runToPosition(-2100, -2100, -2100, -2100);
            chassis.resetEncoder();

            //RIGHT POSITION
            for(int i=0; i<1; i++){
                chassis.runToAngle((TeamInfo.initialSide == InitialSide.RIGHT) ? 36 : -36);

                chassis.resetEncoder();
                chassis.runToPosition(-50, -50, -50, -50);
                chassis.stop();

                arm.runToPosition(arm.middleJunction);
                adjust(chassis, vision, 0);

                while(vision.distance() > 200){
                    adjust(chassis, vision, 0);
                    chassis.forward(-.1);
                    telemetry.addData("distance", vision.distance());
                    telemetry.update();
                }

                arm.runToPosition(arm.highJunction);

                chassis.resetEncoder();
                chassis.runToPosition(-200,-200,-200,-200);
                chassis.stop();

                sleep(5000);

                arm.fall();
                arm.openGripper();

                vision.setDetector("cone");

                chassis.runToPosition(40, 40, 40, 40);

                chassis.runToAngle((TeamInfo.initialSide == InitialSide.RIGHT) ? -90 : 90);

                chassis.resetEncoder();
                if(TeamInfo.initialSide == InitialSide.RIGHT){
                    if(vision.tagId() == RIGHT) chassis.runToPosition(-1100, -1100, -1100, -1100);
                    else if(vision.tagId() == MIDDLE) chassis.runToPosition(-50, -50, -50, -50);
                    else if(vision.tagId() == LEFT) chassis.runToPosition(1000, 1000, 1000, 1000);
                } else if(TeamInfo.initialSide == InitialSide.LEFT) {
                    if(vision.tagId() == RIGHT) chassis.runToPosition(1000, 1000, 1000, 1000);
                    else if(vision.tagId() == MIDDLE) chassis.runToPosition(-50, -50, -50, -50);
                    else if(vision.tagId() == LEFT) chassis.runToPosition(-1100, -1100, -1100, -1100);
                }
            }
            parked = true;
        }
    }

    void adjust(Chassis chassis, Vision vision, int mode){
        final int turn = 0;
        final int strafe = 1;
        while(Math.abs(vision.getAutonPipeline().differenceX()) > 9) {
            double power = (Math.abs(vision.getAutonPipeline().differenceX()) > 20) ? -0.175 * Math.abs(vision.getAutonPipeline().differenceX())/vision.getAutonPipeline().differenceX() : (vision.getAutonPipeline().differenceX())/150.0;
            if(mode == turn) chassis.turn(power);
            if(mode == strafe) chassis.strafe(power);
            telemetry.addData("difference", vision.getAutonPipeline().differenceX());
            telemetry.update();
        }
    }
}
