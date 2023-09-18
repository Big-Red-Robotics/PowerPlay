package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Vision;
import org.firstinspires.ftc.teamcode.components.teaminfo.TeamColor;
import org.firstinspires.ftc.teamcode.components.teaminfo.TeamInfo;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class PoleVisionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        Chassis chassis = new Chassis(motorFL, motorFR, motorBL, motorBR, imu, telemetry);
        chassis.init();

        DcMotor leftLift = hardwareMap.get(DcMotor.class, "leftArm");
        DcMotor rightLift = hardwareMap.get(DcMotor.class, "rightArm");
        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        Servo cam = hardwareMap.get(Servo.class, "cam");
        Arm arm = new Arm(leftLift, rightLift, gripper, cam);
        arm.init();

        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Vision vision = new Vision(camera, telemetry, distanceSensor);
        vision.init();

        telemetry.addLine("waiting to start!");
        telemetry.update();

        vision.setDetector("pole");

        waitForStart();

        adjust(chassis, vision, 0);
        chassis.stop();

        while(opModeIsActive()){
            telemetry.addData("distance", vision.distance());
            telemetry.update();
        }
    }

    void adjust(Chassis chassis, Vision vision, int mode){
        final int turn = 0;
        final int strafe = 1;
        while(Math.abs(vision.getAutonPipeline().differenceX()) > 3) {
            double power = (vision.getAutonPipeline().differenceX() < 0) ? 0.2 : -0.2;
            if(mode == turn) chassis.turn(power);
            if(mode == strafe) chassis.strafe(power);
            telemetry.addData("difference", vision.getAutonPipeline().differenceX());
            telemetry.update();
        }
        chassis.stop();
    }
}
