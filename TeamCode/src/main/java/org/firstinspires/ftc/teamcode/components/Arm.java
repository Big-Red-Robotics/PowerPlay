package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Arm {
    public DcMotor leftLift;
    public DcMotor rightLift;
    public Servo gripper;
    public Servo cam;

    //TODO: CHANGE VALUE TO 420, 630, 910 FOR 11166-RC!!!!
    //150, 300, 450 for test robot
    public final int lowJunction = 420;
    public final int middleJunction = 630;
    public final int highJunction = 910;
    public int armTarget = 0;

    public final int fiveStack = 170;
    public final int fourStack = 140;
    public final int threeStack = 110;
    public final int twoStack = 80;
    public final int ground = 0;

    public enum ManualArm {drop, raise, none};

    public ManualArm manualArm = ManualArm.none;

    public Arm(DcMotor lLift, DcMotor rLift, Servo g, Servo c){
        this.leftLift = lLift;
        this.rightLift = rLift;
        this.gripper = g;
        this.cam = c;
    }

    public void init(){
        //TODO: REVERSE rightLift FOR 11166-RC!!!
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        //resetting encoders at home level
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //setting the motors into the necessary mode for using the encoders
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void openGripper() {
        gripper.setPosition(0.97);
    }

    public void closeGripper() { gripper.setPosition(0.75); }

    public void runToPosition(int position) {
        armTarget = position;

        leftLift.setTargetPosition(armTarget);
        rightLift.setTargetPosition(armTarget);

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (rightLift.isBusy() && leftLift.isBusy()) {
            if (leftLift.getCurrentPosition() < leftLift.getTargetPosition() && rightLift.getCurrentPosition() < rightLift.getTargetPosition()) {
                leftLift.setPower(1.0);
                rightLift.setPower(1.0);
            } else {
                leftLift.setPower(0.0);
                rightLift.setPower(0.0);
            }
        }
    }
    
    public void fall() {
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(getCurrentPosition() > middleJunction) {
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftLift.setPower(0.0);
            rightLift.setPower(0.0);
        }

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setPower(0.0);
        rightLift.setPower(0.0);
    }

    public void setArmPower(Gamepad gamepad, double power, int camLevel, boolean stack) {
        if (rightLift.isBusy() && leftLift.isBusy()){
            if (getCurrentPosition() > armTarget) {
                if (getCurrentPosition() > 650 && getTargetPosition() != 650) {
                    leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } else {
                    leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                leftLift.setPower(0.0);
                rightLift.setPower(0.0);
            } else {
                leftLift.setPower(power);
                rightLift.setPower(power);
            }
        }

        leftLift.setTargetPosition(armTarget);
        rightLift.setTargetPosition(armTarget);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (armTarget > (getCurrentPosition() - 50) && armTarget < (getCurrentPosition() + 50) && stack) {
            armTarget = camLevel;
            stack = false;
        };
    }

    public void armTriggers(Gamepad gamepad) {
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (gamepad.left_trigger > 0) {
            armTarget = getCurrentPosition();

            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftLift.setPower(0.0);
            rightLift.setPower(0.0);
        }

        if (gamepad.right_trigger > 0) {
            armTarget = getCurrentPosition();

            leftLift.setPower(gamepad.right_trigger*1.5);
            rightLift.setPower(gamepad.right_trigger*1.5);
        }
    }

    public int getCurrentPosition(){ return (leftLift.getCurrentPosition() + rightLift.getCurrentPosition()) / 2; }

    public int getTargetPosition(){ return (leftLift.getTargetPosition() + rightLift.getTargetPosition()) / 2; }

    public double getPower(){ return (leftLift.getPower() + rightLift.getPower()) / 2; }
}
