package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="RemoteControl")
public class TeleOp1 extends LinearOpMode {


    DcMotor motor_FLM = null;
    DcMotor motor_RLM = null;
    DcMotor motor_FRM = null;
    DcMotor motor_RRM = null;

    DcMotor motor_Extend = null;
    DcMotor motor_Elbow = null;
    DcMotor motor_Riser = null;

    Servo motor_Gripper = null;

    IMU imu;

    double gripperOpenPosn = 0.99;
    double gripperSampleClosePosn = 0.4;
    double gripperClosePosn = 0.0;
    int armExtensionsTol = 10;
    int armExtendHomePosn = -10;
    int armElbowHomePosn = 10;
    int elbowZeroDegreeOffset = 874;
    double elbowCountPerDegree = 14.67;
    int extendHomePosn = -10;
    int extendStartPosn = -550;
    int elbowStartAngle = 85;
    int extendHangPosn = -250;  //-800;
    int elbowHangAngle = 20;
    double elbowPower = 0;
    double yaw = 0;
    double Kp = 0.04;
    double Kd = 0.0004;
    double lastError = 0;
    double maxPID_Power = 0.75;
    double drive = 0;
    double strafe = 0;
    double twist = 0;


    ElapsedTime timer = new ElapsedTime();



    @Override
    public void runOpMode(){
        initHardware();
        distTelemetry();
        while(!isStarted()){
            distTelemetry();
            yaw=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        waitForStart();

        while (opModeIsActive()){
            distTelemetry();
            motor_Elbow.setPower(gamepad1.right_stick_y*-1);
            motor_Extend.setPower(gamepad1.right_stick_x);
            if(Math.abs(gamepad1.right_trigger-gamepad1.left_trigger)>0.1) {
                drvStraight(gamepad1.right_trigger - gamepad1.left_trigger);
            }
            else {
                drvStraight(0);
            }
            drive = gamepad1.right_trigger-gamepad1.left_trigger;
//            strafe(gamepad1.left_stick_x);
            yaw = yaw+gamepad1.left_stick_x;
            if(Math.abs(yaw-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))>2) {
                PIDControl(yaw, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            }
            if(gamepad1.a){
                motor_Gripper.setPosition(gripperOpenPosn);
            }
            if(gamepad1.b){
                motor_Gripper.setPosition(gripperClosePosn);
            }
            if(gamepad1.x){
                motor_Gripper.setPosition(gripperSampleClosePosn);
            }
        }
    }



    public void drvStraight(double pwr){
        motor_FLM.setPower(-pwr);
        motor_FRM.setPower(pwr);
        motor_RLM.setPower(-pwr);
        motor_RRM.setPower(pwr);
    }
    public void strafe(double dr){
        motor_FLM.setPower(-dr);
        motor_FRM.setPower(-dr);
        motor_RLM.setPower(dr);
        motor_RRM.setPower(dr);
    }

    public double calcStrafeDist(){
        return (motor_FLM.getCurrentPosition()+motor_FRM.getCurrentPosition()-motor_FLM.getCurrentPosition()-motor_RRM.getCurrentPosition());
    }


    public void initHardware(){
        initDriveMotors();
        initArmExtensions();
        initServo();
        initializeIMU();
    }
    public void initDriveMotors(){
        motor_FLM = hardwareMap.dcMotor.get("FLMotor");
        motor_RLM = hardwareMap.dcMotor.get("RLMotor");
        motor_FRM = hardwareMap.dcMotor.get("FRMotor");
        motor_RRM = hardwareMap.dcMotor.get("RRMotor");
        motor_FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_RLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_RRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void initArmExtensions(){
        motor_Extend = hardwareMap.dcMotor.get("ExtendMotor");
        motor_Elbow = hardwareMap.dcMotor.get("ElbowMotor");
    }
    public void initializeIMU(){
        imu=hardwareMap.get(IMU.class, "IMU");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    public void initServo(){
        motor_Gripper = hardwareMap.servo.get("servoGripper");
    }


    public double PIDControl(double refrence, double state) {
        double error = (refrence - state);
        telemetry.addData("Error: ", error);
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd);
        output = Math.min(output, maxPID_Power);
        output = Math.max(output, -maxPID_Power);
        return output;
    }



    public void distTelemetry(){
        telemetry.addData("Elbow Motor Encoder", motor_Elbow.getCurrentPosition());
        telemetry.addData("Extend Motor Encoder", motor_Extend.getCurrentPosition());
        telemetry.addData("Game Pad Left Y", gamepad1.left_stick_y);
        telemetry.addData("Game Pad Left X", gamepad1.left_stick_x);
        telemetry.addData("Game Pad Right Y", gamepad1.right_stick_y);
        telemetry.addData("Game Pad Right X", gamepad1.right_stick_x);
        telemetry.addData("Game Pad DPAD Left", gamepad1.dpad_left);
        telemetry.addData("Game Pad DPAD Right", gamepad1.dpad_right);
        telemetry.addData("Game Pad DPAD Up", gamepad1.dpad_up);
        telemetry.addData("Game Pad DPAD Down", gamepad1.dpad_down);
        telemetry.addData("Game Pad Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Game Pad Right Bumper", gamepad1.right_bumper);
        telemetry.addData("Game Pad Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Game Right Trigger", gamepad1.right_trigger);
//        telemetry.addData("Game Pad DPAD", gamepad1.dpad_left);
//        telemetry.addData("Extend Motor Encoder", motor_Extend.getCurrentPosition());
//        telemetry.addData("Extend Motor Encoder", motor_Extend.getCurrentPosition());
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Yaw Actual (Z)", "%.2f Deg. (Heading)", yaw);
        telemetry.update();
    }

}
