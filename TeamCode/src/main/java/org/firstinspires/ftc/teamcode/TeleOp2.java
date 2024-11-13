package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="RemoteControl2")
public class TeleOp2 extends LinearOpMode {


    DcMotor motor_FLM = null;
    DcMotor motor_RLM = null;
    DcMotor motor_FRM = null;
    DcMotor motor_RRM = null;

    DcMotor motor_Extend = null;
    DcMotor motor_Elbow = null;
    DcMotor motor_Riser = null;

    Servo motor_Gripper = null;


    DistanceSensor FRDist = null;
    DistanceSensor FLDist = null;
    DistanceSensor RLDist = null;
    DistanceSensor RRDist = null;
    DistanceSensor RightDist = null;
    DistanceSensor LeftDist = null;


    IMU imu;

    double gripperOpenPosn = 0.6;
    double gripperClosePosn = 0.15;
    int elbowRequest = 0;
    int extendRequest = 0;
    int armExtensionsTol = 50;
    double yaw = 0;
    double yawReq = 0;
    double Kp = 0.04;
    double Kd = 0.0004;
    double lastError = 0;
    double maxPID_Power = 0.75;
    double drive = 0;
    double strafe = 0;
    double twist = 0;
    boolean armPosnCtrl = false;
    double Kp_posn = 0.05;
    double Kd_posn = 0.004;
    double maxPID_Power_posn = 0.5;
    double power = 0;
    int hangDist = 7;
    double hangDistTol = 1;
    int extendCmd = 1200;
    double avgFrontDist = 0;
    int hangExtendPrepPosn = 1200;                  //was -650
    int hangElbowPrepPosn = 775;                    // was 2000
    int hangExtendPosn = 1000;                 // was -450
    int hangElbowPosn = 1250;                    // was 800
    int hangElbowFinalPosn = 900;                    // was 800
    boolean finalTry = false;
    boolean extendPosnCtrl = false;
    boolean basketRequest = false;
    boolean retractArm = false;



    ElapsedTime timer = new ElapsedTime();



    @Override
    public void runOpMode(){
        initHardware();
        distTelemetry();
        while(!isStarted()){
            distTelemetry();
        }

        waitForStart();

        while (opModeIsActive()){
            distTelemetry();
            if(gamepad2.right_stick_y<-0.1) {
//                motor_Elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    motor_Elbow.setPower(gamepad2.right_stick_y * -0.5);
                armPosnCtrl = true;
                avgFrontDist = 1;
            }
            else {
                if (gamepad2.right_stick_y > 0.1) {
//                    motor_Elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                        motor_Elbow.setPower(gamepad2.right_stick_y * -0.1);
                    armPosnCtrl = true;
                    avgFrontDist = 2;
                }
            }

            if(gamepad2.a){
                armHangSpecimenPosition(40, 1825);
                armPosnCtrl = false;
                avgFrontDist = 3;
            }
        }
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    public void pidDrive(double dr) {
        motor_FLM.setPower(dr);
        motor_FRM.setPower(dr);
        motor_RLM.setPower(dr);
        motor_RRM.setPower(dr);
    }


    public void hangSpecimen(int a){
        double current = FRDist.getDistance(DistanceUnit.INCH);
        power = PIDControlK(hangDist, current, Kp_posn, Kd_posn, maxPID_Power_posn);
        mecanum_drive(-power, 0, 0);
    }

    public void mecanum_drive(double dr, double st, double tw) {
        double power = 0;
        if(Math.abs(st)>0) {
            power = PIDControl(0, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            if (!gamepad1.left_bumper) {
                power = 0;
            }
        }
        double[] speeds = {
                (dr + st + tw + power),
                (dr - st - tw + power),
                (dr - st + tw + power),
                (dr + st - tw + power)
        };
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] *= 0.5/max;
        }
        motor_FLM.setPower(-speeds[0]);
        motor_FRM.setPower(speeds[1]);
        motor_RLM.setPower(-speeds[2]);
        motor_RRM.setPower(speeds[3]);
    }


    public void resetArmExtensions(){
        motor_Extend.setPower(-0.5);
        motor_Elbow.setPower(-0.2);
        motor_Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_Extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_RLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_RRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu.resetYaw();
        motor_FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_RLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_RRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void armHangSpecimenPosition(int extend, int elbow){
//        extendRequest = extend;
//        elbowRequest = elbow;
        motor_Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Elbow.setTargetPosition(elbow);
        motor_Elbow.setPower(0.9);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Extend.setTargetPosition(extend);
        motor_Extend.setPower(0.8);
        motor_Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    public void initHardware(){
        initDriveMotors();
        initArmExtensions();
        initServo();
        initDistanceSens();
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
    public void initDistanceSens(){
        FRDist = hardwareMap.get(DistanceSensor.class, "FRDistance");
        FLDist = hardwareMap.get(DistanceSensor.class, "FLDistance");
        RRDist = hardwareMap.get(DistanceSensor.class, "RRDistance");
        RLDist = hardwareMap.get(DistanceSensor.class, "RLDistance");
        RightDist = hardwareMap.get(DistanceSensor.class, "rightDistance");
        LeftDist = hardwareMap.get(DistanceSensor.class, "leftDistance");
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
        if(error>180){
            error = error - 360;
        }
        if(error<-180){
            error = error + 360;
        }
        telemetry.addData("Error: ", error);
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd);
        output = Math.min(output, maxPID_Power);
        output = Math.max(output, -maxPID_Power);
        return output;
    }


    public double PIDControlK(double refrence, double state, double p, double d, double pwr) {
        double error = (refrence - state);
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * p) + (derivative * d);
        output = Math.min(output, pwr);
        output = Math.max(output, -pwr);
        return output;
    }




    public void distTelemetry(){
//        telemetry.addData("Left Distance in Inches", "%.2f Inches", LeftDist.getDistance(DistanceUnit.INCH));
//        telemetry.addData("Right Distance in Inches", "%.2f Inches", RightDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Front Left Distance in Inches", "%.2f Inches", FLDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Front Right Distance in Inches", "%.2f Inches", FRDist.getDistance(DistanceUnit.INCH));
//        telemetry.addData("Rear Left Distance in Inches", "%.2f Inches", RLDist.getDistance(DistanceUnit.INCH));
//        telemetry.addData("Rear Right Distance in Inches", "%.2f Inches", RRDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Elbow Motor Request", elbowRequest);
        telemetry.addData("Elbow Motor Encoder", motor_Elbow.getCurrentPosition());
        telemetry.addData("Extend Motor Request", extendRequest);
        telemetry.addData("Extend Motor Encoder", motor_Extend.getCurrentPosition());
//        telemetry.addData("FLM Encoder", motor_FLM.getCurrentPosition());
//        telemetry.addData("FRM Encoder", motor_FRM.getCurrentPosition());
//        telemetry.addData("RLM Encoder", motor_RLM.getCurrentPosition());
//        telemetry.addData("RRM Encoder", motor_RRM.getCurrentPosition());
        telemetry.addData("PID power", power);
        telemetry.addData("Average Distance", avgFrontDist);
        telemetry.addData("Extend Cmd", extendCmd);
        telemetry.addData("Right Y  ", gamepad2.right_stick_y);
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }

}
