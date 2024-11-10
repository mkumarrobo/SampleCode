package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.OpenCv.AprilTagDetectionPipeline;

import java.util.ArrayList;


/**
 * Control Hub
 * Motor Port 0 FLMotor
 * Motor Port 1 RLMotor
 * Motor Port 2 RRMotor
 * Motor Port 3 FRMotor
 *
 * I2C Port 0 RRDistance
 * I2C Port 1 FLDistance
 * I2C Port 2 FRDistance
 * I2C Port 3 RLDistance
 *
 * Extension Hub Motor 0 Extend Motor
 * Extension Hub Motor 1 Elbow Motor
 * Extension Hub Motor 2 Riser Motor
 */


@Autonomous(name="TestSens")
public class Autonomous1 extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

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

    int tag11 = 11; // Tag ID 18 from the 36h11 family
    int tag12 = 12;
    int tag13 = 13;
    int tag14 = 14;
    int tag15 = 15;
    int tag16 = 16;

    double Kp = 0.04;
    double Kd = 0.0004;
    double maxPID_Power = 0.75;
    double Kp_posn = 0.2;
    double Kd_posn = 0.04;
    double maxPID_Power_posn = 0.3;
    double Kp_strf = 0.2;
    double Kd_strf = 0.04;
    double maxPID_Power_strf = 0.9;
    double lastError = 0;
    double distanceTol = 1;
    double power=0;
    double twist = 0;
    double gripperOpenPosn = 0.99;
    double gripperSampleClosePosn = 0.0;
    double gripperClosePosn = 0.0;
    int autonomousStage = 0;
    int armExtensionsTol = 10;
    int armExtendHomePosn = -10;
    int armElbowHomePosn = 10;
    int armRiserHomePosn = 0;
    int straightDirection = 0;
    int yellowSampleDirection = 90;

    int elbowZeroDegreeOffset = 837;
    double elbowCountPerDegree = 14.67;
    int extendHomePosn = -40;
    int extendStartPosn = -500;
    int elbowStartAngle = 85;
    int extendHangPosn = -500;  //-800;
    int elbowHangAngle = 20;
    boolean armPosnControl = false;
    ElapsedTime timer = new ElapsedTime();
    IMU imu;

    public AprilTagDetection tagOfInterest = null;



///////////////////////////////////////////////////////////////////////////////////

    @Override
    public void runOpMode() {
        initHardware();
        distTelemetry();
        motor_Gripper.setPosition(gripperClosePosn);
        resetArmExtensions();
        timer.reset();
        while(!isStarted()){
            distTelemetry();
            if(timer.seconds()<1) {
                motor_Gripper.setPosition(gripperOpenPosn);
                armPosnControl = true;
                motor_Extend.setTargetPosition(extendStartPosn);
                motor_Extend.setPower(-0.5);
                motor_Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_Elbow.setTargetPosition((elbowZeroDegreeOffset + (int) (elbowStartAngle * elbowCountPerDegree)));
                motor_Elbow.setPower(0.95);
                motor_Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(timer.seconds()>5){
                motor_Gripper.setPosition(gripperSampleClosePosn);
            }
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == tag11)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if(tag.id == tag12)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if(tag.id == tag13)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if(tag.id == tag14)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if(tag.id == tag15)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if(tag.id == tag16)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                }

            }

            telemetry.update();
            sleep(20);
        }
        waitForStart();
        motor_Gripper.setPosition(gripperSampleClosePosn);
        while(opModeIsActive()){
            if(autonomousStage==0) {
                if (Math.abs(FRDist.getDistance(DistanceUnit.INCH) - 7.5) > distanceTol) {
                    power = PIDControl(7.5, FRDist.getDistance(DistanceUnit.INCH), Kp_posn, Kd_posn, maxPID_Power_posn);
                    drvStraight(-power);
                }
                else {
                    drvStraight(0);
                    autonomousStage = 1;
                }
            }
            if(autonomousStage ==1) {
                armPosnControl = true;
                motor_Extend.setTargetPosition(extendHangPosn);
                motor_Elbow.setTargetPosition(elbowZeroDegreeOffset + (int) (elbowHangAngle * elbowCountPerDegree));
                timer.reset();
                autonomousStage = 2;
            }
            if(autonomousStage ==2) {
                if(timer.seconds()>1){
                    motor_Gripper.setPosition(gripperOpenPosn);
                    armPosnControl = true;
//                    armHangSpecimenPosition(extendHomePosn, elbowZeroDegreeOffset );
                    autonomousStage = 3;
                    timer.reset();
                }
            }
            if(autonomousStage==3){
                if(Math.min(FLDist.getDistance(DistanceUnit.INCH), FRDist.getDistance(DistanceUnit.INCH))<7){
                    drvStraight(-0.3);
                }
                else {
                    drvStraight(0);
                    autonomousStage=4;
                }
            }
            if(autonomousStage==4){
                    if (Math.abs(straightDirection - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > 2) {
                        power = PIDControl(straightDirection, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), Kp, Kd, maxPID_Power);
                        pidDrive(power);
                    } else {
                        pidDrive(0);
                        autonomousStage = 5;
                    }
            }
            if(autonomousStage==5){
                if(Math.abs(LeftDist.getDistance(DistanceUnit.INCH)-12.5)>distanceTol){
                    power = PIDControl(12, LeftDist.getDistance(DistanceUnit.INCH), Kp_strf, Kd_strf, maxPID_Power_strf);
                    strafe(power);
                    timer.reset();
                }
                else {
                    strafe(0);
                    if(timer.seconds()>0){
//                        armHangSpecimenPosition(extendHangPosn, elbowZeroDegreeOffset );
                        autonomousStage = 6;
                    }
                }
            }
            if(autonomousStage==6){
                if(Math.abs(straightDirection-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))>1){
                    power = PIDControl(straightDirection, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), Kp, Kd, maxPID_Power);
                    pidDrive(power);
                }
                else {
                    pidDrive(0);
                    autonomousStage = -1;
                }
            }
            if(autonomousStage==7){
                if(Math.min(FRDist.getDistance(DistanceUnit.INCH), FLDist.getDistance(DistanceUnit.INCH))>13){
                    drvStraight(0.3);
                    timer.reset();
                }
                else if(tagOfInterest.id == tag11) {
                    drvStraight(0);
                    if(timer.seconds()>0){
                        autonomousStage = 6;
                    }
                }
                else {
                    drvStraight(0);
                    if(timer.seconds()>0){
                        autonomousStage = 7;
                    }
                }
            }
            if(autonomousStage==7){
                if(Math.abs(0-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))>1){
                    power = PIDControl(0, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), Kp, Kd, maxPID_Power);
                    pidDrive(power);
                }
                else {
                    pidDrive(0);
                    autonomousStage = 8;
                }
            }
            if(autonomousStage==8){
                if(Math.max(RRDist.getDistance(DistanceUnit.INCH), RLDist.getDistance(DistanceUnit.INCH))<22){
                    drvStraight(0.3);
                    timer.reset();
                }
                else {
                    drvStraight(0);
                    if (timer.seconds() > 0) {
                        autonomousStage = 9;
                    }
                }
            }
            distTelemetry();
        }
    }


///////////////////////////////////////////////////////////////////////////////////


    public void pidDrive(double dr) {
        motor_FLM.setPower(dr);
        motor_FRM.setPower(dr);
        motor_RLM.setPower(dr);
        motor_RRM.setPower(dr);
    }


    public void mecanum_drive(double dr, double st, double tw) {
//        dr*=-1;
        double[] speeds = {
                (dr + st + tw),
                (dr - st - tw),
                (dr - st + tw),
                (dr + st - tw)
        };
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] *= 0.5/max;
        }
        motor_FLM.setPower(speeds[0]);
        motor_FRM.setPower(speeds[1]);
        motor_RLM.setPower(speeds[2]);
        motor_RRM.setPower(speeds[3]);
    }


    public void Twist_IMU(double Heading) {
        if (Heading > imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)){
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < Heading){
                twist = -.4;
                mecanum_drive(0, 0, twist);
            }
        }
        else {
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > Heading){
                twist = .4;
                mecanum_drive(0, 0, twist);
            }
        }
        mecanum_drive(0, 0, 0);
    }

    public double PIDControl(double refrence, double state, double p, double d, double pwr) {
        double error = (refrence - state);
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * p) + (derivative * d);
        output = Math.min(output, pwr);
        output = Math.max(output, -pwr);
        return output;
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

    public void armExtensionsHome(){
        motor_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Riser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(Math.abs(motor_Extend.getCurrentPosition()-extendHomePosn)>armExtensionsTol) {
            motor_Extend.setTargetPosition(extendHomePosn);
            motor_Extend.setPower(0.9);
            motor_Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(Math.abs(motor_Elbow.getCurrentPosition()-armElbowHomePosn)>armExtensionsTol) {
            motor_Elbow.setTargetPosition(10);
            motor_Elbow.setPower(-0.5);
            motor_Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(Math.abs(motor_Riser.getCurrentPosition()-armRiserHomePosn)>armExtensionsTol) {
            motor_Riser.setTargetPosition(00);
            motor_Riser.setPower(-0.35);
            motor_Riser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void armHangSpecimenPosition(int extend, int elbow){
        motor_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if((Math.abs(motor_Extend.getCurrentPosition()-extend)>armExtensionsTol) && armPosnControl) {
            motor_Extend.setTargetPosition(extend);
            motor_Extend.setPower(-0.5);
            motor_Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Extend Motor Request", extend);
            telemetry.update();
        }
        if((Math.abs(motor_Elbow.getCurrentPosition()-elbow)>armExtensionsTol) && armPosnControl) {
            motor_Elbow.setTargetPosition(elbow);
            motor_Elbow.setPower(0.95);
            motor_Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        armPosnControl = false;
    }
    public void testArmExtensions(){
        motor_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Riser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Extend.setTargetPosition(-700);
        motor_Extend.setPower(-0.75);
        motor_Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_Elbow.setTargetPosition(1350);
        motor_Elbow.setPower(0.5);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_Riser.setTargetPosition(300);
        motor_Riser.setPower(0.35);
        motor_Riser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void resetArmExtensions(){
        double startTime = getRuntime();
        motor_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Riser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while((getRuntime()-startTime)<2){
            motor_Extend.setPower(0.5);
            motor_Elbow.setPower(-0.1);
            motor_Riser.setPower(-0.9);
        }
        motor_Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_Riser.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void initHardware(){
        initDistanceSens();
        initDriveMotors();
        initArmExtensions();
        initServo();
        initializeIMU();

        // Initialize camera and pipeline
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
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
        motor_Riser = hardwareMap.dcMotor.get("RiserMotor");
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


    public void distTelemetry(){
        telemetry.addData("Left Distance in Inches", "%.2f", LeftDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Front Left Distance in Inches", "%.2f", FLDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Front Right Distance in Inches", "%.2f", FRDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Rear Left Distance in Inches", "%.2f", RLDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Rear Right Distance in Inches", "%.2f", RRDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Elbow Motor Encoder", motor_Elbow.getCurrentPosition());
        telemetry.addData("Extend Motor Encoder", motor_Extend.getCurrentPosition());
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Autonomous Stage", autonomousStage);
//        telemetry.addData("Game Time", "%.2f", getRuntime());
        telemetry.update();
    }
}
