package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Control Hub
 * Motor Port 0 FLMotor
 * Motor Port 1 RLMotor
 * Motor Port 2 RRMotor
 * Motor Port 3 FRMotor
 *
 * I2C Port 0
 * I2C Port 1
 * I2C Port 2 FRDistance
 * I2C Port 3 RLDistance
 *
 * Extension Hub Motor 0 Extend Motor
 * Extension Hub Motor 1 Elbow Motor
 * Extension Hub Motor 2 Riser Motor
 */


@Autonomous(name="TestSens")
public class Autonomous1 extends LinearOpMode {

    DcMotor motor_FLM = null;
    DcMotor motor_RLM = null;
    DcMotor motor_FRM = null;
    DcMotor motor_RRM = null;

    DcMotor motor_Extend = null;
    DcMotor motor_Elbow = null;
    DcMotor motor_Riser = null;

    DistanceSensor FRDist = null;
    DistanceSensor FLDist = null;
    DistanceSensor RLDist = null;
    DistanceSensor RRDist = null;

    double Kp = 0.04;
    double Kd = 0.0004;
    double lastError = 0;
    double power=0;
    double twist = 0;
    double maxPID_Power = 0.75;
    int autonomousStage = 0;
    int armExtensionsTol = 10;
    int armExtendHangPosn = -800;
    int armElbowHangPosn = 1750;
    int armExtendHomePosn = -10;
    int armElbowHomePosn = 10;
    int armRiserHomePosn = 0;
    int yellowSampleDirection = 135;
    ElapsedTime timer = new ElapsedTime();
    IMU imu;





///////////////////////////////////////////////////////////////////////////////////

    @Override
    public void runOpMode() {
        initHardware();
        while(!isStarted()){
            resetArmExtensions();
        }
        waitForStart();
        armHangSpecimenPosition();
        while(opModeIsActive()){
            if(autonomousStage==0) {
                if ((FRDist.getDistance(DistanceUnit.INCH) > 7)) {
                    drvStraight(0.4);
//                    timer.reset();
                }
                else {
                    drvStraight(0);
                    autonomousStage = 1;
                    timer.reset();
                }
            }
            if(autonomousStage ==1) {
                if (timer.seconds() > 1) {
                    armExtensionsHome();
                    autonomousStage = 2;
                }
            }
            if(autonomousStage==2){
                    if(Math.abs(yellowSampleDirection-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))>4){
                        power = PIDControl(yellowSampleDirection, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                        PID_Drive(power);
                    }
                    else {
                        PID_Drive(0);
                        autonomousStage = 3;
                    }
            }
            if(autonomousStage==3){
                if(Math.min(FRDist.getDistance(DistanceUnit.INCH), FLDist.getDistance(DistanceUnit.INCH))>7){
                    drvStraight(0.35);
                    timer.reset();
                }
                else {
                    drvStraight(0);
                    if(timer.seconds()>0){
                        autonomousStage = 4;
                    }
                }
            }
            if(autonomousStage==4){
                if(Math.abs(90-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))>2){
                    power = PIDControl(90, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                    PID_Drive(power);
                }
                else {
                    PID_Drive(0);
                    autonomousStage = 5;
                }
            }
            if(autonomousStage==5){
                if(Math.min(FRDist.getDistance(DistanceUnit.INCH), FLDist.getDistance(DistanceUnit.INCH))>5){
                    drvStraight(0.35);
                    timer.reset();
                }
                else {
                    drvStraight(0);
                    if(timer.seconds()>0){
                        autonomousStage = 6;
                    }
                }
            }
            if(autonomousStage==6){
                if(Math.abs(0-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))>1){
                    power = PIDControl(0, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                    PID_Drive(power);
                }
                else {
                    PID_Drive(0);
                    autonomousStage = 7;
                }
            }
            distTelemetry();
        }
    }


///////////////////////////////////////////////////////////////////////////////////



    public void PID_Drive(double drive){
        motor_FLM.setPower(drive);
        motor_FRM.setPower(drive);
        motor_RLM.setPower(drive);
        motor_RRM.setPower(drive);
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


    public void testDrvMotors() {
        motor_FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_RLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_RRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_FLM.setTargetPosition(-100);
        motor_FLM.setPower(-0.1);
        motor_FLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_RLM.setTargetPosition(-100);
        motor_RLM.setPower(-0.1);
        motor_RLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_FRM.setTargetPosition(100);
        motor_FRM.setPower(0.1);
        motor_FRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_RRM.setTargetPosition(100);
        motor_RRM.setPower(0.1);
        motor_RRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void drvStraight(double pwr){
        motor_FLM.setPower(-pwr);
        motor_FRM.setPower(pwr);
        motor_RLM.setPower(-pwr);
        motor_RRM.setPower(pwr);
    }


    public void armExtensionsHome(){
        motor_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Riser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(Math.abs(motor_Extend.getCurrentPosition()-armExtendHomePosn)>armExtensionsTol) {
            motor_Extend.setTargetPosition(-10);
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
    public void armHangSpecimenPosition(){
        motor_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Riser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(Math.abs(motor_Extend.getCurrentPosition()-armExtendHangPosn)>armExtensionsTol) {
            motor_Extend.setTargetPosition(-800);
            motor_Extend.setPower(-0.25);
            motor_Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(Math.abs(motor_Elbow.getCurrentPosition()-armElbowHangPosn)>armExtensionsTol) {
            motor_Elbow.setTargetPosition(1750);
            motor_Elbow.setPower(0.75);
            motor_Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
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
        while((getRuntime()-startTime)<1){
            motor_Extend.setPower(0.9);
            motor_Elbow.setPower(-0.9);
            motor_Riser.setPower(-0.9);
        }
        motor_Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_Riser.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void initHardware(){
        initDistanceSens();
        initDriveMotors();
        initArmExtensions();
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
        motor_Riser = hardwareMap.dcMotor.get("RiserMotor");
    }
    public void initDistanceSens(){
        FRDist = hardwareMap.get(DistanceSensor.class, "FRDistance");
        FLDist = hardwareMap.get(DistanceSensor.class, "FLDistance");
        RRDist = hardwareMap.get(DistanceSensor.class, "RRDistance");
        RLDist = hardwareMap.get(DistanceSensor.class, "RLDistance");
    }
    public void initializeIMU(){
        imu=hardwareMap.get(IMU.class, "IMU");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }


    public void distTelemetry(){
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
