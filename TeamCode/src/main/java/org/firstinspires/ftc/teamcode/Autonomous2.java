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


@Autonomous(name="Auton2")
public class Autonomous2 extends LinearOpMode {

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

    double Kp = 0.04;
    double Kd = 0.0004;
    double maxPID_Power = 0.75;
    double Kp_posn = 0.04;
    double Kd_posn = 0.004;
    double maxPID_Power_posn = 0.5;
    double Kp_strf = 0.008;
    double Kd_strf = 0.00008;
    double maxPID_Power_strf = 0.9;
    double lastError = 0;
    int hangDist = 7;
    int dropDist = 2;
    int rearDistTarget = 23;
    double rearDist = 0;

    double distanceTol = 1;
    double power=0;
    double twist = 0;
    double gripperOpenPosn = 0.99;
    double gripperSampleClosePosn = 0;
    double gripperClosePosn = 0;
    int autonomousStage = 0;
    int armExtensionsTol = 10;
    int armElbowHomePosn = 10;
    int armRiserHomePosn = 0;
    int straightDirection = 0;
    int sampleDropDirection = 145;

    int elbowZeroDegreeOffset = 837;
    int elbowSamplePickAngle = -20;
    double elbowCountPerDegree = 14.67;
    int extendHomePosn = -40;
    int extendStartPosn = -650;
    int elbowStartAngle = 85;
    int elbowDropAngle = 75;
    int extendHangPosn = -450;  //-800;
    int elbowHangAngle = 0;
    boolean armPosnControl = false;
    ElapsedTime timer = new ElapsedTime();
    IMU imu;





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
                motor_Extend.setPower(-0.3);
                motor_Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_Elbow.setTargetPosition((elbowZeroDegreeOffset + (int) (elbowStartAngle * elbowCountPerDegree)));
                motor_Elbow.setPower(0.9);
                motor_Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(timer.seconds()>5){
                motor_Gripper.setPosition(gripperSampleClosePosn);
            }
        }
        waitForStart();
        motor_Gripper.setPosition(gripperSampleClosePosn);
        while(opModeIsActive()){
            if(autonomousStage==0) {
                if (Math.abs(FRDist.getDistance(DistanceUnit.INCH) - hangDist) > distanceTol) {
                    power = PIDControl(hangDist, FRDist.getDistance(DistanceUnit.INCH), Kp_posn, Kd_posn, maxPID_Power_posn);
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
                if(timer.seconds()>1&& timer.seconds()<2){
                    motor_Gripper.setPosition(gripperOpenPosn);
                    motor_Extend.setTargetPosition(extendHomePosn);
                }
                if(timer.seconds()>2 && (motor_Extend.getCurrentPosition()>-200)){
                    autonomousStage = 3;
                    timer.reset();
                }
            }
            if(autonomousStage==3){
                if(Math.min(FLDist.getDistance(DistanceUnit.INCH), FRDist.getDistance(DistanceUnit.INCH))<6){
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
                    motor_FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor_FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor_RLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor_RRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor_FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motor_FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motor_RLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motor_RRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motor_Elbow.setTargetPosition(elbowZeroDegreeOffset + (int) (elbowSamplePickAngle * elbowCountPerDegree));
                    motor_Elbow.setPower(0.2);
                    motor_Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    autonomousStage = 5;
                }
            }
            if(autonomousStage==5){
                if(Math.abs(Math.abs(motor_FLM.getCurrentPosition()) - 2000) > 20){
                    power = PIDControl(2000, Math.abs(motor_FLM.getCurrentPosition()), Kp_strf, Kd_strf, maxPID_Power_strf);
                    strafe(-power);
                    timer.reset();
                }
                else {
                    strafe(0);
                    if(timer.seconds()>0){
                        motor_Extend.setTargetPosition(-100);
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
                    autonomousStage = 7;
                }
            }
            if(autonomousStage==7){
                if(Math.abs(LeftDist.getDistance(DistanceUnit.INCH)-13)>distanceTol){
                    power = PIDControl(13, LeftDist.getDistance(DistanceUnit.INCH), 0.15, 0.0001, 0.3);
                    strafe(power);
                    timer.reset();
                }
                else {
                    strafe(0);
                    if(timer.seconds()>0){
                        autonomousStage = 8;
                    }
                }
            }
            if(autonomousStage==8){
                if(Math.abs(straightDirection-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))>1){
                    power = PIDControl(straightDirection, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), Kp, Kd, maxPID_Power);
                    pidDrive(power);
                }
                else {
                    pidDrive(0);
                    rearDist = Math.max(RLDist.getDistance(DistanceUnit.INCH), RRDist.getDistance(DistanceUnit.INCH));
                    if (Math.abs(rearDist - rearDistTarget) > distanceTol) {
                        power = PIDControl(rearDistTarget, rearDist, Kp_posn, Kd_posn, maxPID_Power_posn);
                        drvStraight(power);
                    }
                    if(Math.abs(rearDist - rearDistTarget) < distanceTol) {
                        autonomousStage = 9;
                    }
                }
            }
            if(autonomousStage==9){
                motor_Extend.setTargetPosition(-400);
                    motor_Gripper.setPosition(gripperSampleClosePosn);
                    timer.reset();
                    autonomousStage = 10;
            }
            if(autonomousStage==10){
                if(timer.seconds()>1) {
                    motor_Elbow.setTargetPosition(elbowZeroDegreeOffset + (int) (elbowDropAngle * elbowCountPerDegree));
                    motor_Elbow.setPower(0.9);
                    motor_Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor_Extend.setTargetPosition(-1300);
                    autonomousStage = 11;
                }
            }
            if (autonomousStage == 11){
                if(Math.abs(sampleDropDirection-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))>1){
                    power = PIDControl(sampleDropDirection, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), Kp, Kd, maxPID_Power);
                    pidDrive(power);
                }
                else {
                    pidDrive(0);
                    timer.reset();
                    autonomousStage = 12;
                }
            }
            if(autonomousStage ==12){
                if ( timer.seconds()<1) {
                    power = PIDControl(dropDist, FLDist.getDistance(DistanceUnit.INCH), Kp_posn, Kd_posn, maxPID_Power_posn);
                    drvStraight(-power);
                }
                else {
                    drvStraight(0);
                    autonomousStage = 13;
                }
            }
            if(autonomousStage == 13){
                if(Math.abs(sampleDropDirection-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))>1){
                    power = PIDControl(sampleDropDirection, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), Kp, Kd, maxPID_Power);
                    pidDrive(power);
                }
                else {
                    pidDrive(0);
                    motor_Gripper.setPosition(gripperOpenPosn);
                    timer.reset();
                    autonomousStage = 14;
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
        telemetry.addData("Motor FLM", motor_FLM.getCurrentPosition());
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Autonomous Stage", autonomousStage);
//        telemetry.addData("Game Time", "%.2f", getRuntime());
        telemetry.update();
    }
}
