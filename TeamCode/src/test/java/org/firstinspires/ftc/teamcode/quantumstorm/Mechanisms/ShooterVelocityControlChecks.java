package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import java.lang.reflect.Proxy;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Verify the default TeleOp shooter uses calibrated velocity commands, including INIT/STOP. */
public final class ShooterVelocityControlChecks {
    private static void check(boolean condition,String why) {
        if(!condition)throw new AssertionError(why);
    }

    public static void main(String[] args) {
        double[] output={Double.NaN};
        int[] powerCalls={0},velocityCalls={0};
        DcMotor.RunMode[] mode={DcMotor.RunMode.RUN_WITHOUT_ENCODER};
        MotorConfigurationType originalType=new MotorConfigurationType();
        originalType.setTicksPerRev(2786);
        originalType.setGearing(99.5);
        originalType.setMaxRPM(60);
        MotorConfigurationType[] motorType={originalType};
        DcMotorEx motor=(DcMotorEx)Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class},(proxy,method,values)->{
                    switch(method.getName()) {
                        case "getMotorType": return motorType[0];
                        case "setMotorType": motorType[0]=(MotorConfigurationType)values[0];return null;
                        case "setMode": mode[0]=(DcMotor.RunMode)values[0];break;
                        case "setPower":
                            powerCalls[0]++;output[0]=(double)values[0];break;
                        case "setVelocity":
                            check(mode[0]==DcMotor.RunMode.RUN_USING_ENCODER,"Velocity uses encoder mode");
                            check(values.length==2 && values[1]==AngleUnit.DEGREES,"Velocity uses angular units");
                            velocityCalls[0]++;output[0]=(double)values[0];break;
                        default: throw new AssertionError("Unexpected motor call: "+method.getName());
                    }
                    return null;
                });
        // Reinitialization must reset output and retain the corrected encoder scale.
        for(int run=0;run<2;run++) {
            ShooterVelocityControl.configure(motor);
            check(output[0]==0,"INIT clears the previous OpMode output");
            check(motorType[0]!=originalType && originalType.getTicksPerRev()==2786
                    && originalType.getGearing()==99.5 && originalType.getMaxRPM()==60,
                    "Shooter calibration must not change other motors' shared profile");
            check(motorType[0].getTicksPerRev()==28 && motorType[0].getGearing()==1
                    && motorType[0].getMaxRPM()==6000,"Apply the confirmed 1:1 motor specifications");
            ShooterVelocityControl.setOutput(motor,.10);
            check(Math.abs(output[0]/360*motorType[0].getTicksPerRev()-280)<1e-6,
                    "600 RPM requests 280 ticks/sec, not 27860");
            double actualRpm=280/motorType[0].getTicksPerRev()*60;
            check(actualRpm==600,"280 measured ticks/sec displays 600 RPM");
            ShooterVelocityControl.setOutput(motor,Constants.SHOOTER_RPM_MULTIPLIER);
            check(Math.abs(output[0]-17640)<1e-6,"Default is 2940 RPM");
            ShooterVelocityControl.setOutput(motor,Constants.SHOOTER_RPM_MULTIPLIER+Constants.SHOOTER_RPM_MULTIPLIER_INCREMENT);
            check(Math.abs(output[0]-18000)<1e-6,"One step is 60 RPM");
            check(Constants.SHOOTER_BASE_RPM==DistanceShooterCalibration.MAX_RPM,
                    "Physics RPM limit matches the motor velocity scale");
            for (int xs : new int[]{-1,1}) for (int ys : new int[]{-1,1}) {
                DistanceShooterCalibration.Solution shot=DistanceShooterCalibration.atPosition(xs*36,ys*62.9);
                ShooterVelocityControl.setOutput(motor,shot.rpm/Constants.SHOOTER_BASE_RPM);
                check(shot.ready() && Math.abs(output[0]-18355.2040779)<1e-5,
                        "Quadrant C prediction reaches the motor in degrees/second without rounding");
            }
            ShooterVelocityControl.setOutput(motor,DistanceShooterCalibration.atPosition(0,0).rpm
                    /Constants.SHOOTER_BASE_RPM);
            check(output[0]==0,"Impossible distance becomes zero velocity");
            ShooterVelocityControl.setOutput(motor,2);
            check(output[0]==36000,"Positive output is bounded");
            ShooterVelocityControl.setOutput(motor,-2);
            check(output[0]==-36000,"Negative output is bounded");
            for(double invalid:new double[]{Double.NaN,Double.POSITIVE_INFINITY,Double.NEGATIVE_INFINITY}) {
                ShooterVelocityControl.setOutput(motor,invalid);
                check(output[0]==0,"Invalid target stops the motor");
            }
            ShooterVelocityControl.setOutput(motor,Constants.SHOOTER_RPM_MULTIPLIER);
            ShooterVelocityControl.setOutput(motor,0);
            check(output[0]==0,"STOP clears the active output");
            check(powerCalls[0]==0 && velocityCalls[0]>0,
                    "Default TeleOp shooter uses only velocity commands, including INIT and STOP");
        }
        System.out.println("Default TeleOp velocity API, motor scale, defaults, steps, bounds and STOP checks passed.");
    }
}
