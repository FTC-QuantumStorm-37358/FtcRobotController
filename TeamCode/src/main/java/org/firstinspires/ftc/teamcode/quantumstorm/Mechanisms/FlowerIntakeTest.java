package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/**
 * Bench test for the flower intake servo alone. Needs only "flower_servo" in the
 * active Robot Configuration, so it runs on a spare Control Hub with no drive motors.
 * Same buttons as QuantumStormTeleOp, so tuned values carry straight over, plus
 * bench-only A = DROP / Y = LIFT and a live gamepad readout for troubleshooting.
 */
@TeleOp(name = "Test: Flower Intake Servo", group = "Quantum Storm Test")
public class FlowerIntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        FlowerIntake flowerIntake = new FlowerIntake(hardwareMap);

        // The arm moves to UP as soon as INIT is pressed
        flowerIntake.lift();

        telemetry.addLine("Flower arm moved to UP. Press START to test.");
        telemetry.addLine("START + D-Pad DOWN = DROP");
        telemetry.addLine("START + D-Pad UP = LIFT");
        telemetry.addLine("LB + D-Pad UP/DOWN = Nudge");
        telemetry.addLine("Bench only: A = DROP, Y = LIFT");
        telemetry.addLine("Gamepad not responding? Press START+A on it to make it User 1");
        telemetry.update();

        waitForStart();

        boolean previousA = gamepad1.a;
        boolean previousY = gamepad1.y;

        while (opModeIsActive()) {

            // Same controls as QuantumStormTeleOp
            flowerIntake.handleGamepad(gamepad1);

            // Bench-only single buttons, to rule out the combos while testing.
            // Ignored with START held, since START+A assigns the gamepad.
            if (!gamepad1.start) {
                if (gamepad1.a && !previousA) flowerIntake.drop();
                if (gamepad1.y && !previousY) flowerIntake.lift();
            }

            previousA = gamepad1.a;
            previousY = gamepad1.y;

            telemetry.addData(
                    "Flower Intake",
                    "%s  %.0f deg  (servo pos %.3f)",
                    flowerIntake.isDown() ? "DOWN" : "UP",
                    flowerIntake.getAngle(),
                    flowerIntake.getPosition());
            telemetry.addData("Limits", "%.0f .. %.0f deg",
                    Constants.FLOWER_ANGLE_MIN_DEG, Constants.FLOWER_ANGLE_MAX_DEG);
            telemetry.addLine("START+D-Pad DOWN/UP = DROP/LIFT,  LB+D-Pad = Nudge");
            telemetry.addLine("Bench only: A = DROP, Y = LIFT");

            // Live gamepad readout: if nothing here changes when buttons are
            // pressed, the gamepad is not assigned as User 1.
            telemetry.addLine("");
            telemetry.addData("Gamepad 1",
                    gamepad1.getGamepadId() == Gamepad.ID_UNASSOCIATED
                            ? "NOT ASSIGNED - press START+A on the gamepad"
                            : "assigned (" + gamepad1.type() + ")");
            telemetry.addData("Buttons",
                    "START %s  LB %s  UP %s  DOWN %s  A %s  Y %s",
                    onOff(gamepad1.start), onOff(gamepad1.left_bumper),
                    onOff(gamepad1.dpad_up), onOff(gamepad1.dpad_down),
                    onOff(gamepad1.a), onOff(gamepad1.y));
            telemetry.update();
        }
    }

    private static String onOff(boolean pressed) {
        return pressed ? "ON" : "-";
    }
}
