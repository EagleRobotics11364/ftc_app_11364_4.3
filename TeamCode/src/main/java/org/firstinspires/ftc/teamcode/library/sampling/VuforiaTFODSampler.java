package org.firstinspires.ftc.teamcode.library.sampling;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.library.functions.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class VuforiaTFODSampler {
    private static final String VUFORIA_KEY = "";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private Telemetry telemetry;

    public VuforiaTFODSampler(HardwareMap hardwareMap) throws UnsupportedHardwareException {
        // check if tfod is supported
        if (!ClassFactory.getInstance().canCreateTFObjectDetector())
            throw new UnsupportedHardwareException();

        // init vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // init tfod
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public VuforiaTFODSampler(HardwareMap hardwareMap, Telemetry telemetry) throws UnsupportedHardwareException {
        this(hardwareMap);
        this.telemetry = telemetry;
    }

    public void activate() {
        if(tfod != null) tfod.activate();
        else throw new NullPointerException();
    }

    public void shutdown() {
        if(tfod != null) tfod.shutdown();
        else throw new NullPointerException();
    }

    public Position recognize() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (telemetry != null) telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            if (telemetry != null) telemetry.addData("Gold Mineral Position", "Left");
                            return Position.LEFT;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            if (telemetry != null) telemetry.addData("Gold Mineral Position", "Right");
                            return Position.RIGHT;
                        } else {
                            if (telemetry != null) telemetry.addData("Gold Mineral Position", "Center");
                            return Position.CENTER;
                        }
                    }
                }
                if (telemetry != null) telemetry.update();
            }
        } else throw new NullPointerException();
        return Position.NULL;
    }


    public class UnsupportedHardwareException extends Exception {}
}
