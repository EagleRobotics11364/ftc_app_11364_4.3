package org.firstinspires.ftc.teamcode.library.sampling;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Predicate;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.library.functions.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class VuforiaTFODSampler {
    private static final String VUFORIA_KEY = "AcMSLB//////AAAAGV2X9BmFFk6Pt9dw+Dg7oCSDbgmpvFL2uaQFUQNenTRFP8eywDy/1JH+6MeeMp/aHH3L2pWVW+t2hx9saq2n72eE+/6orS0hL6ooUobxBlvKS6YQqJIQM7ZOTOIVVpgpzVODNQVdcvRW6Vm2yGrRUAPnuEScnQU9ahY8PSApozJ05M8oS33fEP8T76Y8V31jWRqaw1JIsXQRKHzmQpK5l1no4LwBQ/iCxmHHJ3h77zlfKDsP9DQrh0r/r9b8dP7sSMtCQsukfrmwD4o5uF+S6e4ScWTA4tgpXkPMYVfyjVLsynvNHhi2kuzd2goDeP1uNgpSoEXzJQQKcNeo99nKm3BU22USUBPliFrocMRYGnxb";
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

    /**
     * Activates the TensorFlow object detector and turns on the phone LED.
     * @throws NullPointerException if the TensorFlow object detector was not initialized
     */
    public void activate() {
        if (tfod != null) {
            tfod.activate();
            CameraDevice.getInstance().setFlashTorchMode(false);
        }
        else throw new NullPointerException();
    }

    /**
     * Shuts down the TensorFlow object detector to free up system resources.
     * @throws NullPointerException if the TensorFlow object detector was not initialized
     */
    public void shutdown() {
        if (tfod != null) {
            CameraDevice.getInstance().setFlashTorchMode(false);
            tfod.shutdown();
        }
        else throw new NullPointerException();
    }

    /**
     * Identifies the position of the gold sample on the Field.
     * Assumes that the robot is viewing all three samples.
     *
     * @return The position of the gold sample on the Field.
    */
    public Position recognize() {
        return recognize(Position.CENTER);
    }

    /**
     * Identifies the position of the gold sample on the Field.
     * @param cameraViewingDirection Angle at which the phone camera views the field samples.
     *                               Position.LEFT   -> Camera views Left and Center samples
     *                               Position.CENTER -> Camera views all three samples.
     *                               Position.RIGHT  -> Camera views Center and Right samples.
     * @return The position of the gold sample on the Field.
     * @throws NullPointerException if the TensorFlow object detector was not initialized.
     */
    public Position recognize(Position cameraViewingDirection) {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (telemetry != null)
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                List<Recognition> finalRecognitions = new ArrayList<>();
                if (cameraViewingDirection == Position.LEFT || cameraViewingDirection == Position.RIGHT) {
                    if (updatedRecognitions.size() >= 2) {
                        if (updatedRecognitions.size() > 2) {
                            try{
                                finalRecognitions.addAll(getSpecifiedRecognitionsOfLargestWidth(updatedRecognitions, 2));
                            } catch (NotEnoughRecognitionsException e) { return Position.NULL; }
                        } else finalRecognitions = new ArrayList<>(updatedRecognitions);
                        Recognition mineral1 = finalRecognitions.get(0);
                        Recognition mineral2 = finalRecognitions.get(1);
                        if (cameraViewingDirection == Position.LEFT) {
                            if (mineral1.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                if (mineral1.getLeft() < mineral2.getLeft()) return Position.LEFT;
                                else return Position.CENTER;
                            } else if (mineral2.getLabel().equals(LABEL_GOLD_MINERAL)){
                                if (mineral2.getLeft() < mineral1.getLeft()) return Position.LEFT;
                                else return Position.CENTER;
                            } else return Position.RIGHT;
                        } else { // cameraViewingDirection == Position.RIGHT
                            if (mineral1.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                if (mineral1.getLeft() < mineral2.getLeft()) return Position.CENTER;
                                else return Position.RIGHT;
                            } else  if (mineral2.getLabel().equals(LABEL_GOLD_MINERAL)){
                                if (mineral2.getLeft() < mineral1.getLeft()) return Position.CENTER;
                                else return Position.RIGHT;
                            } else return Position.LEFT;
                        }

                    } else return Position.NULL;
                } else if (cameraViewingDirection == Position.CENTER) {
                    if (updatedRecognitions.size() >= 3) {
                        if (updatedRecognitions.size() > 3) {
                            try{
                                finalRecognitions.addAll(getSpecifiedRecognitionsOfLargestWidth(updatedRecognitions, 1, FieldSample.GOLD));
                                finalRecognitions.addAll(getSpecifiedRecognitionsOfLargestWidth(updatedRecognitions, 2, FieldSample.SILVER));
                            } catch (NotEnoughRecognitionsException e) { return Position.NULL; }
                        } else finalRecognitions = new ArrayList<>(updatedRecognitions);
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : finalRecognitions) {
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
                                if (telemetry != null)
                                    telemetry.addData("Gold Mineral Position", "Left");
                                return Position.LEFT;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                if (telemetry != null)
                                    telemetry.addData("Gold Mineral Position", "Right");
                                return Position.RIGHT;
                            } else {
                                if (telemetry != null)
                                    telemetry.addData("Gold Mineral Position", "Center");
                                return Position.CENTER;
                            }
                        }
                    } else {
                        return Position.NULL;
                    }

                }

                if (telemetry != null) telemetry.update();
            }
        } else throw new NullPointerException();
        return Position.NULL;
    }

    /**
     *
     * @param recognitions The list containing the original items identified by the TensorFlow interpreter.
     * @param numberToGet The number of field sample Recognitions to return as a new List object.
     * @return List object containing field sample recognitions of largest width.
     */
    private List<Recognition> getSpecifiedRecognitionsOfLargestWidth(List<Recognition> recognitions, int numberToGet) throws NotEnoughRecognitionsException{
        return getSpecifiedRecognitionsOfLargestWidth(recognitions, numberToGet, FieldSample.NULL);
    }

    /**
     *
     * @param recognitions The list containing the original items identified by the TensorFlow interpreter.
     * @param numberToGet The number of field sample Recognitions to return as a new List object.
     * @param color Filter to only return field samples of a specific color.
     * @return List object containing field sample recognitions of largest width.
     */
    private List<Recognition> getSpecifiedRecognitionsOfLargestWidth(List<Recognition> recognitions, int numberToGet, FieldSample color) throws NotEnoughRecognitionsException {
        // create list to be sorted (sortedRecognitions), and a second list to hold only the specified number of elements (finalRecognitions)
        List<Recognition> sortedRecognitions = new ArrayList<>(recognitions);
        List<Recognition> finalRecognitions = new ArrayList<>();

        // remove elements in sortedRecognitions that are not of the specified color
        for (Recognition recognition : sortedRecognitions) {
            switch (color) {
                case GOLD:
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                        finalRecognitions.add(recognition);
                    break;
                case SILVER:
                    if (recognition.getLabel().equals(LABEL_SILVER_MINERAL))
                        finalRecognitions.add(recognition);
                    break;
                default:
                    finalRecognitions.add(recognition);
            }
        }
        sortedRecognitions = new ArrayList<>(finalRecognitions);
        // sort the sortedRecognitions by calling Collections.sort() and creating an anonymous instantiation of Comparator
        Collections.sort(sortedRecognitions,
                new Comparator<Recognition>() {
                    @Override
                    public int compare(Recognition recognition, Recognition t1) {
                        if (recognition.getWidth() < t1.getWidth()) return -1;
                        else if (recognition.getWidth() == t1.getWidth()) return 0;
                        else return 1;
                    }
                });

        finalRecognitions = new ArrayList<>();
        // add largest sorted items to the finalRecognitions list
        if (sortedRecognitions.size() < numberToGet) throw new NotEnoughRecognitionsException(sortedRecognitions.size(), numberToGet);
        else if (sortedRecognitions.size() == numberToGet) return sortedRecognitions;
        else {
            finalRecognitions = new ArrayList<>();
            for (int i = sortedRecognitions.size(); i > sortedRecognitions.size() - numberToGet; i--) {
                finalRecognitions.add(sortedRecognitions.get(i - 1));
            }
        }
        return finalRecognitions;
    }

    private List<Recognition> filter(List<Recognition> recognitions, Predicate<Recognition> filterClause) {
        List<Recognition> filteredRecognitions = new ArrayList<>();
        for (Recognition recognition : recognitions) {
            if (filterClause.test(recognition)) filteredRecognitions.add(recognition);
        }
        return filteredRecognitions;
    }

    public SamplerResult recognizeUsingGoldLocation_viewRightPosition() {
        Position goldSamplePosition = Position.NULL;
        double finalConfidenceMeasure = 0.0;
        List<Recognition> recognitions = tfod.getUpdatedRecognitions();
        List<Recognition> filteredRecognitions = filter(recognitions, new Predicate<Recognition>() {
            @Override
            public boolean test(Recognition recognition) {
                return recognition.getHeight() > 80 &
                        recognition.getWidth() > 80;
            }
        });
        telemetry.addData("Filtered recognitions", filteredRecognitions.size());
        try {
            List<Recognition> goldRecognitionList = getSpecifiedRecognitionsOfLargestWidth(filteredRecognitions, 1, FieldSample.GOLD);
            Recognition goldRecognition = goldRecognitionList.get(0);
            telemetry.addLine("Found single gold recognition");
            if (goldRecognition.getLeft() < 200) goldSamplePosition = Position.CENTER;
            else goldSamplePosition = Position.RIGHT;
            telemetry.addData("Gold recognition position", goldSamplePosition);
            finalConfidenceMeasure = goldRecognition.getConfidence();
        } catch (NotEnoughRecognitionsException e) {
            telemetry.addLine("Threw notEnoughRecognitionsException");
            telemetry.addLine("Gold sample = left position");
            goldSamplePosition = Position.LEFT;
            try {
                List<Recognition> silverRecognitionList = getSpecifiedRecognitionsOfLargestWidth(filteredRecognitions, 2, FieldSample.SILVER);
                finalConfidenceMeasure = (silverRecognitionList.get(0).getConfidence() + silverRecognitionList.get(1).getConfidence()) / 2;
                telemetry.addLine("Found two silver recognitions");
            } catch (NotEnoughRecognitionsException ee) {
                telemetry.addLine("Did not find two silver recognitions");
                try {
                    List<Recognition> silverRecognitionList = getSpecifiedRecognitionsOfLargestWidth(filteredRecognitions, 1, FieldSample.SILVER);
                    finalConfidenceMeasure = Math.pow(silverRecognitionList.get(0).getConfidence(),2);
                    telemetry.addLine("Found one silver recognition");
                } catch (NotEnoughRecognitionsException eee) {
                    goldSamplePosition = Position.NULL;
                    telemetry.addLine("Did not find any recognitions");
                    telemetry.addLine("Will return null");
                }
            }

        }
        telemetry.addData("return confidence", finalConfidenceMeasure);
        telemetry.addData("return position", goldSamplePosition);
        return new SamplerResult(goldSamplePosition,finalConfidenceMeasure);
    }

    public class VuforiaTFODException extends Exception {
        public VuforiaTFODException() {
            super();
        }
        public VuforiaTFODException(String detailMessage) {
            super(detailMessage);
        }
    }

    public class UnsupportedHardwareException extends VuforiaTFODException { }

    private class NotEnoughRecognitionsException extends VuforiaTFODException {
        public NotEnoughRecognitionsException(int listSize, int neededRecognitionQuantity) {
            super("List contains "+listSize+ " recognitions; needed "+neededRecognitionQuantity);
        }
    }

    public class SamplerResult {
        public Position position;
        public double confidence;

        public SamplerResult(Position position, double confidence) {
            this.position = position;
            this.confidence = confidence;
        }
    }

}
