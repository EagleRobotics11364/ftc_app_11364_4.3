package org.firstinspires.ftc.teamcode.library.sampling

import com.qualcomm.robotcore.hardware.HardwareMap

import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.Predicate
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector
import org.firstinspires.ftc.teamcode.library.functions.Position

import java.util.ArrayList
import java.util.Arrays
import java.util.Comparator
private const val VUFORIA_KEY = "AcMSLB//////AAAAGV2X9BmFFk6Pt9dw+Dg7oCSDbgmpvFL2uaQFUQNenTRFP8eywDy/1JH+6MeeMp/aHH3L2pWVW+t2hx9saq2n72eE+/6orS0hL6ooUobxBlvKS6YQqJIQM7ZOTOIVVpgpzVODNQVdcvRW6Vm2yGrRUAPnuEScnQU9ahY8PSApozJ05M8oS33fEP8T76Y8V31jWRqaw1JIsXQRKHzmQpK5l1no4LwBQ/iCxmHHJ3h77zlfKDsP9DQrh0r/r9b8dP7sSMtCQsukfrmwD4o5uF+S6e4ScWTA4tgpXkPMYVfyjVLsynvNHhi2kuzd2goDeP1uNgpSoEXzJQQKcNeo99nKm3BU22USUBPliFrocMRYGnxb"

private const val TFOD_MODEL_ASSET = "RoverRuckus.tflite"
private const val LABEL_GOLD_MINERAL = "Gold Mineral"
private const val LABEL_SILVER_MINERAL = "Silver Mineral"

class TFODFilteringSamplerKotlin @Throws(TensorFlowSampler.UnsupportedHardwareException::class) constructor(val hardwareMap: HardwareMap): TensorFlowSampler {
    lateinit var tfod : TFObjectDetector

    init {
        if (!ClassFactory.getInstance().canCreateTFObjectDetector()) throw TensorFlowSampler.UnsupportedHardwareException()
        tfod = ClassFactory.getInstance().createTFObjectDetector(
                TFObjectDetector.Parameters(
                        hardwareMap.appContext.resources.getIdentifier(
                                "tfodMonitorViewId", "id", hardwareMap.appContext.packageName)
                ),
                ClassFactory.getInstance().createVuforia(VuforiaLocalizer.Parameters().apply {
                    vuforiaLicenseKey = VUFORIA_KEY
                    cameraName = hardwareMap.get(WebcamName::class.java, "Webcam 1")
                })
        ).apply { loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL) }
    }

    override fun recognizeOneMineral(): FieldSample {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun recognizeGoldUsingTwoMinerals(cameraViewingDirection: Position?): Position {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun recognizeGoldUsingThreeMinerals(): Position {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun activate(): Boolean {
        tfod.activate()
        return true
    }

    override fun deactivate(): Boolean {
        tfod.deactivate()
        return true
    }

    override fun getOutput(): String {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    companion object {

    }
}


