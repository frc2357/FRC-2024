package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.INTAKE_PHOTON_CAMERA;;

public class IntakePhotonCamera extends PhotonVisionCamera{

    public IntakePhotonCamera(String cameraName, Transform3d robotToCameraTransform, double headOnTolerance) {
        super(cameraName, robotToCameraTransform, headOnTolerance);
    }

    public void setAprilTagPipelineActive(){
        super.setPipeline(INTAKE_PHOTON_CAMERA.APRIL_TAG_PIPELINE);
    }

    public void setNeuralNetworkPipelineActive(){
        super.setPipeline(INTAKE_PHOTON_CAMERA.NEURAL_NETWORK_PIPELINE);
    }

    public void setPoseEstimationPipelineActive(){
        super.setPipeline(INTAKE_PHOTON_CAMERA.POSE_ESTIMATION_PIPELINE);
    }

    public void setDefaultPipelineActive(){
        super.setPipeline(INTAKE_PHOTON_CAMERA.DEFAULT_PIPELINE);
    }
    
}
