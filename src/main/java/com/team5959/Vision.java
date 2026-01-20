package com.team5959;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class Vision {
    private final PhotonCamera leftFrontCamera;
    private final PhotonCamera rightFrontCamera;

    private final PhotonPoseEstimator leftPoseEstimator;
    private final PhotonPoseEstimator rightPoseEstimator;
    private AprilTagFieldLayout aprilTagFieldLayout;

    // --- MULTITHREADING ---
    // Ejecuta la visión en un hilo separado para no alentar el Swerve
    private final Notifier visionThread;
    private final AtomicReference<List<EstimatedRobotPose>> latestEstimates = new AtomicReference<>(new ArrayList<>());
    private final AtomicReference<Pose2d> robotPoseReference = new AtomicReference<>(new Pose2d());

    // Simulación
    private VisionSystemSim visionSim;
    private PhotonCameraSim leftCameraSim;
    private PhotonCameraSim rightCameraSim;

    // Constantes de Confianza
    private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.1,0.1,0.1);//(1, 1, 4); //FIXME despues de las pruebas
    private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.01,0.01,0.01);//(0.1, 0.1, 1);

    // Filtros de Seguridad
    private double fieldLength = 50; //17.55
    private double fieldWidth = 50;//8.05
    private static final double MAX_HEIGHT_ERROR_METERS = 50; //FIXME 0.5m
    private static final double MAX_AMBIGUITY_ALLOWED = 0.5; // Filtro Anti-Jitter //0.2

    private static final Transform3d LEFT_FRONT_ROBOT_TO_CAM = new Transform3d(
            new Translation3d(0.3, 0.3, 0.2), 
            new Rotation3d(0, Units.degreesToRadians(22), Units.degreesToRadians(45)));

    private static final Transform3d RIGHT_FRONT_ROBOT_TO_CAM = new Transform3d(
            new Translation3d(0.3, -0.3, 0.2), 
            new Rotation3d(0, Units.degreesToRadians(22), Units.degreesToRadians(-45)));

    public Vision() {
        leftFrontCamera = new PhotonCamera("LeftAprilTagCamera"); // Asegúrate que coincida con PhotonVision UI
        rightFrontCamera = new PhotonCamera("RightAprilTagCamera");

        try {
            // Carga campo 2026 Oficial
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
            fieldLength = aprilTagFieldLayout.getFieldLength();
            fieldWidth = aprilTagFieldLayout.getFieldWidth();
        } catch (Exception e) {
            e.printStackTrace();
            aprilTagFieldLayout = new AprilTagFieldLayout(new ArrayList<>(), fieldLength, fieldWidth);
        }

        leftPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_RIO, //FIXME MULTI_TAG_PNP_ON_COPROCESSOR
                LEFT_FRONT_ROBOT_TO_CAM);
        
        rightPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_RIO, //FIXME MULTI_TAG_PNP_ON_COPROCESSOR
                RIGHT_FRONT_ROBOT_TO_CAM);

        // Estrategia "Pegajosa": Usa la odometría para decidir entre posiciones espejo
        //FIXME
        /*
        leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        */
        leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (RobotBase.isSimulation()) {
            setupSimulation();
        }

        // Iniciar el Hilo de Visión a 50Hz (0.02s)
        visionThread = new Notifier(this::updateVision);
        visionThread.startPeriodic(0.02);
    }

    /**
     * Lógica principal de visión (Corre en Hilo Secundario)
     */
    private void updateVision() {
        if (aprilTagFieldLayout == null) return;

        //FIXME
        // Obtenemos la última posición conocida de forma segura
        /*
        Pose2d referencePose = robotPoseReference.get();
        leftPoseEstimator.setReferencePose(referencePose);
        rightPoseEstimator.setReferencePose(referencePose);
        */

        List<EstimatedRobotPose> newEstimates = new ArrayList<>();

        for (PhotonPipelineResult change : leftFrontCamera.getAllUnreadResults()) {
            if (shouldProcess(change)) {
                Optional<EstimatedRobotPose> est = leftPoseEstimator.update(change);
                if (est.isPresent() /*&& isPoseValid(est.get().estimatedPose)*/) { //FIXME
                    System.out.println("DEBUG VISION IZQ: " + est.get().estimatedPose.toPose2d().toString());
                    newEstimates.add(est.get());
                }
            }
        }

        for (PhotonPipelineResult change : rightFrontCamera.getAllUnreadResults()) {
            if (shouldProcess(change)) { 
                Optional<EstimatedRobotPose> est = rightPoseEstimator.update(change);
                if (est.isPresent() /*&& isPoseValid(est.get().estimatedPose)*/) { //FIXME
                    System.out.println("DEBUG VISION DER: " + est.get().estimatedPose.toPose2d().toString());
                    newEstimates.add(est.get());
                }
            }
        }
        // Publicamos los resultados para el Swerve
        latestEstimates.set(newEstimates);
    }

    /**
     * Filtro de Calidad: Descarta frames borrosos o ambiguos
     */
    private boolean shouldProcess(PhotonPipelineResult result) {
        if (!result.hasTargets()) return false;
        if (result.getTargets().size() > 1) return true; // Multi-tag siempre es bueno
        if (result.getTargets().size() == 1) {
            // Si es un solo tag, debe ser muy claro
            double ambiguity = result.getBestTarget().getPoseAmbiguity();
            if (ambiguity > MAX_AMBIGUITY_ALLOWED) return false;
        }
        return true;
    }

    /**
     * Método para que el Swerve obtenga los datos procesados
     */
    public List<EstimatedRobotPose> getLatestEstimates(Pose2d currentRobotPose) {
        robotPoseReference.set(currentRobotPose);
        return latestEstimates.getAndSet(new ArrayList<>());
    }

    private boolean isPoseValid(Pose3d pose) {
        if (pose.getX() < -0.5 || pose.getX() > fieldLength + 0.5) return false;
        if (pose.getY() < -0.5 || pose.getY() > fieldWidth + 0.5) return false;
        // Filtro de Altura corregido
        if (Math.abs(pose.getZ()) > MAX_HEIGHT_ERROR_METERS) return false;
        return true;
    }

    public Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimatedPose) {
        var estStdDevs = kSingleTagStdDevs; 
        var targets = estimatedPose.targetsUsed;
        int numTags = 0;
        double avgDist = 0;

        for (var target : targets) {
            //FIXME
            /*
            var tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;
            */
            numTags++;
            //avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
        }

        //FIXME
        /*
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        */

        if (numTags > 1) {
             estStdDevs = kMultiTagStdDevs;
        } 

        //FIXME
        /*
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            // Confianza ajustada (Divisor 20)
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 20));
        }
        */

        return estStdDevs;
    }

    // --- GETTERS PARA TELEMETRÍA (VisionTelemetry.java) ---
    
    public PhotonPipelineResult getLeftCameraResult() {
        return leftFrontCamera.getLatestResult();
    }

    public PhotonPipelineResult getRightCameraResult() {
        return rightFrontCamera.getLatestResult();
    }

    public double getLeftLatencyMs() {
        var res = leftFrontCamera.getLatestResult();
        if (res.hasTargets()) {
            return (Timer.getFPGATimestamp() - res.getTimestampSeconds()) * 1000.0;
        }
        return 0.0;
    }

    public double getRightLatencyMs() {
        var res = rightFrontCamera.getLatestResult();
        if (res.hasTargets()) {
            return (Timer.getFPGATimestamp() - res.getTimestampSeconds()) * 1000.0;
        }
        return 0.0;
    }

    // --- SIMULACIÓN ---

    public void simulationPeriodic(Pose2d robotSimPose) {
        if (visionSim != null) {
            visionSim.update(robotSimPose);
        }
    }

    private void setupSimulation() {
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(aprilTagFieldLayout);

        SimCameraProperties cameraProps = new SimCameraProperties();
        cameraProps.setCalibration(640, 480, Rotation2d.fromDegrees(70));
        cameraProps.setCalibError(0.25, 0.10);
        cameraProps.setFPS(30);
        cameraProps.setAvgLatencyMs(30);
        cameraProps.setLatencyStdDevMs(5);

        leftCameraSim = new PhotonCameraSim(leftFrontCamera, cameraProps);
        rightCameraSim = new PhotonCameraSim(rightFrontCamera, cameraProps);

        visionSim.addCamera(leftCameraSim, LEFT_FRONT_ROBOT_TO_CAM);
        visionSim.addCamera(rightCameraSim, RIGHT_FRONT_ROBOT_TO_CAM);
        
        leftCameraSim.enableDrawWireframe(true);
        rightCameraSim.enableDrawWireframe(true);
    }
}
