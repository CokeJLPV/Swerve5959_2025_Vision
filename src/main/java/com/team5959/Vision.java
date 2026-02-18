package com.team5959;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Vision {
    private final PhotonCamera leftFrontCamera;
    private final PhotonCamera rightFrontCamera;

    private final PhotonPoseEstimator leftPoseEstimator;
    private final PhotonPoseEstimator rightPoseEstimator;
    private AprilTagFieldLayout aprilTagFieldLayout;

    private final EstimateConsumer estConsumer;

    // Constantes de Confianza
    private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.1,0.1,0.1);//(1, 1, 4); //FIXME despues de las pruebas
    private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.01,0.01,0.01);//(0.1, 0.1, 1);

    // Filtros de Seguridad
    /*
    private double fieldLength = 50; //17.55
    private double fieldWidth = 50;//8.05
    private static final double MAX_HEIGHT_ERROR_METERS = 50; //FIXME 0.5m
    private static final double MAX_AMBIGUITY_ALLOWED = 0.5; // Filtro Anti-Jitter //0.2
    */

    private static final Transform3d LEFT_FRONT_ROBOT_TO_CAM = new Transform3d(
            new Translation3d(0.3, 0.3, 0.2), 
            new Rotation3d(0, Units.degreesToRadians(22), Units.degreesToRadians(45)));

    private static final Transform3d RIGHT_FRONT_ROBOT_TO_CAM = new Transform3d(
            new Translation3d(0.3, -0.3, 0.2), 
            new Rotation3d(0, Units.degreesToRadians(22), Units.degreesToRadians(-45)));

    public Vision(EstimateConsumer estConsumer) {
        this.estConsumer = estConsumer;
        leftFrontCamera = new PhotonCamera("LeftAprilTagCamera"); // Asegúrate que coincida con PhotonVision UI
        rightFrontCamera = new PhotonCamera("RightAprilTagCamera");

        try {
            // Carga campo 2026 Oficial
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
            System.out.println("VISION: Mapa 2026 Rebuilt cargado correctamente.");
        } catch (Exception e) {
            System.out.println("VISION ERROR CRÍTICO: No se pudo cargar el mapa 2026.");
            e.printStackTrace();
            aprilTagFieldLayout = new AprilTagFieldLayout(new ArrayList<>(), 16.54, 8.21);
        }

        leftPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, //FIXME MULTI_TAG_PNP_ON_COPROCESSOR
                LEFT_FRONT_ROBOT_TO_CAM);
        
        rightPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, //FIXME MULTI_TAG_PNP_ON_COPROCESSOR
                RIGHT_FRONT_ROBOT_TO_CAM);

        // Estrategia "Pegajosa": Usa la odometría para decidir entre posiciones espejo
        //FIXME
        /*
        leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        */
        leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * Este método debe ser llamado periódicamente (desde SwerveChassis.periodic)
     */
    public void periodic() {
        if (aprilTagFieldLayout == null) return;

        // 1. Procesar Cámara Izquierda
        processCamera(leftFrontCamera, leftPoseEstimator);

        // 2. Procesar Cámara Derecha
        processCamera(rightFrontCamera, rightPoseEstimator);
    }

    /**
     * Lógica unificada para procesar cualquier cámara
     */
    private void processCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {
        for (var result : camera.getAllUnreadResults()) {
            if (!result.hasTargets()) continue;

            // Filtro básico de ambigüedad para single-tag en pruebas
            if (result.getTargets().size() == 1 && result.getBestTarget().getPoseAmbiguity() > 0.4) {
                continue; 
            }

            Optional<EstimatedRobotPose> visionEst = estimator.update(result);
            
            visionEst.ifPresent(est -> {
                // Calcular confianza dinámica basada en la heurística de la plantilla
                var stdDevs = getEstimationStdDevs(est, result.getTargets());
                
                // ENVIAR AL CONSUMIDOR (SwerveChassis)
                estConsumer.accept(
                    est.estimatedPose.toPose2d(), 
                    est.timestampSeconds, 
                    stdDevs
                );
            });
        }
    }

    /**
     * Algoritmo heurístico de la plantilla para calcular confianza dinámica
     */
    private Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {
        var estStdDevs = kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        for (var tgt : targets) {
            var tagPose = aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tgt.getBestCameraToTarget().getTranslation().getNorm();
        }

        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;

        if (numTags > 1) {
            estStdDevs = kMultiTagStdDevs;
        } else {
            // Un solo tag
            // Aumentar la incertidumbre según la distancia (Heurística de la plantilla)
            // Factor: 1 + (dist^2 / 30)
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        return estStdDevs;
    }

    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
