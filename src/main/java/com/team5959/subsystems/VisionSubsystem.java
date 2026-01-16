package com.team5959.subsystems;

import org.photonvision.targeting.PhotonPipelineResult;

import com.team5959.Vision;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private final Vision vision;
  public VisionSubsystem(Vision vision) {
    this.vision = vision;

    // Nombre del subsistema en el Dashboard
    this.setName("VisionSystem"); 

  }

  @Override
  public void periodic() {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Subsystem");

    // --- CÁMARA IZQUIERDA ---
    builder.addBooleanProperty("Left/HasTarget", 
        () -> vision.getLeftCameraResult().hasTargets(), null);
    
    builder.addIntegerProperty("Left/ID", 
        () -> getTargetID(vision.getLeftCameraResult()), null);

    builder.addDoubleProperty("Left/Ambiguity", 
        () -> getAmbiguity(vision.getLeftCameraResult()), null);
        
    /* CORREGIDO: Usamos el método auxiliar getLatency para evitar el error de compilación
    builder.addDoubleProperty("Left/LatencyMs", 
        () -> getLatency(vision.getLeftCameraResult()), null);
    */

    builder.addDoubleProperty("Left/DistToTag", 
        () -> getDistanceToTag(vision.getLeftCameraResult()), null);

    builder.addIntegerProperty("Left/TagCount", 
        () -> getTagCount(vision.getLeftCameraResult()), null);


    // --- CÁMARA DERECHA ---
    builder.addBooleanProperty("Right/HasTarget", 
        () -> vision.getRightCameraResult().hasTargets(), null);

    builder.addIntegerProperty("Right/ID", 
        () -> getTargetID(vision.getRightCameraResult()), null);

    builder.addDoubleProperty("Right/Ambiguity", 
        () -> getAmbiguity(vision.getRightCameraResult()), null);

    /*  CORREGIDO: Usamos el método auxiliar getLatency
    builder.addDoubleProperty("Right/LatencyMs", 
        () -> getLatency(vision.getRightCameraResult()), null);
    */

    builder.addDoubleProperty("Right/DistToTag", 
        () -> getDistanceToTag(vision.getRightCameraResult()), null);

    builder.addIntegerProperty("Right/TagCount", 
        () -> getTagCount(vision.getRightCameraResult()), null);
  }

  // --- Métodos auxiliares ---
  
  private int getTargetID(PhotonPipelineResult result) {
      if (result.hasTargets()) return result.getBestTarget().getFiducialId();
      return -1;
  }

  private double getAmbiguity(PhotonPipelineResult result) {
      if (result.hasTargets()) return result.getBestTarget().getPoseAmbiguity();
      return 0.0;
  }

  /* Método auxiliar para evitar el error de compilación con DoubleSupplier
  private double getLatency(PhotonPipelineResult result) {
      return result.getLatencyMillis();
  }*/

  // Calcula la distancia lineal al tag (hipotenusa 3D)
  private double getDistanceToTag(PhotonPipelineResult result) {
      if (result.hasTargets()) {
          return result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
      }
      return 0.0;
  }

  private int getTagCount(PhotonPipelineResult result) {
      if (result.hasTargets()) return result.getTargets().size();
      return 0;
  }
}
