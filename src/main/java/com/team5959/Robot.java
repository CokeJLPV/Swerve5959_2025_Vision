// Copyright (c) FIRST and other WPILib contributors.
// Código de fuente abierto; puedes modificarlo y/o compartirlo bajo los términos del
// archivo de licencia WPILib BSD en el directorio raíz de este proyecto.

package com.team5959; //Paquete donde se encuentra este proyecto.
// Importa la clase TimedRobot, que proporciona la estructura básica para un robot basado en tiempo.
import edu.wpi.first.wpilibj.TimedRobot;

// Importa la interfaz Command, que representa una acción o tarea que puede ser programada y ejecutada.
import edu.wpi.first.wpilibj2.command.Command;

// Importa la clase CommandScheduler, que se encarga de gestionar la ejecución de comandos.
import edu.wpi.first.wpilibj2.command.CommandScheduler;
/**
 * La máquina virtual está configurada para ejecutar automáticamente esta clase, y para llamar a las funciones correspondientes a
 * cada modo, como se describe en la documentación de TimedRobot. Si cambias el nombre de esta clase o
 * el paquete después de crear este proyecto, también debes actualizar el archivo build.gradle en el
 * proyecto.
 */
public class Robot extends TimedRobot {
  //Instancia para guardar el comando autónomo que se ejecutará en modo autónomo.
  private Command m_autonomousCommand;

  //Instancia de la clase RobotContainer que gestiona los subsistemas, comandos y enlaces de botones.
  private RobotContainer m_robotContainer;
 /**
   * Esta función se ejecuta cuando el robot se inicia por primera vez y debe usarse para cualquier
   * código de inicialización.
   */
  @Override
  public void robotInit() {
    // Instancia nuestro RobotContainer. Esto realizará todas nuestras asignaciones de botones y colocará nuestro
    // selector de autónomo en el dashboard.
    m_robotContainer = new RobotContainer();
  }
  /**
   * Esta función se llama cada 20 ms, sin importar el modo. Úsala para elementos como diagnósticos
   * que deseas ejecutar durante deshabilitado, autónomo, teleoperado y prueba.
   *
   * <p>Esto se ejecuta después de las funciones periódicas específicas del modo, pero antes de las actualizaciones integradas de
   * LiveWindow y SmartDashboard.
   */
  @Override
  public void robotPeriodic() {
    // Ejecuta el Scheduler. Esto es responsable de consultar botones, agregar comandos recién programados,
    // ejecutar comandos ya programados, eliminar comandos terminados o interrumpidos,
    // y ejecutar métodos periódicos de subsistemas. Esto debe llamarse desde el bloque periódico del robot
    // para que cualquier cosa en el marco basado en comandos funcione.
    CommandScheduler.getInstance().run();
  }
  /** Esta función se llama una vez cada vez que el robot entra en modo Deshabilitado. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  /** Este autónomo ejecuta el comando autónomo seleccionado por tu clase {@link RobotContainer}. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
   
    // programa el comando autónomo (ejemplo)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** Esta función se llama periódicamente durante el modo autónomo. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Esto asegura que el autónomo se detenga cuando
    // el teleoperado comience a ejecutarse. Si deseas que el autónomo
    // continúe hasta que sea interrumpido por otro comando, elimina
    // esta línea o coméntala.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
  /** Esta función se llama periódicamente durante el control del operador. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    // Cancela todos los comandos en ejecución al inicio del modo de prueba.
    CommandScheduler.getInstance().cancelAll();
    }

  /** Esta función se llama periódicamente durante el modo de prueba. */
  @Override
  public void testPeriodic() {}

  /** Esta función se llama una vez cuando el robot se inicia por primera vez. */
  @Override
  public void simulationInit() {}

  /** Esta función se llama periódicamente mientras está en simulación. */
  @Override
  public void simulationPeriodic() {}
}