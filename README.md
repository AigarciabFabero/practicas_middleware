# MIDDLEWARE - MÁSTER ROBÓTICA E IA

[ROS NOETIC](#ros---noetic)<br>
[ROS2 HUMBLE](#ros2-humble)<br>
[YARP](#yarp)

## [ROS NOETIC](./ros/readme_ros.md)

Este repositorio contiene prácticas y ejercicios diseñados para aprender y aplicar conceptos avanzados de **Robot Operating System (ROS)**, específicamente relacionados con la gestión de **topics**, colas, y el manejo de concurrencia mediante spinners multihilo. Estos ejercicios son parte del programa del **Máster en Robótica e Inteligencia Artificial** y tienen como objetivo principal optimizar el flujo de datos y la estabilidad del sistema en entornos robóticos.

### Objetivos

- Familiarizarse con la configuración y optimización de colas al crear **publicadores** y **suscriptores** en ROS.
- Implementar y comparar modelos de concurrencia utilizando **spinners de un solo hilo (single-thread)** y **multihilo (multi-threaded spinners)**.
- Explorar estrategias para manejar la sobrecarga de mensajes en sistemas multihilo.
- Medir el impacto de configuraciones avanzadas en el rendimiento de nodos ROS.

### Contenido

#### Ejercicio 1: Publicador y Suscriptor Básico
- Implementación de nodos publicadores y suscriptores en **Python** y **C++**.
- Ajuste de la profundidad de colas y análisis del impacto en el flujo de datos.

#### Ejercicio 2: Gestión Multihilo con Spinners
- Uso de **AsyncSpinner** en Python y **MultiThreadedSpinner** en C++ para manejar múltiples callbacks simultáneamente.
- Creación de nodos que procesan datos desde varios topics sin bloquearse.

#### Ejercicio 3: Comparación entre `spinOnce()` y `spin()`
- Diferencias prácticas entre estos métodos para la gestión del ciclo de vida de nodos.

#### Ejercicio 4: Manejo de Sobrecarga en un Sistema Multihilo
- Simulación de un sistema con múltiples publicadores a distintas velocidades.
- Análisis del rendimiento del sistema ajustando el número de hilos y la configuración de las colas.

### Requisitos

- **Sistema Operativo**: Ubuntu 22.04
- **ROS**: Noetic
- **Dependencias**: 
  - [Repositorio oficial de tutoriales de ROS](https://github.com/fjrodl/ros_tutorials)

## [ROS2 HUMBLE](./ros2/readme_ros2.md)

Este repositorio contiene una serie de ejercicios diseñados para introducir y profundizar en conceptos clave de ROS 2, como la publicación y suscripción de mensajes, la definición de configuraciones avanzadas de Quality of Service (QoS) y la gestión de concurrencia mediante executors multihilo en Python y C++. A través de estos ejercicios, se explorarán diferentes estrategias para manejar tareas simultáneas, ajustar colas de topics y optimizar la comunicación entre nodos. Además, se analizarán las implicaciones de estas configuraciones en el rendimiento de sistemas robóticos distribuidos.

### Objetivos

- Familiarizarse con el modelo Publish/Subscribe en ROS 2.
- Comprender las configuraciones de Quality of Service (QoS) en ROS 2.
- Aprender a manejar colas y opciones de publicación y suscripción en ROS 2.
- Introducirse a la concurrencia y gestión multihilo utilizando SingleThreadedExecutor y MultiThreadedExecutor.
- Desarrollar habilidades para crear y utilizar servicios y mensajes personalizados en ROS 2.

### Contenido

#### Ejercicios: Publicación y Suscripción en ROS 2

1. **Implementación de un Publicador Mínimo**: Crear un nodo que publique mensajes en el topic `/chatter`.
2. **Implementación de un Suscriptor Mínimo**: Desarrollar un nodo que procese mensajes del topic `/chatter`.
3. **Publicador y Suscriptor con Mensajes Personalizados**: Definir y usar mensajes personalizados.
4. **Publicador con Temporizador**: Controlar la frecuencia de publicación usando temporizadores.
5. **Suscriptor con Callback Personalizado**: Procesar mensajes con funciones específicas.
6. **Publicador y Suscriptor con QoS en ROS 2**: Experimentar con diferentes configuraciones de QoS.

#### Ejercicios: Gestión de Concurrencia y Callbacks en ROS 2

1. **Grupos de Callbacks**: Manejar tareas concurrentes en ROS 2.
2. **Composición de Múltiples Nodos**: Ejecutar varios nodos en un único proceso.
3. **Nodo Combinado**: Implementar un nodo que actúe como publicador y suscriptor simultáneamente.
4. **Creación de un Grupo de Callbacks Personalizado**.
5. **Desarrollo de un Ejecutor Personalizado**.

#### Ejercicio Avanzado: Creación de un Servicio en ROS 2

- **Servicio para Contar Vocales**: Crear un servicio que reciba una palabra y devuelva el número de vocales.

### Requisitos

- **Sistema Operativo**: Ubuntu 22.04
- **ROS 2**: Humble
- **Repositorio de Ejemplos**: [GitHub de ROS 2](https://github.com/ros2/examples)

## [YARP](./yarp/readme_yarp.md)

YARP (Yet Another Robot Platform) es un middleware de código abierto diseñado para facilitar la comunicación modular y distribuida en sistemas robóticos. Proporciona herramientas y bibliotecas para conectar componentes de software que interactúan en un entorno robótico, permitiendo que diferentes módulos se comuniquen entre sí de manera eficiente, independientemente de su ubicación o lenguaje de programación.

### Objetivos

- Familiarizarse con el uso de puertos en YARP tanto desde la línea de comandos como desde nodos programados en C++.
- Practicar la creación y configuración de conexiones entre puertos para el envío y recepción de mensajes.
- Comprender y aplicar conceptos de programación multihilo en YARP, gestionando tareas concurrentes entre puertos.
- Aprender a desarrollar nodos simples en YARP para la comunicación eficiente entre módulos de un sistema robótico.

### Contenido

#### Ejercicios 1: Comandos Básicos de YARP en la Línea de Comandos
- **Descripción**: Uso de comandos para verificar el entorno de YARP, iniciar el servidor de nombres, crear y manipular puertos, y realizar pruebas de conectividad.
- **Objetivo**: Familiarizarse con el CLI en YARP.

#### Ejercicios 2: Uso de Puertos en YARP
- **Descripción**: Implementación de nodos que utilizan puertos de escritura y lectura para enviar y recibir mensajes.
- **Objetivo**: Comprender la estructura básica de comunicación en YARP.

#### Ejercicios 3: BufferedPort en YARP
- **Descripción**: Uso de puertos con almacenamiento en búfer para manejar comunicación asíncrona.
- **Objetivo**: Practicar la comunicación confiable y asincrónica entre módulos.

#### Ejercicios 4: Tiempo Real en YARP
- **Descripción**: Uso del método `setStrict()` para procesar únicamente los mensajes más recientes en aplicaciones en tiempo real.
- **Objetivo**: Gestionar datos de manera eficiente en sistemas donde la información actualizada es crítica.

#### Ejercicios 5: Multihilo en YARP
- **Descripción**: Implementación de tareas concurrentes utilizando la clase `yarp::os::Thread`.
- **Objetivo**: Diseñar sistemas distribuidos y concurrentes con múltiples hilos en YARP.

#### Ejercicios 6: Relación entre YARP, ROS y ROS 2
- **Descripción**: Comparación de las características y comportamiento de YARP, ROS y ROS 2.
- **Objetivo**: Reconocer las similitudes y diferencias entre estos middlewares.

### Recursos

- **Docker básico para prácticas**: [Repositorio en GitHub](https://github.com/fjrodl/icub-gazebo-grasping-sandbox)
- **Instrucciones de configuración**: [Guía de configuración del Docker](https://github.com/fjrodl/icub-gazebo-grasping-sandbox/blob/master/dockerfiles/README.md)

### Requisitos

- **Sistema Operativo**: Ubuntu 22.04 o entorno compatible.
- **Entorno YARP**:
  - Servidor de nombres (`yarpserver`).
  - Compilador con soporte para CMake.