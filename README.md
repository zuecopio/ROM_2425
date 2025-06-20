# 🤖 Proyecto de Robótica Móvil

![Imagen de ejemplo del proyecto.](media/main.png)

## Índice

- [Descripción](#-descripción)
- [Objetivos](#-objetivos)
- [Requisitos](#-requisitos)
    - [Instalación de CoppeliaSim](#-instalación-de-coppeliasim)
    - [Configuración del entorno de ROS 2](#-configuración-del-entorno-de-ros-2)
    - [Clonar repositorio y compilar el paquete](#-clonar-repositorio-y-compilar-el-paquete)
- [Ejecución del programa](#-ejecución-del-programa)
    - [Paso 1. Cartografiar en escenario](#-paso-1-cartografiar-en-escenario)
    - [Paso 2. Ejecutar programa principal](#-paso-2-ejecutar-programa-principal)
- [Conclusiones](#-conclusiones)
- [Trabajo a futuro](#-trabajo-a-futuro)
- [Licencia](#-licencia)
- [Agradecimientos](#-agradecimientos)

## 📋 Descripción

Este proyecto de robótica móvil tiene como objetivo desarrollar un sistema autónomo capaz de navegar y realizar tareas específicas en un entorno simulado utilizando ROS 2 y CoppeliaSim. 
El robot será capaz de:
- 🗺️ Cartografiar su entorno
- 📍 Planificar rutas
- 📦 Ejecutar órdenes de navegación para recoger y entregar objetos

## 🎯 Objetivos

- ✅ Desarrollar un sistema de navegación autónoma para un robot móvil.
- ✅ Implementar la cartografía y localización del entorno utilizando ROS 2.
- ✅ Integrar el simulador CoppeliaSim para pruebas y validación.
- ✅ Crear un sistema de teleoperación para el control manual del robot.
- ✅ Desarrollar un sistema de planificación de rutas y ejecución de órdenes.

## 🧰 Requisitos

- 🐢 ROS 2 Humble
- 🖥️ CoppeliaSim Edu V4.9.0
- 🤖 TurtleBot3
- 🐧 Ubuntu 22.04

### 🛠️ Instalación de CoppeliaSim

Para instalar CoppeliaSim, sigue estos pasos:

```bash
cd ~
wget https://downloads.coppeliarobotics.com/V4_9_0_rev6/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04.tar.xz
tar -xvf CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04.tar.xz
rm CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04.tar.xz
echo "export PATH=$PATH:~/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04" >> ~/.bashrc
```

### ⚙️ Configuración del entorno de ROS 2

Para configurar el entorno de ROS 2, añade las siguientes líneas a tu archivo `~/.bashrc` desde la terminal:

```bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo "export LDS_MODEL=LDS-02" >> ~/.bashrc
```

### 👩‍💻 Clonar repositorio y compilar el paquete

Para clonar el repositorio y compilar el paquete, sigue estos pasos:

1. Clona el repositorio en una carpeta conocida:

    ```bash
    cd ~
    git clone https://github.com/zuecopio/ROM_2425
    ```

2. Compila el paquete:

    ```bash
    cd ROM_2425/ros2_ws
    colcon build
    ```

3. Configura el entorno para el paquete compilado, para hacer este valor permanente lo ponemos en el `.bashrc`:

    ```bash
    echo "source ~/ROM_2425/ros2_ws/install/setup.bash" >> ~/.bashrc
    ```

## 🚀 Ejecución del programa

### 🔍 Paso 1. Cartografiar en escenario

#### 🧭 Terminal 1 - Lanzar escenario de cartografia con CoppeliaSim:

```bash
ros2 launch little_warehouse cartographer.launch.py resolution:=0.001
```

#### 🎮 Terminal 2 - Lanzar nodo teleoperador para moverse por el entorno:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

![A la derecha: escenario en CoppeliaSim; abajo a la izquierda: interfaz en RViz2; arriba a la izquierda: nodo de teleoperación.](media/cartographer.png)

> A la derecha: escenario en CoppeliaSim; abajo a la izquierda: interfaz en RViz2; arriba a la izquierda: nodo de teleoperación.

#### 🗺️ Terminal 3 - Guardar el mapa (.pgm):

Una vez ya se ha realizado una exploración del escenario, se ejecuta el siguiente comando para guardar un mapa.

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ROM_2425/ros2_ws/src/little_warehouse/maps/coppeliasim_map
```

Esto genera en la carpeta de `maps` un fichero `coppeliasim_map.yaml` y una imagen `coppeliasim_map.pgm`.

Es posible que el mapa no se genere de la manera esperada, pero se puede ajustar utilizando una aplicación como GIMP.

|   Imagen original del mapa   |    Imagen del mapa retocado con GIMP   |
|------------------------------|----------------------------------------|
| ![Imagen original del mapa](./media/coppeliasim_map.png) | ![Imagen del mapa retocado con GIMP](./media/coppeliasim_map_improved.png) |

Además, se pueden agregar zonas de limitación de velocidad pintando áreas con diferentes tonalidades de gris. Cuanto más oscuro sea el color, menor será la velocidad permitida; en cambio, cuanto más claro sea, la limitación de velocidad será menos estricta.

|   Imagen del mapa con las limitaciones de velocidad   |
|------------------------------|
| ![Imagen del mapa con las limitaciones de velocidad](./media/speed_mask_coppeliasim_map.png) |

### 🧠 Paso 2. Ejecutar programa principal

Asegúrate de cerrar todos los procesos anteriores.

#### 🧪 Terminal 1 - Lanzar simulador:

```bash
ros2 launch little_warehouse coppeliasim_no_rviz2.launch.py
```

![Escenario en CoppeliaSim.](media/coppeliasim_scene.png)

> Escenario en CoppeliaSim.


#### 🛑 Terminal 2 - Limitador de velocidad:

Esta terminal permanecerá a la espera de que se inicie el navegador con la [Terminal 3](#-terminal-3---navegación). Luego, finalizará su ejecución.

```bash
cd ~/ROM_2425/ros2_ws/src/little_warehouse  # importante situarse en esta carpeta
ros2 launch little_warehouse speed_limit.launch.py params_file:=./params/speed_params.yaml mask:=./maps/speed_mask_coppeliasim_map.yaml
```

#### 🧭 Terminal 3 - Navegación:

```bash
cd ~/ROM_2425/ros2_ws/src/little_warehouse # importante situarse en esta carpeta
ros2 launch little_warehouse navigation_with_speed_limit.launch.py map:=./maps/coppeliasim_map.yaml params_file:=./params/nav2_params_speed_limit.yaml
```

![Interfaz en RViz2 para la navegación.](media/rviz2_navegation.png)

> Interfaz en RViz2 para la navegación.

#### 📦 Terminal 4 - Enviar orden de envío:

Para iniciar la orden de un pedido, se ejecuta el nodo `navigation_node`. Si no se indica qué orden se desea lanzar, se utilizará, por defecto, la del lunes.

```bash
ros2 run little_warehouse navigation_node
```

También es posible lanzar este nodo especificando la orden que se quiere realizar:

```bash
ros2 run little_warehouse navigation_node --ros-args -p order:="friday"
```

🗓️ El argumento `order` puede ser uno de los siguientes: `monday`, `tuesday`, `wednesday`, `thursday`, `friday`, `saturday` o `sunday`.

![Publicación de las posiciones para completar el pedido.](media/navigation_node.png)

> Publicación de las posiciones para completar el pedido.

El contenido de las órdenes incluye diversos elementos que están distribuidos en las estanterías. Cada orden tiene una lista de componentes que se puede consultar en el archivo `orders.yaml`. El robot puede localizar los elementos gracias al fichero `positions.yaml`, que define para cada elemento una posición de pick para el robot.

Por ejemplo, la orden `monday` contiene los siguientes elementos:

```yaml
monday:
  - apple
  - blueberry
  - cherry
  - mango
  - melon
  - nectarine
  - papaya
  - peach
  - raspberry
  - watermelon
```

#### 🏁 Resultados finales

| Escenario en CoppeliaSim | Interfaz en RViz2 para la navegación |
|------------------------------|------------------------------|
| ![Escenario en CoppeliaSim.](media/coppeliasim_scene_final.png) | ![Interfaz en RViz2 para la navegación.](media/rviz2_navegation_final.png) |

## 📌 Conclusiones

- ✅ Se ha logrado implementar un sistema completo de navegación autónoma en un entorno simulado, integrando ROS 2 y CoppeliaSim.
- ✅ Ha sido posible cartografiar el entorno y guardar el mapa generado para su posterior uso.
- ✅ Se ha desarrollado un sistema de planificación y ejecución de órdenes de navegación con restricciones de velocidad.
- ✅ Se ha incorporado un sistema de teleoperación útil para exploración y depuración, principalmente para cartografiar el entorno.
- ✅ El proyecto nos ha permitido aplicar los conocimientos de percepción, control y planificación en un entorno de simulación realista aprendidos durante el desarrollo de la asignatura de Robótica Móvil.

## 🌱 Trabajo a futuro

- 🔧 Simular el proceso de recogida de los productos de las estanterías.
- 🧭 Ajustar y optimizar los parámetros del archivo `nav2_params_speed_limit.yaml` para mejorar la eficiencia, permitiendo la eliminación de los puntos de navegación auxiliares en los extremos de las estanterías que facilitaban el movimiento del robot entre ellas.
- 🏗️ Revisar la configuración del escenario para mejorar la navegabilidad: esto puede implicar aumentar la separación entre estanterías en la simulación o, alternativamente, migrar a un simulador más avanzado que permita una mejor generalización a entornos reales donde se busca optimizar el espacio sin necesidad de grandes separaciones.

## 📄 Licencia

Este proyecto está licenciado bajo la Licencia MIT. 
Consulta el archivo `LICENSE` 📜 para más información.

## 🙏 Agradecimientos

Agradecemos a nuestros profesores de Robótica Móvil: 👨‍🏫 Leopoldo Armesto y 👨‍🏫 Ricardo Nuñez, por su guía y apoyo en el desarrollo de este proyecto.

