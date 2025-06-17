# Proyecto de Robótica móvil

## Descripción

Este proyecto de robótica móvil tiene como objetivo desarrollar un sistema autónomo capaz de navegar y realizar tareas específicas en un entorno simulado utilizando ROS 2 y CoppeliaSim. El robot será capaz de cartografiar su entorno, planificar rutas y ejecutar órdenes de navegación para recoger y entregar objetos.

## Objetivos

- Desarrollar un sistema de navegación autónoma para un robot móvil.
- Implementar la cartografía y localización del entorno utilizando ROS 2.
- Integrar el simulador CoppeliaSim para pruebas y validación.
- Crear un sistema de teleoperación para el control manual del robot.
- Desarrollar un sistema de planificación de rutas y ejecución de órdenes.

## Requisitos

- ROS 2 Humble
- CoppeliaSim Edu V4.9.0
- TurtleBot3
- Ubuntu 22.04

### ¿Cómo instalar CoppeliaSim?

Para instalar CoppeliaSim, sigue estos pasos:

```bash
cd /home/USER/non-snap/CoppeliaSim
wget https://downloads.coppeliarobotics.com/V4_9_0_rev6/CoppeliaSim_Edu_V4_9_0_\
rev6_Ubuntu22_04.tar.xz
tar -xvf CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04.tar.xz
rm CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04.tar.xz
```

### Para añadir en el .bashrc

Añade las siguientes líneas a tu archivo `.bashrc` para configurar el entorno:

```sh
# >>> ROS 2 Humble >>>

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source /home/USER/.../ROM_2425/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02 # puede que esto sobre

# <<< ROS 2 Humble <<<

# >>> CoppeliaSim >>>

export PATH=$PATH:/home/USER/non-snap/CoppeliaSim/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04/

# <<< CoppeliaSim <<<
```

## ¿Cómo ejecutar el programa?

### Paso 1. Cartografiar en escenario

#### Terminal 1

Lanzar paquetes para cartografiar

```bash
ros2 launch little_warehouse cartographer.launch.py resolution:=0.001
```

*Captura de pantalla*

#### Terminal 2

Lanzar nodo teleoperador para moverse por el entorno

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

*Captura de pantalla*

#### Terminal 3

Generar un mapa `.pgm` de lo que se ha cartografiado

```bash
ros2 run nav2_map_server map_saver_cli -f ./little_warehouse_map
```

*Captura de pantalla*

### Paso 2. Ejecutar programa principal?

Asegúrate de que no tienes en ejecución ningún proceso de los que lanzamos anteriormente, y en una terminal ejecuta primeramente el simulador:

#### Terminal 1

Nos aseguramos de que no tenemos en ejecución ningún proceso de los que lanzamos anteriormente, y en una terminal ejecutamos primeramente el simulador:

```bash
ros2 launch little_warehouse coppeliasim_no_rviz2.launch.py
```

*Captura de pantalla*

#### Terminal 2

Lanza el fichero launch del limitador de velocidad:

```bash
ros2 launch little_warehouse speed_limit.launch.py params_file:=./params/speed_params.yaml mask:=./maps/speed_mask_coppeliasim_map.yaml
```

*Captura de pantalla*

#### Terminal 3

Ahora debemos lanzar el launch de navegación:

```bash
ros2 launch little_warehouse navigation_with_speed_limit.launch.py map:=./maps/coppeliasim_map.yaml params_file:=./params/nav2_params_speed_limit.yaml
```

*Captura de pantalla*

#### Terminal 4

Por último, lanza el nodo que enviará al robot las posiciones por donde debe pasar para recoger los elementos entre las estanterías:

```bash
ros2 run little_warehouse navigation_node
```

Otra forma de lanzar este nodo es especificando qué orden se quiere realizar:

```bash
ros2 run little_warehouse navigation --ros-args -p order:="friday"
```

El argumento order puede ser: "monday, tuesday, wednesday, thursday, friday, saturday, sunday".

*Captura de pantalla*

## Conclusiones y trabajo a futuro

Blabla

## Licencia

Este proyecto está licenciado bajo la Licencia MIT. Para más detalles, consulta el archivo LICENSE.

## Agradecimientos

Agradecemos a nuestros profesores de Robótica Móvil: Leopoldo Armesto y Ricardo Nuñez, por su guía y apoyo en el desarrollo de este proyecto.

