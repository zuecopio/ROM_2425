# Proyecto de Robótica móvil

## Descripcción?

## Objetivos?

## Requisitos

Blabla

### ¿Cómo instalar CoppeliaSim?

Blabla

```bash
cd /home/alumno.upv.es.TU_USUARIO/non-snap/CoppeliaSim
wget https://downloads.coppeliarobotics.com/V4_9_0_rev6/CoppeliaSim_Edu_V4_9_0_\
rev6_Ubuntu22_04.tar.xz
tar -xvf CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04.tar.xz
rm CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04.tar.xz
```

### Para añadir en el .bashrc

Blabla

```sh
# >>> ROS 2 Humble >>>

export ROS_DOMAIN_ID=EL_NUMERO_QUE_QUIERASS
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source /home/alumno.upv.es.TU_USUARIO/W/GIIROB/G3/ROM/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02

# <<< ROS 2 Humble <<<

# >>> CoppeliaSim >>>

export PATH=$PATH:~/non-snap/CoppeliaSim/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04/

# <<< CoppeliaSim <<<
```

## ¿Cómo ejecutar el programa?

Blabla

### Paso 1. Cartografiar en escenario

Blabla

#### Terminal 1: Lanzar paquetes para cartografiar

Blabla

```bash
ros2 launch little_warehouse cartographer.launch.py resolution:=0.001
```

#### Terminal 2: Lanzar nodo teleoperador para moverse por el entorno

Blabla

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

#### Terminal 3: Generar un mapa .pmg de lo que se ha cartografiado

Blabla

```bash
ros2 run nav2_map_server map_saver_cli -f ./little_warehouse_map
```

### Paso 2. Ejecutar programa principal?

Blabla

#### Terminal 1: 

Nos aseguramos de que no tenemos en ejecución ningún proceso de los que lanzamos anteriormente, y en una terminal ejecutamos primeramente el simulador:

```bash
ros2 launch little_warehouse coppeliasim_no_rviz2.launch.py
```

#### Terminal 2:

Lanzamos el fichero launch del limitador de velocidad:

```bash
ros2 launch little_warehouse speed_limit.launch.py params_file:=./param/speed_params.yaml mask:=./maps/speed_mask_coppeliasim_map.yaml
```

#### Terminal 3:

Ahora debemos lanzar el launch de navegación:

```bash
ros2 launch little_warehouse navigation_with_speed_limit.launch.py map:=./maps/coppeliasim_map.yaml params_file:=./param/nav2_params_speed_limit.yaml
```

#### Terminal 4:

Por último se lanza el nodo que enviará al robot la posiciones por donde debe pasar para recoger los elementos entre las estanterías:

```bash
ros2 run little_warehouse navigation
```

Otra forma de lanzar este nodo es especificando que orden se quiere realizar:

```bash
ros2 run little_warehouse navigation --ros-args -p order:="friday"
```

El argumento order puede ser: "monday, tuesday, wensday, thursday, friday, saturday, sunday"

## Licencia

Blabla

## Agradecimientos

Blabla (a nuestros profes de Robótica Móvil: Leopoldo Armesto y  Ricardo Nuñez)

