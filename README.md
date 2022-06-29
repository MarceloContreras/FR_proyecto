# FR_proyecto
## Proyecto Final del curso de Fundamentos de Robótica (MT-0006) UTEC 

El presente proyecto se ha encargado de modificar el brazo robótico del Spot Robot de Boston Dynamics para aumentarle una articulación prismática, obteniendo un robot manipulador de 7 grados de libertad para amplificar el espacio operacional y alcanzable. Se definieron nuevas dimensiones y en base a ellas, se obtuvieron los parámetros Dehavit - Hartenberg. El robot modificado fue modelado mediante Inventor, generando un modelo enmallado que fue exportado como archivo formato URDF para utilizarse en ROS. Fueron definidas tres configuraciones de prueba que fueron visualizadas mediante Rviz para cinemática directa, cinemática inversa de posición y control cinemático de posición. También, fue obtenido el modelo dinámico del robot a través de la librería RBDL y fue empleado para dos esquemas de control dinámico. Tanto el control cinemático como dinámico logró seguir las referencias con una norma de error menor a 0.0001.

## Dependencias necesarias

### ROS Kinetic

### Python (v2.8)
* NumPy
* RosPy
* MatPlotlib
* RBDL 

## Estructura del robot

![alt text](https://github.com/MarceloContreras/FR_proyecto/blob/main/DH_Axes.png)

Lista de parámetros DH:

|   |      $d_1$     |    $\theta$    |     a    | $\alpha$ |
|:-:|:--------------:|:--------------:|:--------:|:--------:|
| 1 |    0.140137    |  $q_0 + \pi$   |     0    |  $\pi\2$ |
| 2 |        0       |  $-q_1 + \pi$  |  0.3385  |     0    |
| 3 |        0       | $q_2 + \pi\2$  | -0.09734 |  $\pi\2$ |
| 4 |        0       |     $q_3$      |     0    |     0    |
| 5 | $q_4$ + 0.3833 |      $\pi$     |     0    |  $\pi\2$ |
| 6 |        0       |  $q_5 + \pi$   |     0    |  $\pi\2$ |
| 7 |    0.275027    |  $q_6 + \pi$   | -0.05025 |     0    |

## Modelamiento del robot

Como se menciona al inicio, el modelamiento fue logrado a partir de un proyecto Inventor, posteriormente exportado a URDF y Xacro mediante los scripts encontrados en el repositorio https://github.com/syuntoku14/fusion2urdf.

![alt text](https://github.com/MarceloContreras/FR_proyecto/blob/main/Spot_fusion.png)

## Visualización y simulación

```bash
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

![alt text](https://github.com/MarceloContreras/FR_proyecto/blob/main/Modelo_Rviz.png)



