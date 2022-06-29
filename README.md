# FR_proyecto
## Proyecto Final del curso de Fundamentos de Robótica (MT-0006) UTEC 

El presente proyecto se ha encargado de modificar el brazo robótico del Spot Robot de Boston Dynamics para aumentarle una articulación prismática, obteniendo un robot manipulador de 7 grados de libertad para amplificar el espacio operacional y alcanzable. Se definieron nuevas dimensiones y en base a ellas, se obtuvieron los parámetros Dehavit - Hartenberg. El robot modificado fue modelado mediante Inventor, generando un modelo enmallado que fue exportado como archivo formato URDF para utilizarse en ROS. Fueron definidas tres configuraciones de prueba que fueron visualizadas mediante Rviz para cinemática directa, cinemática inversa de posición y control cinemático de posición. También, fue obtenido el modelo dinámico del robot a través de la librería RBDL y fue empleado para dos esquemas de control dinámico. Tanto el control cinemático como dinámico logró seguir las referencias con una norma de error menor a 0.0001.

![alt text](https://github.com/MarceloContreras/FR_proyecto/blob/main/DH_Axes.png)
![alt text](https://github.com/MarceloContreras/FR_proyecto/blob/main/Spot_fusion.png)
![alt text](https://github.com/MarceloContreras/FR_proyecto/blob/main/Modelo_Rviz.png)



