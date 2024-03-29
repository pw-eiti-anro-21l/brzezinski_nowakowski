# anro_ik
Paczka stworzona na potrzeby piątych laboratoriów. Znajdują się w niej Node'y do liczenia kinematyki odwrotnej manipulatora.
## Usage
### Nodes
- `ikin.py`
  Pobiera położenie i orientację z topicu `/PoseStamped`, a następnie liczy pożądane wartości poszczególnych stawów oraz publikuje je na topicu `/joint_states`.
  Interpolacja wielomianowa:
  ![RQT_PLOT_IK_FK](docs/rqt_plot_ik_fk.png)
- `oint.py`  
  Przyjmuje kolejno argumenty: współrzędna x, współrzędna y, współrzędna z, pitch, czas, rodzaj interpolacji. Na ich podstawie wysyła żądanie do serwisu
  `interpolacja_operacyjna`.
  ```
  ros2 run anro_ik oint ${x} ${y} ${z} ${pitch} ${time} ${interpolation type}
  ```
  Przykład:
  ```
  ros2 run anro_ik oint 0.5 0.5 0.5 90 1 lin
  ```
  ![OINT](https://media.githubusercontent.com/media/pw-eiti-anro-21l/brzezinski_nowakowski/main/anro_ik/docs/oint.png)
  
- `oint2.py` 
  Przyjmuje kolejno argumenty: długość boku/wielka półoś a,  długość boku/mała półoś b, czas time, rodzaj trajektorii trajectory. Na ich podstawie wysyła żądanie do serwisu
  `interpolacja_operacyjna`.
  ```
  ros2 run anro_ik oint ${a} ${b} ${time} ${trajectory}
  ```
  Przykłady:
  ```
  ros2 run anro_ik oint 0.2 0.2 10 rec
  ```
  ![RECTANGLE](https://github.com/pw-eiti-anro-21l/brzezinski_nowakowski/blob/main/anro_ik/docs/rectangle.png?raw=true)
  ```
  ros2 run anro_ik oint 0.3 0.2 20 ell
  ```
  ![ELLIPSE](https://github.com/pw-eiti-anro-21l/brzezinski_nowakowski/blob/main/anro_ik/docs/ellipse.png?raw=true)
  
### Launch files
- `ik.launch.py`  
  Uruchamia `ikin` z paczki `anro_ik` oraz `oint_control_srv` z paczki `anro_interpolation`, która odpowiada za trajektorię liniową
  
  ```
  ros2 launch anro_ik ik.launch.py
  ```
  
  ![RQT_GRAPH_IK](docs/rqt_graph_ik.png)
  
- `ik2.launch.py`  
  Uruchamia `ikin` oraz `oint_control_srv` z paczki `anro_ik`. W przeciwieństwie do `ik.launch.py` tu zadajemy prostokąt lub elipsę jako trajektorię.
  
- `rviz.launch.py`  
  Uruchamia jednocześnie `robot_state_publisher` z paczki `robot_state_publisher` oraz `rviz2` z paczki `rviz2`. Do `robot_state_publisher` zostaje przekazany plik `manipulator.fixed.urdf.xml` z paczki `anro_manipulator`. Do `rviz2` zostaje przekazany plik konfiguracyjny `manipulator.rviz` z paczki  `anro_ik`.
  ```
  ros2 launch anro_ik rviz.launch.py
  ```

## Topics
### Published topics
- `/joint_states`
- `/tf`
- `/Path`
- `/PoseStamped`
### Subscribed topics
- `/PoseStamped`

## Services
### Service server & client
- `/interpolacja_operacyjna`

### Autorzy
- Gabriel Brzeziński (gabrysbrzezinski@gmail.com)  
- Kacper Nowakowski (casperus99@wp.pl) 
