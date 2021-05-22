# anro_interpolation
Paczka stworzona na potrzeby piątych laboratoriów. Znajdują się w niej Node'y do liczenia kinematyki odwrotnej manipulatora.
## Usage
### Nodes
- `ikin.py`
  Pobiera położenie i orientację z topicu `/PoseStamped`, a następnie liczy pożądane wartości poszczególnych stawów oraz publikuje je na topicu `/joint_states`.
- `oint.py`  
  Przyjmuje kolejno argumenty: współrzędna x, współrzędna y, współrzędna z, pitch, czas, rodzaj interpolacji. Na ich podstawie wysyła żądanie do serwisu `interpolacja_operacyjna`.
  ```
  ros2 run anro_ik oint ${x} ${y} ${z} ${pitch} ${time} ${interpolation type}
  ```
  Przykład:
  ```
  ros2 run anro_ik oint 0.5 0.5 0.5 90 1 lin
  ```
### Launch files
- `ik.launch.py`  
  Uruchamia `ikin` z paczki `anro_ik` oraz `oint_control_srv` z paczki 'anro_interpolation`
  ```
  ros2 launch anro_ik ik.launch.py
  ```
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
### Service client
- `/interpolacja_operacyjna`

### Autorzy
- Gabriel Brzeziński (gabrysbrzezinski@gmail.com)  
- Kacper Nowakowski (casperus99@wp.pl) 
