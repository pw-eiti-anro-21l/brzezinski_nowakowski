# anro_manipulator
Paczka stworzona na potrzeby drugich laboratoriów. Znajduje się w niej prosty Node do sterowania turtlesim za pomocą klawiatury
## Usage
### dh_converter.py
  ```
  python3 dh_converter.py -i=<INPUT_PATH> -o=<OUTPUT_PATH>
  ```
### Launch files
- `manipulator.launch.py`  
  Uruchamia `state_publisher` z paczki `anro_manipulator`
  ```
  ros2 launch anro_manipulator manipulator.launch.py
  ```
- `rviz_fixed.launch.py`  
  Uruchamia jednocześnie `robot_state_publisher` z paczki `robot_state_publisher` oraz `rviz2` z paczki `rviz2`. Do `robot_state_publisher` zostaje przekazany plik `manipulator.fixed.urdf.xml` z paczki `anro_manipulator`
  ```
  ros2 launch anro_manipulator rviz_fixed.launch.py
  ```
  ![RQT_GRAPH](docs/rviz_fixed_launch_rqt_graph.png)
- `rviz.launch.py`  
  Uruchamia jednocześnie `robot_state_publisher` z paczki `robot_state_publisher` oraz `rviz2` z paczki `rviz2`. Do `robot_state_publisher` zostaje przekazany plik `manipulator.urdf.xml`. Jeżeli argument `fixed` ustawiono na `true`, to do `robot_state_publisher` zostaje wtedy przekazany plik `manipulator.fixed.urdf.xml`. Do `rviz2` zostaje przekazany plik konfiguracyjny `manipulator.rviz`.
  ```
  ros2 launch anro_manipulator rviz.launch.py
  ```
  ![RQT_GRAPH](docs/rviz_launch_rqt_graph.png)
## Topics
### Published topics
- `/joint_states`
- `/tf`

## URDF
### Tabela D-H
- To zwykły plik tekstowy, który jest zapisany w formie słownika, którego jedyną wartością jest lista kolejnych słowników. 
Ich klucze oznaczają odpowiednie kolumny a kolejne słowniki odpowiadają za kolejne wiersze macierzy D-H.
- Nasza tabela D-H wygląda następująco:  
  |   | a<sub>i-1</sub> | α<sub>i-1</sub> | d<sub>i</sub> | Φ<sub>i</sub> |
  |---|-----------------|-----------------|---------------|---------------|
  | 1 | 0               | 0               | d<sub>1</sub> | Φ<sub>1</sub> |
  | 2 | 0               | 0               | d<sub>2</sub> | 0             |
  | 3 | 0               | -90             | 0             | Φ<sub>2</sub> |
  | 4 | a<sub>2</sub>   | 0               | 0             | Φ<sub>3</sub> |
  | 5 | a<sub>3</sub>   | 0               | 0             | Φ<sub>4</sub> |
  | 6 | a<sub>4</sub>   | 0               | 0             | 0             |

### Konwerter
- Jest to plik z rozszerzeniem `.py`, który odpowiada za wygenerowanie współrzędnych "xyz" oraz "rpy" wszystkich stawów i złącz oraz długości tych złącz poprzez 
  tworzenie/nadpisywanie pliku `TabDH.yaml`
### Tworzenie modelu
- Z wygenerowanie modelu manipulatora odpowiada plik `manipulator.fixed.xacro.xml` Pobiera on parametry z pliku `TabDH.yaml` za pomocą następujących komend:
  ```
  <xacro:property name="package" value="anro_manipulator" />
  <xacro:property name="params" value="${load_yaml('TabDH.yaml')}" />
  ```
  A następnie korzystamy z nich w taki sposób:
  ```
  <origin xyz="${params['link2'][0]}" rpy="${params['link2'][1]}" />
  .
  .
  .
  <origin xyz="${params['joint2'][0]}" rpy="${params['joint2'][1]}"/>
  ```
- To czy plik URDF wygeneruje się ze stawami ruchomymi czy też stałymi zależy od parametru z jakim uruchomimy plik `.xacro`. 
  Wtedy odpowiedni parametr makra przyjmie należną wartość zmieniając typ stawów z "fixed" na "revolute":
  ```
  <xacro:arg name="fixed" default="true"/>
  <xacro:property name="continuous" value="revolute"/>
  <xacro:if value="$(arg fixed)">
    <xacro:property name="continuous" value="fixed"/>
  </xacro:if>
  ```
### Autorzy
- Gabriel Brzeziński (gabrysbrzezinski@gmail.com)  
- Kacper Nowakowski (casperus99@wp.pl) 
