# anro_manipulator
Paczka stworzona na potrzeby trzecich laboratoriów. Znajdują się w niej Node'y do liczenia kinematyki prostej manipulatora.
## Usage  
- `NONKDL_DKIN`  
  Skrypt ten pobiera informacje o zadanych kątach z tematu `/joint_states`. Pobiera również plik `manipulator.json` z paczki `anro_manipulator`. Na podstawie tych informacji z użyciem złożonej macierzy DH liczy on pozycję i orientację ostatniego stawu czyli połączenia narzędzia z trzecim ramieniem. Publikuje tą informację na temacie `/PoseStamped2` z użyciem wiadomości w formacie `PoseStamped`.
  ![RQT_PLOT](https://github.com/pw-eiti-anro-21l/brzezinski_nowakowski/blob/Kacper3/anro_manipulator/docs/nonkdl_plot.png?raw=true)
  ![MODEL](https://github.com/pw-eiti-anro-21l/brzezinski_nowakowski/blob/Kacper3/anro_manipulator/docs/nonkdl_test.png?raw=true)
- `KDL_DKIN`  
  Skrypt ten działa podobnie jak `NONKDL_DKIN` co do treści subskrybowanych i publikowanych. Różni się tylko publikowanym tematem - w tym wypadku jest to `/PoseStamped`. Jednakże mechanika jest zupełnie inna. Pobieramy względne współrzędne i orientacje kolejnych stawów z pliku `manipulator.yaml`. Następnie za pomocą biblioteki `PyKDL` generujemy łańcuch kinematyczny, którego kolejne segmenty są tworzone na podstawie pobranych współrzędnych. Na końcu używamy rekursywnego solvera kinematyki prostej, który zwraca nam współrzędne i orientację ostatniego stawu.
### Launch files
- `fk.launch.py`  
  Uruchamia jednocześnie: 
  - `state_publisher`, `NONKDL_DKIN`, `KDL_DKIN` z paczki `anro_fk`
  - `robot_state_publisher` z paczki `robot_state_publisher`
  - `rviz2` z paczki `rviz2`  

  Do `robot_state_publisher` zostaje przekazany plik `manipulator.urdf.xml` z paczki `anro_manipulator`.   
  Do `rviz2` zostaje przekazany plik konfiguracyjny `manipulator.rviz` z paczki `anro_fk`.
  ```
  ros2 launch anro_fk fk.launch.py
  ```  
  ![RQT_GRAPH](https://github.com/pw-eiti-anro-21l/brzezinski_nowakowski/blob/Kacper3/anro_manipulator/docs/lab3_graph.png?raw=true)
  ![RQT_PLOT](https://github.com/pw-eiti-anro-21l/brzezinski_nowakowski/blob/Kacper3/anro_manipulator/docs/kdl_plot.png?raw=true)
  ![MODEL](https://github.com/pw-eiti-anro-21l/brzezinski_nowakowski/blob/Kacper3/anro_manipulator/docs/kdl_test.png?raw=true)
## Topics
### Published topics
- `/joint_states`
- `/PoseStamped`
- `/PoseStamped2`
### Subscribed topics
- `/joint_states`

### Konfiguracja RVIZ2 - `manipulator.rviz`
Domyślna konfiguracja RVIZ2 wyświetlająca model manipulatora oraz pozycje publikowane przez `NONKDL_DKIN` oraz `KDL_DKIN`  .

### Autorzy
- Gabriel Brzeziński (gabrysbrzezinski@gmail.com)  
- Kacper Nowakowski (casperus99@wp.pl) 
