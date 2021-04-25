# anro_manipulator
Paczka stworzona na potrzeby drugich laboratoriów. Znajduje się w niej model manipulatora oraz prosty node sterujący pozycją manipulatora.
## Usage
### Konwerter D-H - `dh_converter.py`
Jest to skrypt odpowiadający za wygenerowanie współrzędnych zgodnych z urdf na podstawie tablicy D-H. Przyjmuje dwa obowiązkowe argumenty:
- `-i` lub `--Input`
  Ścieżka wejściowa do pliku json zawierającego tablicę D-H
- `-o` lub `--Output`
  Ścieżka wyjściowa w której ma zostać zapisany plik yaml zawierającego współrzędne zgodne z urdf.
  ```
  python3 dh_converter.py -i=<INPUT_PATH> -o=<OUTPUT_PATH>
  ```
### Launch files
- `[TU WSTAW NOWY LAUNCH Z MANIPULATOREM NONKDL I KDL].launch.py`  
  Uruchamia jednocześnie `state_publisher`, `NONKDL_DKIN` oraz `KDL_DKIN` z paczki `[NAZWA 3 PACZKI]`
  ```
  ros2 launch [URUCHOMIENIE LAUNCHA W 3 PACZCE]
  ```  
  ![RQT_GRAPH](https://github.com/pw-eiti-anro-21l/brzezinski_nowakowski/blob/Kacper3/anro_manipulator/docs/lab3_graph.png?raw=true)
  - `state_publisher`  
    Skrypt ten publikuje na temacie `/joint_states` wiadomości typu `JointState`, których główną zawartością jest lista zadanych kątów (w radianach) dla stawów manipulatora. Zadane kąty to     parametry w stopniach, które można zmieniać poprzez komendę `param set`.
![RQT_PLOT](https://github.com/pw-eiti-anro-21l/brzezinski_nowakowski/blob/Kacper3/anro_manipulator/docs/state_publicher_plot.png?raw=true)
![MODEL](https://github.com/pw-eiti-anro-21l/brzezinski_nowakowski/blob/Kacper3/anro_manipulator/docs/origin_test.png?raw=true)
  - `NONKDL_DKIN`  
    Skrypt ten pobiera informacje o zadanych kątach z tematu `/joint_states`. Pobiera również plik `manipulator.json` z drugiej paczki. Na podstawie tych informacji z użyciem złożonej macierzy DH liczy on pozycję i orientację ostatniego stawu czyli połączenia narzędzia z trzecim ramieniem. Publikuje tą informację na temacie `/PoseStamped2` z użyciem wiadomości w formacie `PoseStamped`.
![RQT_PLOT](https://github.com/pw-eiti-anro-21l/brzezinski_nowakowski/blob/Kacper3/anro_manipulator/docs/nonkdl_plot.png?raw=true)
![MODEL](https://github.com/pw-eiti-anro-21l/brzezinski_nowakowski/blob/Kacper3/anro_manipulator/docs/nonkdl_test.png?raw=true)
  - `KDL_DKIN`  
    Skrypt ten działa podobnie jak `NONKDL_DKIN` co do treści subskrybowanych i publikowanych. Różni się tylko publikowanym tematem - w tym wypadku jest to `/PoseStamped`. Jednakże mechanika jest zupełnie inna. Pobieramy względne współrzędne i orientacje kolejnych stawów z pliku `manipulator.yaml`. Następnie za pomocą biblioteki `PyKDL` generujemy łańcuch kinematyczny, którego kolejne segmenty są tworzone na podstawie pobranych współrzędnych. Na końcu używamy rekursywnego solvera kinematyki prostej, który zwraca nam współrzędne i orientację ostatniego stawu.
![RQT_PLOT](https://github.com/pw-eiti-anro-21l/brzezinski_nowakowski/blob/Kacper3/anro_manipulator/docs/kdl_plot.png?raw=true)
![MODEL](https://github.com/pw-eiti-anro-21l/brzezinski_nowakowski/blob/Kacper3/anro_manipulator/docs/kdl_test.png?raw=true)
- `rviz_fixed.launch.py`  
  Uruchamia jednocześnie `robot_state_publisher` z paczki `robot_state_publisher` oraz `rviz2` z paczki `rviz2`. Do `robot_state_publisher` zostaje przekazany plik `manipulator.fixed.urdf.xml`. Do `rviz2` zostaje przekazany plik konfiguracyjny `manipulator.rviz`.
  ```
  ros2 launch anro_manipulator rviz_fixed.launch.py
  ```
  ![RQT_GRAPH](https://media.githubusercontent.com/media/pw-eiti-anro-21l/brzezinski_nowakowski/main/anro_manipulator/docs/rviz_fixed_launch_rqt_graph.png)
- `rviz.launch.py`  
  Uruchamia jednocześnie `robot_state_publisher` z paczki `robot_state_publisher` oraz `rviz2` z paczki `rviz2`. Do `robot_state_publisher` zostaje przekazany plik `manipulator.urdf.xml`. Jeżeli argument `fixed` ustawiono na `true`, to do `robot_state_publisher` zostaje wtedy przekazany plik `manipulator.fixed.urdf.xml`. Do `rviz2` zostaje przekazany plik konfiguracyjny `manipulator.rviz`.
  ```
  ros2 launch anro_manipulator rviz.launch.py
  ```
  ![RQT_GRAPH](https://media.githubusercontent.com/media/pw-eiti-anro-21l/brzezinski_nowakowski/main/anro_manipulator/docs/rviz_launch_rqt_graph.png)
## Topics
### Published topics
- `/joint_states`
- `/PoseStamped`
- `/PoseStamped2`
- `/tf`
### Subscribed topics
- `/joint_states`

## URDF
W tym folderze znajdują się wszystkie pliki związane z modelem manipulatora. Dla plików rozszerzeniem `.xacro.xml` są automatycznie generowane odpowiadające im pliki `.urdf.xml`. Dla plików `.fixed.xacro.xml` dodatkowo generowany jest plik `.fixed.urdf.xml`, pliki xacro z takim rozszerzeniem powinny przyjmować opcjonalny argument `fixed`. Dla każdego pliku `.xacro.xml` kownertowany jest również plik z rozszerzeniem `.json` na plik z rozszerzeniem `.yaml`, z którego model xacro powinien wczytać parametry.
### Tablica D-H - `manipulator.json`
Tablica json do której wpisuje się kolejne wiersze tabeli D-H. Tablica ta jest automatycznie konwertowana na plik `manipulator.yaml`, z którego są wczytywane parametry do pliku `manipulator.fixed.xacro.xml`  
Tabela D-H manipulatora:  
|   | a<sub>i-1</sub> | α<sub>i-1</sub> | d<sub>i</sub> | Φ<sub>i</sub> |
|---|-----------------|-----------------|---------------|---------------|
| 1 | 0               | 0               | d<sub>1</sub> | Φ<sub>1</sub> |
| 2 | 0               | 0               | d<sub>2</sub> | 0             |
| 3 | 0               | -90             | 0             | Φ<sub>3</sub> |
| 4 | a<sub>3</sub>   | 0               | 0             | Φ<sub>4</sub> |
| 5 | a<sub>4</sub>   | 0               | 0             | Φ<sub>5</sub> |
| 6 | a<sub>5</sub>   | 0               | 0             | 0             |

Robot ma 4 stopnie swobody, drugi wiersz jest dodany na potrzeby generowania modelu urdf, a 6 wiersz dodaje narzędzie bez stopni swodoby. Zmienne Φ<sub>1</sub>, Φ<sub>3</sub>,Φ<sub>4</sub>,Φ<sub>5</sub> dodają stopinie swobody odpowiedzialne za obroty stawów. 
### Model - `manipulator.fixed.xacro.xml`
Model w formacie xacro, który jest automatycznie konwertowany na odpowiadające mu pliki urdf `manipulator.urdf.xml` oraz `manipulator.fixed.urdf.xml`. Pobiera on parametry z pliku `manipulator.yaml` oraz przyjmuje opcjonalny argument `fixed`. Dla `fixed=true` generowany plik urdf ma zablokowane stawy.
- `manipulator.urdf.xml`  
  ![Model](https://media.githubusercontent.com/media/pw-eiti-anro-21l/brzezinski_nowakowski/main/anro_manipulator/docs/Model.png)
- `manipulator.fixed.urdf.xml`  
  ![Model_fixed](https://media.githubusercontent.com/media/pw-eiti-anro-21l/brzezinski_nowakowski/main/anro_manipulator/docs/Model_fixed.png)
### Konfiguracja RVIZ2 - `manipulator.rviz`
Domyślna konfiguracja RVIZ2 wyświetlająca model manipulatora.

### Autorzy
- Gabriel Brzeziński (gabrysbrzezinski@gmail.com)  
- Kacper Nowakowski (casperus99@wp.pl) 
