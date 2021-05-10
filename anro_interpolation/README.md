# anro_manipulator
Paczka stworzona na potrzeby drugich laboratoriów. Znajduje się w niej model manipulatora oraz prosty node sterujący pozycją manipulatora.
## Usage
### Nodes
- `jint_control_srv.py`  
  Tworzy serwis `interpolacja_konfiguracyjna` typu `Jint` z paczki `anro_msg`. Na podstawie otrzymanych kątów oraz zadanego czasu interpoluje kolejne pozycje manipulatora. Możliwa jest interpolacja wielomianowa `pol` oraz liniowa `lin`.   
  Interpolacja liniowa:
  ![RQT_JINT](docs/rqt_plot_jint_lin.png)   
  Interpolacja wielomianowa:
  ![RQT_JINT](docs/rqt_plot_jint_pol.png)
- `jint.py`  
  Przyjmuje kolejno argumenty: kąt stawu 1, kąt stawu 2, kąt stawu 3, kąt stawu 4, czas, rodzaj interpolacji. Na ich podstawie wysyła żadanie do serwisu `interpolacja_konfiguracyjna`.   
  Interpolacja liniowa:
  ![RQT_JINT](docs/rqt_plot_oint_lin.png)   
  Interpolacja wielomianowa:
  ![RQT_JINT](docs/rqt_plot_oint_pol.png)
- `oint_control_srv.py`  
  Tworzy serwis `interpolacja_operacyjna` typu `Oint` z paczki `anro_msg`. Na podstawie otrzymanego położenia oraz orientacji interpoluje kolejne pozycje. Możliwa jest interpolacja wielomianowa `pol` oraz liniowa `lin`. Położenie jest publikowane na topic'u `/PoseStamped` jako `PoseStamped` z paczki `geometry_msgs` oraz na `/Path` jako `Path` z paczki `nav_msg`.
- `oint.py`  
  Przyjmuje kolejno argumenty: współrzędna x, współrzędna y, współrzędna z, roll, pitch, yaw. Na ich podstawie wysyła żadanie do serwisu `interpolacja_operacyjna`.
### Launch files
- `jint.launch.py`  
  Uruchamia `jint_control_srv.py`
  ![RQT_JINT](docs/rviz_jint.png)
- `oint.launch.py`  
  Uruchamia `oint_control_srv.py`
  ![RQT_OINT](docs/rviz_oint.png)

## Services
### Service server
- `/interpolacja_konfiguracyjna`
- `/interpolacja_operacyjna`
### Service client
- `/interpolacja_konfiguracyjna`
- `/interpolacja_operacyjna`

### Autorzy
- Gabriel Brzeziński (gabrysbrzezinski@gmail.com)  
- Kacper Nowakowski (casperus99@wp.pl) 
