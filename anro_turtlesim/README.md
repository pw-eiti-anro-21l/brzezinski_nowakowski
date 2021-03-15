# anro_turtlesim
Paczka stworzona na potrzeby pierwszych laboratoriów. Znajduje się w niej prosty Node do sterowania turtlesim za pomocą klawiatury
## Usage
```
ros2 run anro_turtlesim turtlesim_control
```
### Launch files
- `turtlesim_control.launch.py`  
  Uruchamia jednocześnie node `turtlesim_node` z paczki `turtlesim` oraz w oddzielnej konsoli gnome-terminal node `turtlesim_control` z paczki `anro_turtlesim`
  ```
  ros2 launch anro_turtlesim turtlesimm_control.launch.py
  ```
## Topics
### Published topics
- `/turtle1/cmd_vel`
## Parameters
- `forward`  (*char*)  
  Klawisz do ruchu w przód
- `backward`  (*char*)  
  Klawisz do ruchu w tył
- `left`  (*char*)  
  Klawisz do obrotu w lewo
- `right`  (*char*)  
  Klawisz do obrotu w prawo
- `stop`  (*char*)  
  Klawisz do natychmiastowego zatrzymania
### Autorzy
- Gabriel Brzeziński (gabrysbrzezinski@gmail.com)  
- Kacper Nowakowski (casperus99@wp.pl) 