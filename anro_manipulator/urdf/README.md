# URDF
W tym folderze znajdują się wszystkie pliki odpowiadające za wygenerowanie modelu manipulatora
## Tabela D-H
- To zwykły plik tekstowy, który jest zapisany w formie słownika, którego jedyną wartością jest lista kolejnych słowników. 
Ich klucze oznaczają odpowiednie kolumny a kolejne słowniki odpowiadają za kolejne wiersze macierzy D-H.
- Nasza tabela D-H wygląda następująco:
![D-H MATRIX](./docs/DHmatrix.png)
- Podanie niezerowych wartości w niestosownych miejscach może spowodować błędne wczytanie modelu.
## Konwerter
- Jest to plik z rozszerzeniem `.py`, który odpowiada za wygenerowanie współrzędnych "xyz" oraz "rpy" wszystkich stawów i złącz oraz długości tych złącz poprzez 
  tworzenie/nadpisywanie pliku `TabDH.yaml`
## Tworzenie modelu
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
  
