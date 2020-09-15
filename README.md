# 3D Localization IMU
Development of an inertial navigation system, integration on the bicycle, collection of sensor data, sensor data fusion and visualization

authors:
 - Jonas Dobokay
 - Berthold Rakoczy 
 - Alexander Sperka

The goal of the project work at HS Offenburg was the design, manufacturing, programming, system integration and validation of a mobile intertial measuring unit (IMU) designed with the help of commercially available components. The IMU should be able to record a driven route as accurately as possible.

An extract of the documentation, the code and the results can be found in this repository. For more detailed information please send me a mail.

Task
    - Design of an inertial navigation system
    - Integration on a vehicle (here: bicycle)
    - Collecting sensor data
    - Sensor data fusion and visualization

The used hardware consists among others of
    - Arduino Mega with Extension Shield
    - Adafruit BNO055 9-Axis Sensor Board
    - Adafruit Ultimate GPS Breakout
    - Hall sensor (US5881LUI)
    - SD Card Board

Simply put, the position in space (in quaternions) and the distance change over time (detected by a Hall sensor on the front wheel) generate a spatial vector. By keeping the time interval as small as possible (100 Hertz), the summed up vectors give an exact picture of the recorded distance travelled by the respective vehicle.
The quaternions and the Hall sensor pulses are captured by the microcontroller and stored on the SD card in the preliminary model (a radio link would certainly be useful as an outlook). 
The data on the SD card are subsequently read in and processed on the laptop using the Matlab program, and the distance travelled is visually processed.
Depending on the purpose of use and required accuracy, this data can be compared with the GPS coordinates and possibly merged with a Kalmann filter.

Extract from the conclusion:
"The display of the isolated Yaw axis can be regarded as extremely successful. Here it was possible to achieve a maximum deviation of 0.08% between start and end point (related to the entire route) for a circuit of any shape in the best case.

Please do not hesitate to contact us if you have any suggestions for improvement, questions or comments.


---

# 3D-Lokalisierung-IMU
Aufbau eines Inertialnavigationssystems, Integration am Fahrrad, Sammeln von Sensordaten, Sensordatenfusion und Visualisierung

Autoren:
 - Jonas Dobokay
 - Berthold Rakoczy 
 - Alexander Sperka

Ziel der Projektarbeit an der HS Offenburg war die Konstruktion, Fertigung, Programmierung, Systemintegration und Validierung einer Mithilfe von handelsüblichen Bauteilen entworfenen, mobilen intertialen Messeinrichtung (IMU). Diese soll eine gefahrene Strecke möglichst genau aufzeichnen können.

Ein Auszug der Dokumentation, des Codes und der Ergebnisse sehen Sie in diesem Repository. Für ausführlichere Informationen schreiben Sie mir einfach eine Mail.

Aufgabenstellung
    •	Aufbau eines Inertialnavigationssystems
    •	Integration an einem Fahrzeug (hier: Fahrrad)
    •	Sammeln von Sensordaten
    •	Sensordatenfusion und Visualisierung

Die Verwendete Hardware besteht dabei unter anderem aus:
    •	Arduino Mega mit Extension Shield
    •	Adafruit BNO055 9-Achs Sensor Board
    •	Adafruit Ultimate GPS Breakout
    •	Hall-Sensor (US5881LUI)
    •	SD Card Board

Über die Lage im Raum (in Quaternionen) und die Streckenänderung über der Zeit (mit einem Hall Sensor am Vorderrad erfasst) wird einfach gesagt ein räumlicher Vektor erzeugt. Indem man das Zeitintervall möglichst klein hält (100 Hertz), erhält man durch die aufsummierten Vektoren ein genaues Bild des aufgezeichneten Weges, den man mit dem jeweiligen Fahrzeug zurückgelegt hat.
Die Quaternionen und die Hall Sensor Impulse werden vom Mikrocontroller erfasst und im vorläufigen Modell auf die SD-Karte abgespeichert (als Ausblick wäre sicherlich eine Funkverbindung sinnvoll). 
Die Daten der SD-Karte werden nachträglich auf dem Laptop über das Programm Matlab eingelesen, verarbeitet und die gefahrene Strecke visuell aufbereitet.
Diese Daten können je nach Verwendungszweck und erfoderlichen Genauigkeit noch mit den GPS-Koordinaten abgeglichen und eventuell über einen Kalmann Filter fusioniert werden.

Auszug aus dem Fazit:
"Als  überaus  erfolgreich  kann  man  die  Darstellung  der  isolierten  Yaw  Achse  betrachten. Hierbei  ist  es  gelungen,  bei  einem   Rundkurs  beliebiger  Form  im  günstigsten  Fall  eine Maximalabweichung von 0,08% zwischen Start und Endpunkt zu erreichen (bezogen auf  die gesamte Strecke)."

Für Verbesserungsvorschlägen, Fragen oder Anregungen stehen wir gerne zur Verfügung.
