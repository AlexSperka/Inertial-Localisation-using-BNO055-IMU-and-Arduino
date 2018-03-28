# 3D-Localisation-IMU-3D-Lokalisierung-IMU
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
