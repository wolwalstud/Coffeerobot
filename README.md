# coffeebot

Dieses Projekt wurde im Rahmen einer Masterarbeit erstellt. Es handelt sich um einen mechatronisch erweiterten Roboter, welcher mithilfe eines GUI auf verschiedene Positionen programmiert werden kann und diese Positionen auf Befehl hin anfährt. Zusätzlich ist auf dem Roboter eine Kaffeemaschine montiert, mit welchen der Nutzer in Genuss von frischen Kaffee kommt.

Dieses System wurde erstellt um eine qualitative Umfrage durchzuführen, welche sich mit den Reaktionen von Mitarbeiter gegenüber dem Roboter auseinandersetzt.(Welche Reaktionen zeigen die Mitarbeiter gegenüber dem Roboter in ihrem Arbeitsumfeld und welche Einstellungen, Motive oder kognitiven Konzepte hinterliegen potentiell diesen Reaktionen?)

Die Pakete in diesem Repository enthalten sowohl das GUI, als auch die nötigen Treiber den Roboter anzusteuern.

Starten der Anwendung:

Ist eine Verbindung zum Roboter hergestellt (roscore muss am Roboter laufen) kann die Anwendung mit folgendem Bashskript gestartet werden:

./Coffeebot.sh

Ist kein Roboter vorhanden kann eine Simulation gestartet werden mithilfe des Bashskripts:

./Coffeebot_sim.sh


![alt text](https://github.com/wolwalstud/coffeebot/blob/main/startside.png)

![alt text](https://github.com/wolwalstud/coffeebot/blob/main/gui.png)

![alt text](https://github.com/wolwalstud/coffeebot/blob/main/kalibration.png)
