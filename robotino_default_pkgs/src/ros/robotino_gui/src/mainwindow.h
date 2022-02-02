#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QAction>
#include <QTimer>
#include <QEvent>
#include "Manual_Window/manual_window.h"
#include "Teleop_Window/teleop_window.h"
#include "Help_Window/help_window.h"
#include "sidebar.h"

#include <QProcess>

//ros
#include "ros/ros.h"
#include "../src/ROS_src/_ros.h"
namespace Ui {
class MainWindow;
}
class MainWindow : public QMainWindow
{
Q_OBJECT
public:

    explicit MainWindow(QWidget *parent = nullptr);
    virtual ~MainWindow();
    virtual void closeEvent(QCloseEvent * e) override;

protected:
    void showEvent(QShowEvent *event);


public Q_SLOTS:
    void pose_values();
    void on_Calibration_clicked();
    void on_Exit_clicked();
    void on_RViz_clicked();
    void on_Teleop_clicked();
    void on_Aktualisieren_clicked();
    void doTeleop();
    void doCalibration();
    void doRviz();
    void doExit();
    void showPositions();
    void on_sendButton_clicked();
    void itemDoubleClicked(QListWidgetItem* listWidgetItem);
    void doHelp();


/*/private slots: Note if you want to create function from mainwindow.ui delete
 * private slots and put them in public QSLOTS
 * */


private:
    Ui::MainWindow *ui;
    _Ros* ros_f;
    QAction* teleOp;
    QAction* positionAct;
    QAction* calibrationAct;
    QAction* rvizAct;
    QAction* exitAct;
    QAction* helpAct;

    QTimer* timer;
    QProcess* start_csv;
    QProcess* start_teleop;
    QProcess* start_rviz;
    QProcess* kill_ros;
    Manual_Window* manual_window;
    SideBar* sidebar;
    Teleop_Window* teleop_window;	
    Help_Window* help_window;

    std_msgs::String goal_input;
    QListWidgetItem* listWidgetItem;
    
    geometry_msgs::Twist twist_input;

    const char* help_message = 
    
        "Folgen Sie folgenden Schritte für eine erfolgreiche Einrichtung\n"
        "und Bedienung des Coffeebots: \n"
        "1. Schritt: Sind keine Positionen gespeichert muss der Roboter\n"
        "   erstmals darauf kalibriert werden. Klicken sie hierzu auf \"Kalibration\"\n"
        "   in der Seitenleiste.\n"
        "2. Schritt: Folgend können Sie dann mit einen klick auf \"Teleoparation\"\n"
        "   den Roboter selbst steuern und an die gewünschte Position führen. Oder\n"
        "   Sie klicken auf \"RViz\", womit eine Karte geöffnet wird. Hier können Sie\n"
        "   ebenfalls mithilfe des Tools \"2D Nav Goal\" in der Menüleiste von RViz\n"
        "   rechts oben den Roboter auf eine gewünschte Position auf der Karte schicken.\n"
        "3. Schritt: Ist der Roboter auf der gewünschten Position geben Sie einen\n"
        "   Namen in das Feld des Kalibrationsfenstern für die Position ein und\n"
        "   drücken Sie \"Save!\".\n"
        "4. Schritt: Haben Sie alle gewünschten Positionen können Sie das \n"
        "   Kalibrationsfenster und falls gewünscht die anderen Fenster schließen.\n"
        "5. Schritt: Drücken Sie auf \"Aktualisieren\", damit die neuen Positionen\n"
        "   übernommen werden.\n"
        "6. Schritt: Doppelklicken Sie eine Position aus der Liste, um den Roboter \n"
        "   dorthin zu schicken oder geben Sie die Position ein und klicken\n"
        "   Sie auf \"Send\"!\n\n"

        "Zum Schließen der Anwendung alle Fenster schließen und auf Exit klicken \n";


};

#endif // MAINWINDOW_H
