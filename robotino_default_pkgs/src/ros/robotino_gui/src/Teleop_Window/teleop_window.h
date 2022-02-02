#ifndef TELEOP_WINDOW_H
#define TELEOP_WINDOW_H
#include <QDialog>
#include <string>
#include <QTimer>
#include <QEvent>
#include <QKeyEvent>
#include "../ROS_src/_ros.h"
#include <QListWidget> 
#include "std_msgs/String.h"
#include <sstream>
#include <QProcess>
#include <QtGui>

namespace Ui {
class Teleop_Window;
}

class Teleop_Window : public QDialog
{
    Q_OBJECT

public:
    explicit Teleop_Window(QWidget *parent = nullptr);
    virtual ~Teleop_Window() override;

protected:
void showEvent(QShowEvent *event);
    
    
public Q_SLOTS:
    void on_info_button_clicked();
    void rotateLabel();

private:
    //virtual functions for take the user's keyboard events
    virtual void keyPressEvent(QKeyEvent *event) override;
    virtual void keyReleaseEvent(QKeyEvent *event) override;
    Ui::Teleop_Window *ui;
    _Ros* ros_f;
    geometry_msgs::Twist twist_input;
    //ros::Rate loop_rate(10);


    float vel_linear=0.0;
    float vel_angular=0.0;

    const char* info_message = "Keyboard Teleoparation Info\n\n"

            "-Move Forward : Num 8\n\n"

            "-Move Backward: Num 2\n\n"

            "-Turn Left :  Num 4\n\n"

            "-Turn Rigth : Num 6\n\n"

            "-Stop : Num 5 \n\n"
            "****************************************************\n"
            "For better control press num lock and use the numpad ";
};

#endif // TELEOP_WINDOW_H
