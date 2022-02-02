#ifndef MANUAL_WINDOW_H
#define MANUAL_WINDOW_H
#include <QDialog>
#include <QEvent>
#include <QKeyEvent>
#include "../ROS_src/_ros.h"
#include <QListWidget> 
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <QProcess>
#include <QWidget>


namespace Ui {
class Manual_Window;
}

class Manual_Window : public QDialog
{
    Q_OBJECT

public:
    explicit Manual_Window(QWidget *parent = nullptr);
    virtual ~Manual_Window() override;
    virtual void closeEvent(QCloseEvent * e) override;
    
public Q_SLOTS:
    void on_savepose_command_clicked();

private:
    Ui::Manual_Window *ui;
    _Ros* ros_f;
    QProcess* kill_process = new QProcess;
    geometry_msgs::Twist twist_input;
    std_msgs::String pos_input;
    std_msgs::Bool closed;


};

#endif // MANUAL_WINDOW_H
