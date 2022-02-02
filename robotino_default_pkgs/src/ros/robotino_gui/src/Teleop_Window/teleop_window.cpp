#include "teleop_window.h"
#include "ui_teleop_window.h"
#include <QMessageBox>
#include <QPixmap>
#include <QMatrix>
#include <QDebug>
#include <QWidget>
#include <QString>
#include <iostream>




Teleop_Window::Teleop_Window(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Teleop_Window)
{   
    ui->setupUi(this);
    
    this->setWindowTitle("Coffeebot Teleoparation ");
    
    ros_f = new _Ros();
    //setAttribute(Qt::WA_TranslucentBackground);

    qDebug() << "Coffeebot Teleoparation" ;
}

Teleop_Window::~Teleop_Window()
{   
    delete ui; delete ros_f;
}

void Teleop_Window::showEvent(QShowEvent *event) {
    QDialog::showEvent(event);
    QTimer::singleShot(50, this, SLOT(rotateLabel()));
    return;
}

void Teleop_Window::on_info_button_clicked()
{
    QMessageBox::about(this,"Control Info ",info_message);
}

void Teleop_Window::rotateLabel(){
   
    QPixmap pm_stop(":/resources/images/stop.png");
    QPixmap pm_arrow(":/resources/images/arrow.png");

    QMatrix rmforward;
    rmforward.rotate(270);
    ui->forward_label->setPixmap(pm_arrow.transformed(rmforward));
    ui->forward_label->setScaledContents(true);
    ui->forward_label->setFixedSize(100,100);

    ui->right_label->setPixmap(pm_arrow);
    ui->right_label->setScaledContents(true);
    ui->right_label->setFixedSize(100,100);

    QMatrix rmleft;
    rmleft.rotate(180);
    ui->left_label->setPixmap(pm_arrow.transformed(rmleft));
    ui->left_label->setScaledContents(true);
    ui->left_label->setFixedSize(100,100);
    QMatrix rmback;
    rmback.rotate(90);
    ui->back_label->setPixmap(pm_arrow.transformed(rmback));
    ui->back_label->setScaledContents(true);
    ui->back_label->setFixedSize(100,100);

    ui->stop_label->setPixmap(pm_stop);
    ui->stop_label->setScaledContents(true);
    ui->stop_label->setFixedSize(100,100);


}


//Takes user's from keyboard for teleoperation
void Teleop_Window::keyPressEvent(QKeyEvent *event){
    switch(event->key()) {
        case Qt::Key_8:
            //forward
            if (vel_linear<0.0){
                vel_linear=0.0;
            }
            if(vel_linear<0.1){
                vel_linear=vel_linear+0.01;
            }
            twist_input.linear.x = vel_linear;
            twist_input.linear.y=0.0;
            twist_input.linear.z=0.0;
            twist_input.angular.z = 0.0;
            ros_f->set_twist(twist_input);
            
            ui->forward_label->setStyleSheet("QLabel { background-color : white; color : black; }");
            break;
        case Qt::Key_2:
            //backward
            if (vel_linear>0.0){
                vel_linear=0.0;
            }
            if(vel_linear>-0.1){
                vel_linear=vel_linear-0.01;
            }
            twist_input.linear.x = vel_linear;
            twist_input.linear.y=0.0;
            twist_input.linear.z=0.0;
            twist_input.angular.z = 0.0;
            ros_f->set_twist(twist_input);
            //ros_f->twist_publisher();
            ui->back_label->setStyleSheet("QLabel { background-color : white; color : black; }");
            break;
        case Qt::Key_6:
            //right
            if (vel_angular>0.0){
                vel_angular=0.0;
            }
            if(vel_angular>-1.0){
                vel_angular=vel_angular-0.05;
            }
            twist_input.angular.z = vel_angular;
            twist_input.angular.y=0.0;
            twist_input.angular.x=0.0;
            twist_input.linear.x = 0.0;

            ros_f->set_twist(twist_input);
            //ros_f->twist_publisher();
            ui->right_label->setStyleSheet("QLabel { background-color : white; color : black; }");
            break;
        case Qt::Key_4:
            //left
            if (vel_angular<0.0){
                vel_angular=0.0;
                
            }
            if(vel_angular<1.0){
                vel_angular=vel_angular+0.05;
            }
            twist_input.angular.z = vel_angular;
            twist_input.angular.y=0.0;
            twist_input.angular.x=0.0;
            twist_input.linear.x = 0.0;
            ros_f->set_twist(twist_input);
            //ros_f->twist_publisher();
            ui->left_label->setStyleSheet("QLabel { background-color : white; color : black; }");
            break;
        case Qt::Key_5:
            //stop
            vel_angular=0;
            vel_linear=0;
            twist_input.angular.z=vel_angular;
            twist_input.linear.x=vel_linear;
            ros_f->set_twist(twist_input);
            //ros_f->twist_publisher();
            ui->stop_label->setStyleSheet("QLabel { background-color : white; color : black; }");
            break;
        }
    ros_f->twist_publisher();      
}
//Keyboard inputs released
void Teleop_Window::keyReleaseEvent(QKeyEvent *event){
    switch(event->key()) {
        case Qt::Key_8:
            //forward
            ui->forward_label->setStyleSheet("");
            //twist_input.linear.x = vel_linear;
            //ros_f->set_twist(twist_input);

            break;
        case Qt::Key_2:
            //backward
            ui->back_label->setStyleSheet("");
            //twist_input.linear.x = vel_linear;
            //ros_f->set_twist(twist_input);
            break;
        case Qt::Key_6:
            //right
            //twist_input.angular.z = vel_angular;
            //ros_f->set_twist(twist_input);
            ui->right_label->setStyleSheet("");
            break;
        case Qt::Key_4:
            //left
            //twist_input.angular.z = vel_angular;
            //ros_f->set_twist(twist_input);
            
            ui->left_label->setStyleSheet("");
            break;
        case Qt::Key_5:
            //stop
            ui->stop_label->setStyleSheet("");
            break;
        }

  

}

