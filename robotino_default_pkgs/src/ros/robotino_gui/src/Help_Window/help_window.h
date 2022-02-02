#ifndef HELP_WINDOW_H
#define HELP_WINDOW_H
#include <QWidget>
#include <QDialog>


namespace Ui {
class Help_Window;
}

class Help_Window : public QDialog
{
    Q_OBJECT

public:
    explicit Help_Window(QWidget *parent = nullptr);
    virtual ~Help_Window() override;
    
public Q_SLOTS:
    

private:
    Ui::Help_Window *ui;


};

#endif // HELP_WINDOW_H
