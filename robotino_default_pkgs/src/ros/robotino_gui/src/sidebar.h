#ifndef SIDEBAR_H
#define SIDEBAR_H

#include <QAction>
#include <QWidget>
//#include "Teleop_Window/teleop_window.h"


class SideBar : public QWidget {
  Q_OBJECT
public:
  explicit SideBar(QWidget *parent = nullptr);
  void addAction(QAction *action);
  QAction *addAction(const QString &text, const QIcon &icon = QIcon());
  QSize minimumSizeHint() const;
    QList<QAction *> mActions;

  

signals:

public slots:

protected:
  void paintEvent(QPaintEvent *event);
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void leaveEvent(QEvent *event);

  QAction *actionAt(const QPoint &at);

private:
  QAction* teleopAc;
  QAction *mCheckedAction;
  QAction *mOverAction;
};

#endif // SIDEBAR_H